// demo: CAN-BUS Shield, receive data with check mode
// send data coming to fast, such as less than 10ms, you can use this way
// loovee, 2014-6-13

#include <SPI.h>
#include "mcp_can.h"
#include <EEPROM.h>

#define ENCODER_A 3
#define ENCODER_B 4
#define MOTOR_PWM1 (5)
#define MOTOR_PWM2 (6)
#define CAN0_INT 2 // Set INT to pin 2
#define LED1_PIN (A0)
#define LED2_PIN (A1)

#define VBAT_PIN (A6)
float gAnalogReference = 5;


unsigned char Flag_Recv = 0;
unsigned char len = 0;
unsigned char buf[8];
char str[20];

long count = 0;

MCP_CAN CAN(10); // Set CS to pin 10


uint8_t gDiagSession = 0;

struct GLOBAL_PARAMS {
  uint32_t mSerial = 0;
  float mPIDkP = 10;
  float mPIDkI = 20;
  float mPIDkD = 0.3 ;
  float mPIDmax = 255;
  uint16_t mCANid = 100;
  uint8_t mNodeID = 1;
} gParam;

float gPIDtarget = 0;

void print_eeprom() {
  Serial.print("mSerial="); Serial.println(gParam.mSerial);
  Serial.print("mPIDkP ="); Serial.println(gParam.mPIDkP);
  Serial.print("mPIDkI ="); Serial.println(gParam.mPIDkI);
  Serial.print("mPIDkD ="); Serial.println(gParam.mPIDkD);
  Serial.print("mPIDmax="); Serial.println(gParam.mPIDmax);
  Serial.print("mNodeID="); Serial.println(gParam.mNodeID);
}


//#include <PIDController.h>
//PIDController pid;


void read_eeprom()
{
  for(size_t i=0; i<sizeof(gParam); i++)
  {
    *(((char*)&gParam) + i) = EEPROM.read(i);
  }
}

void update_eeprom()
{
  for(size_t i=0; i<sizeof(gParam); i++)
  {
    EEPROM.update(i, *(((char*)&gParam) + i));
  }
}

void pid_setpoint(float target)
{
  //pid.setpoint(gTargetPosition); // The "goal" the PID controller tries to "reach"
  gPIDtarget = target;
}

void set_pid()
{
  //pid.tune(mPIDkP, mPIDkI, mPIDkD); // Tune the PID, arguments: kP, kI, kD
  //pid.limit(-mPIDmax, mPIDmax);     // Limit the PID output between 0 and 255, this is important to get rid of integral windup!
}

float calc_pid(float current)
{
  //int motor_value = pid.compute(count);

  static float sIntegral;
  static float sLastError;
  static float sResult;
  static unsigned long sLastTime;
  
  if (sLastTime == millis()) return sResult;

  float error = gPIDtarget - current;
  float dt = (millis() - sLastTime) * 0.001;
  sLastTime = millis();

  sResult = gParam.mPIDkP * error; // P

  // I
  if (abs(sResult) >= gParam.mPIDmax) 
  {
    sIntegral = 0; // anti windup
  } 
  else if (sLastError * error <= 0)
  {
    sIntegral = 0; // anti windup
  }
  else 
  {
    sIntegral += error * dt;
    sResult += gParam.mPIDkI * sIntegral;
  }

  sResult += gParam.mPIDkD * (error - sLastError) / dt; // D

  // Serial.print("calc_pid: e=");
  // Serial.print(error);
  // Serial.print(" I=");
  // Serial.print(sIntegral);
  // Serial.print(" res=");
  // Serial.print(sResult);
  // Serial.println("");

  sLastError = error;
  sResult = constrain(sResult, -gParam.mPIDmax, gParam.mPIDmax);
  if (abs(sResult) < 0.1 * gParam.mPIDmax)
    sResult = 0;
  if (abs(error) < 5) 
    sResult = 0;

  return sResult;
}

bool gEncoderError = false;
void rencoder()
{
  static byte sLastA = 0;
  //static byte sLastB = 0;
  byte a = digitalRead(ENCODER_A);
  byte b = digitalRead(ENCODER_B);
  if (a == sLastA) return;

  //if ((a != sLastA) && (b != sLastB))  gEncoderError = true;

  sLastA = a;
  //sLastB = b;
  if (a == b)
  {
    count++;
  }
  else
  {
    count--;
  }
}

void can_filter()
{
  CAN.init_Mask(0, 0, 0x07000000); // 256er Block
  CAN.init_Mask(1, 0, 0x07F00000); // 16er Block
  CAN.init_Filt(3, 0, ((unsigned long)gParam.mCANid) << 16);//0x01000000); //((unsigned long)gParam.mCANid) << 16);
  CAN.init_Filt(1, 0, 0x07000000); // Diagnose
  CAN.setMode(MCP_NORMAL); // Set operation mode to normal so the MCP2515 sends acks to received data.
}

void setup()
{
  Serial.begin(115200);
  pinMode(MOTOR_PWM1, OUTPUT);
  pinMode(MOTOR_PWM2, OUTPUT);
  digitalWrite(MOTOR_PWM1, LOW);
  digitalWrite(MOTOR_PWM2, LOW);
  pinMode(VBAT_PIN, INPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);

  digitalWrite(LED1_PIN, HIGH);

START_INIT:

  if (CAN_OK == CAN.begin(MCP_STDEXT, CAN_125KBPS, MCP_16MHZ)) // init can bus : baudrate = 500k
  {
    pinMode(CAN0_INT, INPUT);        // Configuring pin for /INT input
    can_filter();

    Serial.println("CAN BUS Shield init ok!");
  }
  else
  {
    Serial.println("CAN BUS Shield init fail");
    Serial.println("Init CAN BUS Shield again");
    delay(100);
    goto START_INIT;
  }
  digitalWrite(ENCODER_A, HIGH); // turn on pullup resistor
  digitalWrite(ENCODER_B, HIGH);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), rencoder, CHANGE); // 2 is pin 18

  // pid.begin();                      // initialize the PID instance
  set_pid();
}
short gPWM = 0;

void setPwm(char mode, byte duty)
{
  static short sLast;
  short m = mode * duty;
  if (m == sLast) return;
  sLast = m;
  duty = 255 - duty; // PWM zwischen Bremsen und Fahren
  // Serial.print("SetPwm(");
  // Serial.print((int)mode);
  // Serial.print(",");
  // Serial.print(duty);
  // Serial.println(")");
  if (mode == 1) // forward
  {
    digitalWrite(MOTOR_PWM1, HIGH);
    analogWrite(MOTOR_PWM2, duty);
    gPWM = duty;
  }
  else if (mode == -1) // backward
  {
    digitalWrite(MOTOR_PWM2, HIGH);
    analogWrite(MOTOR_PWM1, duty);
    gPWM = -duty;
  }
  else if (mode == 0) // coast
  {
    digitalWrite(MOTOR_PWM1, LOW);
    digitalWrite(MOTOR_PWM2, LOW);
    gPWM = 0;
  }
  else // break
  {
    digitalWrite(MOTOR_PWM1, HIGH);
    digitalWrite(MOTOR_PWM2, HIGH);
    gPWM = 9999;
  }
}

unsigned long gTimeOut = 0;
long gTargetPosition = 0;
long gTargetSpeed = 0;
byte gMode = 0; // 0:PWM, 1:Position, 2:Speed

#define sign(x) (constrain((x), -1, 1))

void controlPosition()
{
  int motor_value = calc_pid(count);
  setPwm(sign(motor_value), abs(motor_value));

  // if (count > gTargetPosition)
  // {
  //   setPwm(-1, 255);
  // }
  // else if (count < gTargetPosition)
  // {
  //   setPwm(1, 255);
  // }
}
unsigned long gSpeedStartTime = 0;
long gSpeedStartPos = 0;

void controlSpeed()
{
  gTargetPosition = gSpeedStartPos + gTargetSpeed * 0.001 * (millis() - gSpeedStartTime);
  pid_setpoint(gTargetPosition);
  controlPosition();
  // unsigned long sLastCalc = 0;
  
  // long sLastPos = 0;
  // long speed = 1000 * (count - sLastPos) / (millis() - sLastCalc);

  // if (speed > gTargetSpeed)
  // {
  //   setPwm(-1, 255);
  // }
  // else if (speed < gTargetSpeed)
  // {
  //   setPwm(1, 255);
  // }

  // sLastPos = count;
  // sLastCalc = millis();
}
void send_diag(uint8_t sid, uint16_t pid)
{
    buf[0] = 0x00 + 3; buf[1] = sid;
    buf[2] = pid >> 8;  buf[3] = pid && 0xFF;
    buf[4] = 0; buf[5] = 0; buf[6] = 0; buf[7] = 0; 
    CAN.sendMsgBuf(0x780 + gParam.mNodeID, 0, 8, buf);
}

void send_diag8(uint8_t sid, uint16_t pid, uint8_t data)
{
    buf[0] = 0x00 + 4; buf[1] = sid;
    buf[2] = pid >> 8;  buf[3] = pid && 0xFF;
    buf[4] = data; buf[5] = 0; buf[6] = 0; buf[7] = 0; 
    CAN.sendMsgBuf(0x780 + gParam.mNodeID, 0, 8, buf);
}
void send_diag16(uint8_t sid, uint16_t pid, uint16_t data)
{
    buf[0] = 0x00 + 4; buf[1] = sid;
    buf[2] = pid >> 8;  buf[3] = pid && 0xFF;
    memcpy(buf+4, &data, 2);  buf[6] = 0; buf[7] = 0; 
    CAN.sendMsgBuf(0x780 + gParam.mNodeID, 0, 8, buf);
}

void send_diag32(uint8_t sid, uint16_t pid, uint32_t data)
{
    buf[0] = 0x00 + 7; buf[1] = sid;
    buf[2] = pid >> 8;  buf[3] = pid && 0xFF;
    memcpy(buf+4, &data, 4);
    CAN.sendMsgBuf(0x780 + gParam.mNodeID, 0, 8, buf);
}

void send_diag(uint8_t sid, uint16_t pid, float data)
{
    buf[0] = 0x00 + 7; buf[1] = sid;
    buf[2] = pid >> 8;  buf[3] = pid && 0xFF;
    memcpy(buf+4, &data, 4);
    CAN.sendMsgBuf(0x780 + gParam.mNodeID, 0, 8, buf);
}

void send_diagerr(uint8_t sid, uint8_t code)
{
    buf[0] = 0x00 + 3; 
    buf[1] = sid;
    buf[2] = sid;
    buf[3] = code;
    buf[4] = 0; buf[5] = 0; buf[6] = 0; buf[7] = 0; 
    CAN.sendMsgBuf(0x780 + gParam.mNodeID, 0, 8, buf);
}

void handle_diag(uint16_t canId)
{
  bool functional = (canId == 0x7DF);
  uint8_t tpType = buf[0] >> 4;
  uint8_t len = buf[0] & 0x0F;
  uint8_t sid = buf[1];
  uint16_t pid = (buf[2] << 8) + buf[3];
  if (tpType != 0)
  {
    send_diagerr(sid, 0x13); // Message length or format incorrect
    return;
  }
  if (sid == 0x22) // read by ID
  {
    if      (pid == 1) { send_diag32(0x62, pid, gParam.mSerial); }
    else if (pid == 2) { send_diag8(0x62, pid, gParam.mNodeID);}
    else if (pid == 3) { send_diag(0x62, pid, gParam.mPIDkP);}
    else if (pid == 4) { send_diag(0x62, pid, gParam.mPIDkI);}
    else if (pid == 5) { send_diag(0x62, pid, gParam.mPIDkD);}
    else if (pid == 6) { send_diag(0x62, pid, gParam.mPIDmax);}
    else if (pid == 7) { send_diag16(0x62, pid, gParam.mCANid);}
    else  { send_diagerr(sid, 0x12); // subfunction not supported
    }    
  }
  if (sid == 0x2E) // write by ID
  {
    if      (pid == 1) { memcpy(&gParam.mSerial, buf+4, 4); send_diag(0x6E, pid); }
    else if (pid == 2) { memcpy(&gParam.mNodeID, buf+1, 4); send_diag(0x6E, pid); }
    else if (pid == 3) { memcpy(&gParam.mPIDkP,  buf+4, 4); send_diag(0x6E, pid); } 
    else if (pid == 4) { memcpy(&gParam.mPIDkI,  buf+4, 4); send_diag(0x6E, pid); } 
    else if (pid == 5) { memcpy(&gParam.mPIDkD,  buf+4, 4); send_diag(0x6E, pid); } 
    else if (pid == 6) { memcpy(&gParam.mPIDmax, buf+4, 4); send_diag(0x6E, pid); }
    else if (pid == 7) { memcpy(&gParam.mCANid,  buf+4, 2); send_diag(0x6E, pid); }
    else  { 
      send_diagerr(sid, 0x12); // subfuncton not suport ed
      return;
    } 
    update_eeprom();
    can_filter();
  }

}

void can_loop()
{

  long unsigned int rxId;
  if (CAN_MSGAVAIL == CAN.checkReceive()) // check if data coming
  {
    CAN.readMsgBuf(&rxId, &len, buf); // read data,  len: data length, buf: data buf
    uint16_t id = 0x07FF & rxId;
    if (id == 0x07DF)
      handle_diag(id);
    if (id == ((uint16_t)0x0700 + gParam.mNodeID))
      handle_diag(id);

    if ((rxId & 0x7F0) != gParam.mCANid)
      return;
    char cmd = rxId & 0x0F;
    if (cmd <= 4)
    {
      gMode = cmd;
      word t;
      memcpy(&t, buf+4, 2);
      gTimeOut = t + millis();
    }
    if (cmd == 0) // PWM
    {
      setPwm(buf[0], buf[1]);
    }
    else if (cmd == 1) // Position
    {
      memcpy(&gTargetPosition, buf, 4);
      pid_setpoint(gTargetPosition);
    }
    else if (cmd == 2) // Speed
    {
      memcpy(&gTargetSpeed, buf, 4);
      gSpeedStartTime = millis();
      gSpeedStartPos = count;
    }
    else if (cmd == 10) // set kP & k
    {
      memcpy(&gParam.mPIDkP, buf, 4);
      memcpy(&gParam.mPIDkI, buf+4, 4);
      set_pid();
    }

    else if (cmd == 11) // set kP & k
    {
      memcpy(&gParam.mPIDkD, buf, 4);
      memcpy(&gParam.mPIDmax, buf+4, 4);
      set_pid();
    }

    // Serial.print("CAN RX ");
    // Serial.print(rxId, HEX);
    // Serial.print(": ");
    // for (int i = 0; i < len; i++) // print the data
    // {
    //   Serial.print(buf[i], HEX);
    //   Serial.print(" ");
    // }
    // Serial.println();
  }
}


void loop()
{
  rencoder();

  if (gEncoderError)
  {
    Serial.println("ENCODER ERROR");
    gEncoderError = false;
  }
  if (millis() >= gTimeOut)
  {
    setPwm(0, 0);
    gMode = 0;
    gTimeOut = -1;
    Serial.println("TIMEOUT!");
  }

  if (gMode == 1) {
    controlPosition();
  } else if (gMode == 2)  {
    controlSpeed();
  }

  can_loop();


  static unsigned long sLast10ms = 0;
  if (millis() > (sLast10ms + 10))
  {
    sLast10ms = millis();
    

    buf[0] = sign(gPWM);
    buf[1] = abs(gPWM);
    buf[2] = gMode;
    buf[3] = 0;
    memcpy(buf+4, &count, 4);
    CAN.sendMsgBuf(gParam.mCANid + 0x10, 0, 8, buf);
    return;
  }



  static unsigned long sLast100ms = 3;
  if (millis() > (sLast100ms + 100))
  {
    sLast100ms = millis();
  }


  static unsigned long sLast1s = 0;
  if (millis() > (sLast1s + 100))
  {
    sLast1s = millis();
    short v = analogRead(VBAT_PIN);
    if (v > 900)
    {
      analogReference(DEFAULT);
      gAnalogReference = 5;
    } else if (v < 180)
    {
      analogReference(INTERNAL);
      gAnalogReference = 1.1;
    }
    // Serial.print("V=");
    // Serial.print(v / 2);
    // Serial.print(" Mode=");
    // Serial.print(gMode);
    // Serial.print(" Pos=");
    // Serial.print(count);
    // Serial.print(" Target=");
    // Serial.print(gTargetPosition);
    // Serial.print(" OUT=");
    // Serial.println(gPWM);

    float vbat = (float)v * gAnalogReference * 9 / 1023;
    memcpy(buf, &vbat, 4);
    memcpy(buf+4, &gTargetPosition, 4);

    CAN.sendMsgBuf(gParam.mCANid+0x11, 0, 8, buf);
  }
}
