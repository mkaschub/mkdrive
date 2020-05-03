// demo: CAN-BUS Shield, receive data with check mode
// send data coming to fast, such as less than 10ms, you can use this way
// loovee, 2014-6-13

#include <SPI.h>
#include "mcp_can.h"

#define ENCODER_A 3
#define ENCODER_B 4
#define MOTOR_EN (5)
#define MOTOR_PH (6)
#define MOTOR_SL (7)
#define CAN0_INT 2 // Set INT to pin 2
#define BASE_ID (0x100)

#define VBAT_PIN (A6)
float gAnalogReference = 5;


unsigned char Flag_Recv = 0;
unsigned char len = 0;
unsigned char buf[8];
char str[20];

long count = 0;

MCP_CAN CAN(10); // Set CS to pin 10



//#include <PIDController.h>
//PIDController pid;
float gPIDkP = 10;
float gPIDkI = 20;
float gPIDkD = 0.3 ;
float gPIDmax = 255;
float gPIDtarget = 0;

void pid_setpoint(float target)
{
  //pid.setpoint(gTargetPosition); // The "goal" the PID controller tries to "reach"
  gPIDtarget = target;
}

void set_pid()
{
  //pid.tune(gPIDkP, gPIDkI, gPIDkD); // Tune the PID, arguments: kP, kI, kD
  //pid.limit(-gPIDmax, gPIDmax);     // Limit the PID output between 0 and 255, this is important to get rid of integral windup!
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

  sResult = gPIDkP * error; // P

  // I
  if (abs(sResult) >= gPIDmax) 
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
    sResult += gPIDkI * sIntegral;
  }

  sResult += gPIDkD * (error - sLastError) / dt; // D

  // Serial.print("calc_pid: e=");
  // Serial.print(error);
  // Serial.print(" I=");
  // Serial.print(sIntegral);
  // Serial.print(" res=");
  // Serial.print(sResult);
  // Serial.println("");

  sLastError = error;
  sResult = constrain(sResult, -gPIDmax, gPIDmax);
  if (abs(sResult) < 0.1 * gPIDmax)
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

void setup()
{
  Serial.begin(115200);
  pinMode(MOTOR_PH, OUTPUT);
  pinMode(MOTOR_SL, OUTPUT);
  digitalWrite(MOTOR_SL, LOW);
  pinMode(VBAT_PIN, INPUT);

START_INIT:

  if (CAN_OK == CAN.begin(MCP_STDEXT, CAN_125KBPS, MCP_16MHZ)) // init can bus : baudrate = 500k
  {

    pinMode(CAN0_INT, INPUT);        // Configuring pin for /INT input
    CAN.init_Filt(0, 0, 0x01000000); //((unsigned long)BASE_ID) << 16);
    CAN.init_Mask(0, 0, 0x07F00000);
    CAN.init_Filt(1, 0, 0);
    CAN.init_Mask(1, 0, 0x07FF0000);
    CAN.setMode(MCP_NORMAL); // Set operation mode to normal so the MCP2515 sends acks to received data.

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
  // Serial.print("SetPwm(");
  // Serial.print((int)mode);
  // Serial.print(",");
  // Serial.print(duty);
  // Serial.println(")");
  if (mode == 1) // forward
  {
    digitalWrite(MOTOR_PH, LOW);
    digitalWrite(MOTOR_SL, HIGH);
    analogWrite(MOTOR_EN, duty);
    gPWM = duty;
  }
  else if (mode == -1) // backward
  {
    digitalWrite(MOTOR_PH, HIGH);
    digitalWrite(MOTOR_SL, HIGH);
    analogWrite(MOTOR_EN, duty);
    gPWM = -duty;
  }
  else if (mode == 0) // coast
  {
    digitalWrite(MOTOR_SL, LOW);
    digitalWrite(MOTOR_EN, LOW);
    gPWM = 0;
  }
  else // break
  {
    digitalWrite(MOTOR_SL, HIGH);
    digitalWrite(MOTOR_EN, LOW);
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

  long unsigned int rxId;
  if (CAN_MSGAVAIL == CAN.checkReceive()) // check if data coming
  {
    CAN.readMsgBuf(&rxId, &len, buf); // read data,  len: data length, buf: data buf
    if ((rxId & 0x7F0) != BASE_ID)
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
      memcpy(&gPIDkP, buf, 4);
      memcpy(&gPIDkI, buf+4, 4);
      set_pid();
    }

    else if (cmd == 11) // set kP & k
    {
      memcpy(&gPIDkD, buf, 4);
      memcpy(&gPIDmax, buf+4, 4);
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

  static unsigned long sLast10ms = 0;
  if (millis() > (sLast10ms + 10))
  {
    sLast10ms = millis();
    

    buf[0] = sign(gPWM);
    buf[1] = abs(gPWM);
    buf[2] = gMode;
    buf[3] = 0;
    memcpy(buf+4, &count, 4);
    CAN.sendMsgBuf(BASE_ID+0x10, 0, 8, buf);
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

    float vbat = (float)v * gAnalogReference * 10 / 1023;
    memcpy(buf, &vbat, 4);
    memcpy(buf+4, &gTargetPosition, 4);

    CAN.sendMsgBuf(BASE_ID+0x11, 0, 8, buf);
  }
}
