#include <SPI.h>
#include "mcp_can.h"
#include <EEPROM.h>
#include "diag.h"

#define ENCODER_A 3
#define ENCODER_B 4
#define MOTOR_PWM1 (5)
#define MOTOR_PWM2 (6)
#define CAN0_INT 2 // INT 2 == Pin D2
#define LED1_PIN (A0)
#define LED2_PIN (A1)
#define BTN_PIN (7)
#define VBAT_PIN (A6)


// global variables 
static float gVBat = 12;
unsigned char gCanRxBuffer[8];
long gEncoderPosition = 0;
bool gEncoderError = false;
float gPIDtarget = 0;

MCP_CAN CAN(10); // CS = pin 10




void print_eeprom() {
  Serial.print("mSerial="); Serial.println(gParam.mSerial);
  Serial.print("mPIDkP ="); Serial.println(gParam.mPIDkP);
  Serial.print("mPIDkI ="); Serial.println(gParam.mPIDkI);
  Serial.print("mPIDkD ="); Serial.println(gParam.mPIDkD);
  Serial.print("mPIDmax="); Serial.println(gParam.mPIDmax);
  Serial.print("mNodeID="); Serial.println(gParam.mNodeID);
}


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
  sLastError = error;
  sResult = constrain(sResult, -gParam.mPIDmax, gParam.mPIDmax);
  if (abs(sResult) < 0.1 * gParam.mPIDmax)
    sResult = 0;
  if (abs(error) < 5) 
    sResult = 0;

  return sResult;
}

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
    gEncoderPosition++;
  }
  else
  {
    gEncoderPosition--;
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
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);

  digitalWrite(LED1_PIN, HIGH);
  digitalWrite(LED2_PIN, HIGH);

  read_eeprom();

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
}

int16_t gPWM = 0;


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
  int motor_value = calc_pid(gEncoderPosition);
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

void can_loop()
{
  long unsigned int rxId;
  uint8_t len;
  if (CAN_MSGAVAIL == CAN.checkReceive()) // check if data coming
  {
    digitalWrite(LED2_PIN, LOW);

    CAN.readMsgBuf(&rxId, &len, gCanRxBuffer); // read data,  len: data length, gCanRxBuffer: data gCanRxBuffer
    uint16_t id = 0x07FF & rxId;
    if ((id == 0x07DF) || (id == ((uint16_t)0x0700 + gParam.mNodeID)))
    {
      if (diagHandleMessage(id, gCanRxBuffer)) 
      { 
        update_eeprom();
        can_filter();
      }
    }
    if ((rxId & 0x7F0) != gParam.mCANid)
      return;
    char cmd = rxId & 0x0F;
    if (cmd <= 4)
    {
      gMode = cmd;
      word t;
      memcpy(&t, gCanRxBuffer+4, 2);
      gTimeOut = t + millis();
    }
    if (cmd == 0) // PWM
    {
      setPwm(gCanRxBuffer[0], gCanRxBuffer[1]);
    }
    else if (cmd == 1) // Position
    {
      memcpy(&gTargetPosition, gCanRxBuffer, 4);
      pid_setpoint(gTargetPosition);
    }
    else if (cmd == 2) // Speed
    {
      memcpy(&gTargetSpeed, gCanRxBuffer, 4);
      gSpeedStartTime = millis();
      gSpeedStartPos = gEncoderPosition;
    }
    else if (cmd == 10) // set kP & k
    {
      memcpy(&gParam.mPIDkP, gCanRxBuffer, 4);
      memcpy(&gParam.mPIDkI, gCanRxBuffer+4, 4);
    }

    else if (cmd == 11) // set kP & k
    {
      memcpy(&gParam.mPIDkD, gCanRxBuffer, 4);
      memcpy(&gParam.mPIDmax, gCanRxBuffer+4, 4);
    }
  }
}

void buttonLoop()
{ 
  static uint32_t sBeginPress = 0;
  if (digitalRead(BTN_PIN))
  {
    sBeginPress = 0;
  } 
  else 
  {
    if (sBeginPress == 0) { sBeginPress = millis(); }
    else if (sBeginPress == 0xFFFFFFFF) {}
    else if ((millis() - sBeginPress) >= 1000)
    {
      gDiagSession = gDiagSession == 0 ? 1 : 0;
      sBeginPress = 0xFFFFFFFF;
    }
  }
  digitalWrite(LED1_PIN, gDiagSession == 0 ? HIGH : LOW);
}


void vBatLoop()
{
  static bool gAnalogReference5V = true;
  short v = analogRead(VBAT_PIN);
  float vbat = (float)v * (gAnalogReference5V ? 5.0 : 1.1) * 11.6 / 1023;

  gVBat = 0.1 * vbat + 0.9 * gVBat;

  if (v > 900)
  {
    analogReference(DEFAULT);
    gAnalogReference5V = true;
  } 
  else if (v < 180)
  {
    analogReference(INTERNAL);
    gAnalogReference5V = false;
  }

}


void loop()
{
  rencoder();

  if (gEncoderError)
  {
    gDTC.mEncoderMisses += 1;
    gEncoderError = false;
  }

  if (millis() >= gTimeOut)
  {
    setPwm(0, 0);
    gMode = 0;
    gTimeOut = -1;
    //Serial.println("TIMEOUT!");
  }

  if (gMode == 1) {
    controlPosition();
  } else if (gMode == 2)  {
    controlSpeed();
  }

  can_loop();
  buttonLoop();

  static unsigned long sLastStatus1 = 0;
  static unsigned long sLastStatus2 = 0;
  static unsigned long sLast10ms = 0;
  static unsigned long sLast100ms = 0;
  static unsigned long sLast1s = 0;

  if (millis() > (sLastStatus1 + (uint32_t)gParam.mCycleStatus1))
  {
    sLastStatus1 = millis();


    gCanRxBuffer[0] = sign(gPWM);
    gCanRxBuffer[1] = abs(gPWM);
    gCanRxBuffer[2] = gMode;
    gCanRxBuffer[3] = 0;
    memcpy(gCanRxBuffer+4, &gEncoderPosition, 4);
    CAN.sendMsgBuf(gParam.mCANid + 0x10, 0, 8, gCanRxBuffer);

    // DEBUG MSG
    gCanRxBuffer[0] = digitalRead(BTN_PIN);
    gCanRxBuffer[1] = gDiagSession;       
    //gCanRxBuffer[2] = (uint8_t)gAnalogReference;
    memcpy(gCanRxBuffer+4, &sLastStatus1, 4);
    CAN.sendMsgBuf(gParam.mCANid + 0x0F, 0, 8, gCanRxBuffer);
  }
  else if (millis() > (sLastStatus2 + (uint32_t)gParam.mCycleStatus2))
  {
    sLastStatus2 = millis();
    memcpy(gCanRxBuffer, &gVBat, 4);
    memcpy(gCanRxBuffer+4, &gTargetPosition, 4);
    CAN.sendMsgBuf(gParam.mCANid+0x11, 0, 8, gCanRxBuffer);
  }
  else if (millis() > (sLast10ms + 100))
  {
    sLast10ms = millis();
    digitalWrite(LED2_PIN, HIGH);
  } 
  else if (millis() > (sLast100ms + 100))
  {
    sLast100ms = millis();
    vBatLoop();
  }
  else if (millis() > (sLast1s + 100))
  {
    sLast1s = millis();   
  }
}
