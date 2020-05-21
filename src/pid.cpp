#include <Arduino.h>
#include "diag.h"

static float gPIDtarget = 0;

void pidSetpoint(float target)
{
  gPIDtarget = target;
}

float pidGetSetpoint()
{
    return gPIDtarget;
}

float pidCompute(float current)
{
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