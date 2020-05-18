#include <Arduino.h>
#include "diag.h"
#include "mcp_can.h"

uint8_t gDiagSession = 0;
struct global_param_type gParam;

static uint8_t buf[8];
extern MCP_CAN CAN;

void send_diag(uint8_t sid, uint16_t pid)
{
    buf[0] = 0x00 + 3; buf[1] = sid;
    buf[2] = pid >> 8;  buf[3] = pid & 0x00FF;
    buf[4] = 0; buf[5] = 0; buf[6] = 0; buf[7] = 0; 
    CAN.sendMsgBuf(0x780 + gParam.mNodeID, 0, 8, buf);
}

void send_diag8(uint8_t sid, uint16_t pid, uint8_t data)
{
    buf[0] = 0x00 + 4; buf[1] = sid;
    buf[2] = pid >> 8;  buf[3] = pid & 0x00FF;
    buf[4] = data; buf[5] = 0; buf[6] = 0; buf[7] = 0; 
    CAN.sendMsgBuf(0x780 + gParam.mNodeID, 0, 8, buf);
}
void send_diag16(uint8_t sid, uint16_t pid, uint16_t data)
{
    buf[0] = 0x00 + 5; buf[1] = sid;
    buf[2] = pid >> 8;  buf[3] = pid & 0x00FF;
    memcpy(buf+4, &data, 2);  buf[6] = 0; buf[7] = 0; 
    CAN.sendMsgBuf(0x780 + gParam.mNodeID, 0, 8, buf);
}

void send_diag32(uint8_t sid, uint16_t pid, uint32_t data)
{
    buf[0] = 0x00 + 7; buf[1] = sid;
    buf[2] = pid >> 8;  buf[3] = pid & 0x00FF;
    memcpy(buf+4, &data, 4);
    CAN.sendMsgBuf(0x780 + gParam.mNodeID, 0, 8, buf);
}

void send_diag(uint8_t sid, uint16_t pid, float data)
{
    buf[0] = 0x00 + 7; buf[1] = sid;
    buf[2] = pid >> 8;  buf[3] = pid & 0xFF;
    memcpy(buf+4, &data, 4);
    CAN.sendMsgBuf(0x780 + gParam.mNodeID, 0, 8, buf);
}

void send_diagerr(uint8_t sid, uint8_t code)
{
    buf[0] = 0x00 + 3; 
    buf[1] = 0x7F ;
    buf[2] = sid;
    buf[3] = code;
    buf[4] = 0; buf[5] = 0; buf[6] = 0; buf[7] = 0; 
    CAN.sendMsgBuf(0x780 + gParam.mNodeID, 0, 8, buf);
}

// Return value: true = need to update eeprom and can filters 
bool handle_diag(uint16_t canId, uint8_t inbuf[])
{
  bool functional = (canId == 0x7DF);
  uint8_t tpType = inbuf[0] >> 4;
  //uint8_t len = inbuf[0] & 0x0F;
  uint8_t sid = inbuf[1];
  uint16_t pid = (inbuf[2] << 8) + inbuf[3];
  if (tpType != 0)
  {
    send_diagerr(sid, 0x13); // Message length or format incorrect
    return false;
  }
  if (sid == 0x10) // Session Control
  {
    pid = inbuf[2];
    if ((gDiagSession != 0) && (pid == 0))
    {
      gDiagSession = 0;
      send_diag(sid + 0x40, gParam.mNodeID);
    }
    else if ((pid == 2) && (!functional))
    {
      gDiagSession = 1;
      send_diag32(sid + 0x40, gParam.mNodeID, gParam.mSerial);
    }
    else if (pid == 5) 
    {
      uint32_t serial; 
      memcpy(&serial, inbuf + 4, 4);
      if(serial == gParam.mSerial)
      {
        gDiagSession = 1;
        send_diag32(sid + 0x40, gParam.mNodeID, gParam.mSerial);
      }
    }
  }
  else if (sid == 0x22) // read by ID
  {
    if      (pid == 1) { send_diag32(sid + 0x40, pid, gParam.mSerial); }
    else if (pid == 2) { send_diag8 (sid + 0x40, pid, gParam.mNodeID);}
    else if (pid == 3) { send_diag  (sid + 0x40, pid, gParam.mPIDkP);}
    else if (pid == 4) { send_diag  (sid + 0x40, pid, gParam.mPIDkI);}
    else if (pid == 5) { send_diag  (sid + 0x40, pid, gParam.mPIDkD);}
    else if (pid == 6) { send_diag  (sid + 0x40, pid, gParam.mPIDmax);}
    else if (pid == 7) { send_diag16(sid + 0x40, pid, gParam.mCANid);}
    else  { send_diagerr(sid, 0x12); // subfunction not supported
    }    
  }
  else if (sid == 0x2E)    // write by ID
  {
    if ((gDiagSession == 0) && functional)
    {
      return false;
    }
    if      (pid == 1) { memcpy(&gParam.mSerial, inbuf+4, sizeof(gParam.mSerial)); send_diag(sid + 0x40, pid); }
    else if (pid == 2) { memcpy(&gParam.mNodeID, inbuf+4, sizeof(gParam.mNodeID)); send_diag(sid + 0x40, pid); }
    else if (pid == 3) { memcpy(&gParam.mPIDkP,  inbuf+4, sizeof(gParam.mPIDkP));  send_diag(sid + 0x40, pid); } 
    else if (pid == 4) { memcpy(&gParam.mPIDkI,  inbuf+4, sizeof(gParam.mPIDkI));  send_diag(sid + 0x40, pid); } 
    else if (pid == 5) { memcpy(&gParam.mPIDkD,  inbuf+4, sizeof(gParam.mPIDkD));  send_diag(sid + 0x40, pid); } 
    else if (pid == 6) { memcpy(&gParam.mPIDmax, inbuf+4, sizeof(gParam.mPIDmax)); send_diag(sid + 0x40, pid); }
    else if (pid == 7) { memcpy(&gParam.mCANid,  inbuf+4, sizeof(gParam.mCANid));  send_diag(sid + 0x40, pid); }
    else  { 
      send_diagerr(sid, 0x12); // subfuncton not suport ed
      return false;
    }
    return true;
  } else if (!functional)
  {
    send_diagerr(sid, 0x11); // service not suport ed
  }
  return false;
}
