

struct global_param_type 
{
  uint32_t mSerial = 0;
  float mPIDkP = 10;
  float mPIDkI = 20;
  float mPIDkD = 0.3 ;
  float mPIDmax = 255;
  uint16_t mCANid = 100;
  uint8_t mNodeID = 1;
  uint8_t mCycleStatus1 = 10;
  uint8_t mCycleStatus2 = 10;
};

struct error_counters 
{
  uint16_t mEncoderMisses = 0;
};

extern uint8_t gDiagSession;
extern struct global_param_type gParam;
extern struct error_counters gDTC;


/**
 * Hande received diagnosis message
 *
 * @param  canId 11 bit CAN id of received message
 * @param  buf   Pointer to the data buffer (8 bytes)
 * @return true = need to update eeprom and can filters
 */
bool diagHandleMessage(uint16_t canId, uint8_t buf[]);
