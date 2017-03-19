#ifndef INC_ENCODER_H_INCLUDED
#define INC_ENCODER_H_INCLUDED

#include <stdint.h>
#include "my_fp.h"

class Encoder
{
public:
   static void Init(void);
   static void SetMode(bool useAbzMode, bool useSyncMode);
   static void SetMinPulseTime(uint32_t time);
   static bool SeenNorthSignal();
   static void UpdateRotorAngle(int dir);
   static void UpdateRotorFrequency(int timeBase);
   static uint16_t GetRotorAngle(int dir);
   static uint32_t GetSpeed();
   static u32fp GetRotorFrequency();
   static void SetFilterConst(uint8_t flt);
   static void SetImpulsesPerTurn(uint16_t imp);
};

#endif // INC_ENCODER_H_INCLUDED
