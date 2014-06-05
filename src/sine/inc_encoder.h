#ifndef INC_ENCODER_H_INCLUDED
#define INC_ENCODER_H_INCLUDED

#include <stdint.h>
#include "my_fp.h"

class Encoder
{
   public:
      static void Init(void);
      static void Update();
      static uint16_t GetAngle();
      static uint32_t GetSpeed();
      static u32fp GetFrq();
      static void SetFilterConst(uint8_t flt);
      static void SetImpulsesPerTurn(uint16_t imp);
};

#endif // INC_ENCODER_H_INCLUDED
