#include "fu.h"


static void CalcFac();

static uint32_t _boost;
static u32fp fac;
static uint32_t _maxAmp;
static u32fp endFrq;

/** Set 0 Hz boost to overcome winding resistance */
void fu_SetBoost(uint32_t boost /**< amplitude in digit */)
{
   _boost = boost;
   CalcFac();
}

/** Set frequency where the full amplitude is to be provided */
void fu_SetWeakeningFrq(u32fp frq)
{
   endFrq = frq;
   CalcFac();
}

/** Get amplitude for a given frequency */
uint32_t fu_GetAmp(u32fp frq)
{
   uint32_t amp = FP_TOINT(FP_MUL(fac, frq)) + _boost;
   if (0 == frq)
   {
      amp = 0;
   }
   if (amp > _maxAmp)
   {
      amp = _maxAmp;
   }
   return amp;
}

uint32_t fu_GetAmpPerc(u32fp frq, uint32_t perc)
{
   return (perc * fu_GetAmp(frq)) / 100;
}

void fu_SetMaxAmp(uint32_t maxAmp)
{
   _maxAmp = maxAmp;
   CalcFac();
}

static void CalcFac()
{
   fac = FP_DIV(FP_FROMINT(_maxAmp - _boost), endFrq);
}
