#ifndef FU_H_INCLUDED
#define FU_H_INCLUDED

#include <stdint.h>
#include "my_fp.h"

#ifdef __cplusplus
extern "C"
{
#endif

void fu_SetBoost(uint32_t boost);
void fu_SetWeakeningFrq(u32fp frq);
void fu_SetMaxAmp(uint32_t maxAmp);
uint32_t fu_GetAmp(u32fp frq);
uint32_t fu_GetAmpPerc(u32fp frq, uint32_t perc);

#ifdef __cplusplus
}
#endif

#endif // FU_H_INCLUDED
