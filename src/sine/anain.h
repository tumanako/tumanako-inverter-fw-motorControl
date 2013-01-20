#ifndef ANAIO_H_INCLUDED
#define ANAIO_H_INCLUDED


#define STM32F1
#include <libopencm3/stm32/f1/adc.h>
#include "anain_prj.h"

#define ANA_IN_ENTRY(name, port, pin) AIN_##name,
enum AnaIns
{
    ANA_IN_LIST
    ANA_IN_LAST
};
#undef ANA_IN_ENTRY

#ifdef __cplusplus
extern "C"
{
#endif

void anain_init(void);
u16  anain_get(enum AnaIns);

#ifdef __cplusplus
}
#endif


#endif // ANAIO_H_INCLUDED
