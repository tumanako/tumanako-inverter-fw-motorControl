#ifndef ANAIN_PRJ_H_INCLUDED
#define ANAIN_PRJ_H_INCLUDED

#include "hwdefs.h"

#define NUM_SAMPLES 12
#define SAMPLE_TIME ADC_SMPR_SMP_7DOT5CYC

#if (HWCONFIG == HWCONFIG_TUMANAKO_KIWIAC)

#define ANA_IN_LIST \
   ANA_IN_ENTRY(throttle1, PORTC, PIN3) \
   ANA_IN_ENTRY(throttle2, PORTC, PIN0) \
   ANA_IN_ENTRY(il1,       PORTA, PIN5) \
   ANA_IN_ENTRY(il2,       PORTA, PIN6) \
   ANA_IN_ENTRY(udc,       PORTC, PIN0) \
   ANA_IN_ENTRY(tmpm,      PORTC, PIN2) \
   ANA_IN_ENTRY(tmphs,     PORTC, PIN1) \

#elif (HWCONFIG == HWCONFIG_OLIMEX)

#define ANA_IN_LIST \
   ANA_IN_ENTRY(throttle1, PORTC, PIN1) \
   ANA_IN_ENTRY(throttle2, PORTC, PIN0) \
   ANA_IN_ENTRY(il1,       PORTA, PIN5) \
   ANA_IN_ENTRY(il2,       PORTA, PIN6) \
   ANA_IN_ENTRY(udc,       PORTC, PIN3) \
   ANA_IN_ENTRY(tmpm,      PORTC, PIN2) \
   ANA_IN_ENTRY(tmphs,     PORTC, PIN4) \


#endif



#endif // ANAIN_PRJ_H_INCLUDED
