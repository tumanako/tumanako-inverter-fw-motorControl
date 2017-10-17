#ifndef ANAIN_PRJ_H_INCLUDED
#define ANAIN_PRJ_H_INCLUDED

#include "hwdefs.h"

#define NUM_SAMPLES 12
#define SAMPLE_TIME ADC_SMPR_SMP_7DOT5CYC

#define COMMON_ANA_IN_LIST \
   ANA_IN_ENTRY(throttle1, PORTC, PIN1) \
   ANA_IN_ENTRY(throttle2, PORTC, PIN0) \
   ANA_IN_ENTRY(udc,       PORTC, PIN3) \
   ANA_IN_ENTRY(tmpm,      PORTC, PIN2) \
   ANA_IN_ENTRY(tmphs,     PORTC, PIN4) \
   ANA_IN_ENTRY(uaux,      PORTA, PIN3) \
   ANA_IN_ENTRY(il1,       PORTA, PIN5)

#ifdef HWCONFIG_REV1
#define ANA_IN_LIST \
   COMMON_ANA_IN_LIST \
   ANA_IN_ENTRY(il2,       PORTA, PIN6)
#endif

#ifdef HWCONFIG_REV2
#define ANA_IN_LIST \
   COMMON_ANA_IN_LIST \
   ANA_IN_ENTRY(il2,       PORTB, PIN0)
#endif

#ifdef HWCONFIG_TESLA
#define ANA_IN_LIST \
   COMMON_ANA_IN_LIST \
   ANA_IN_ENTRY(il2,       PORTB, PIN0)
#endif


#endif // ANAIN_PRJ_H_INCLUDED
