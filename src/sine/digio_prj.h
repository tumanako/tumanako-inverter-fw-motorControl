#ifndef PinMode_PRJ_H_INCLUDED
#define PinMode_PRJ_H_INCLUDED

#include "hwdefs.h"

#define COMMON_DIG_IO_LIST \
    DIG_IO_ENTRY(cruise_in,   GPIOB, GPIO5,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(start_in,    GPIOB, GPIO6,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(brake_in,    GPIOA, GPIO2,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(mprot_in,    GPIOA, GPIO3,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(fwd_in,      GPIOA, GPIO4,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(rev_in,      GPIOC, GPIO6,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(emcystop_in, GPIOC, GPIO7,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(bk_in,       GPIOB, GPIO12, PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(bms_in,      GPIOC, GPIO8,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(north_in,    GPIOD, GPIO2,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(uvlo_in,     GPIOA, GPIO0,  PinMode::INPUT_PU)   \
    DIG_IO_ENTRY(ocur_in,     GPIOA, GPIO1,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(desat_in,    GPIOC, GPIO9,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(dcsw_out,    GPIOC, GPIO13, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(vtg_out,     GPIOC, GPIO11, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(prec_out,    GPIOB, GPIO1,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(led_out,     GPIOC, GPIO12, PinMode::OUTPUT)      \


#ifdef HWCONFIG_REV1
#define DIG_IO_LIST \
   COMMON_DIG_IO_LIST \
   DIG_IO_ENTRY(err_out,     GPIOC, GPIO10, PinMode::OUTPUT)      \
   DIG_IO_ENTRY(speed_out,   GPIOC, GPIO9,  PinMode::OUTPUT)      \
   DIG_IO_ENTRY(brk_out,     GPIOA, GPIO7,  PinMode::OUTPUT)
#endif

#ifdef HWCONFIG_REV2
#define DIG_IO_LIST \
   COMMON_DIG_IO_LIST \
   DIG_IO_ENTRY(err_out,     GPIOC, GPIO10, PinMode::OUTPUT)      \
   DIG_IO_ENTRY(speed_out,   GPIOC, GPIO5,  PinMode::OUTPUT)      \
   DIG_IO_ENTRY(brk_out,     GPIOC, GPIO5,  PinMode::OUTPUT)
#endif

//In tesla route unused functions to unconnected pin
#ifdef HWCONFIG_TESLA
#define DIG_IO_LIST \
   COMMON_DIG_IO_LIST \
   DIG_IO_ENTRY(speed_out,   GPIOC, GPIO5,  PinMode::INPUT_FLT)      \
   DIG_IO_ENTRY(brk_out,     GPIOC, GPIO5,  PinMode::INPUT_FLT)      \
   DIG_IO_ENTRY(err_out,     GPIOC, GPIO5,  PinMode::INPUT_FLT)      \
   DIG_IO_ENTRY(temp1_out,   GPIOC, GPIO8,  PinMode::OUTPUT)   \
   DIG_IO_ENTRY(temp0_out,   GPIOC, GPIO10, PinMode::OUTPUT)
#endif

#endif // PinMode_PRJ_H_INCLUDED
