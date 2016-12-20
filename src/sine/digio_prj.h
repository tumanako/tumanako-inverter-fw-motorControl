#ifndef PinMode_PRJ_H_INCLUDED
#define PinMode_PRJ_H_INCLUDED

#include "hwdefs.h"

#if (HWCONFIG == HWCONFIG_TUMANAKO_KIWIAC)

#define DIG_IO_LIST \
    DIG_IO_ENTRY(dcsw_out,    GPIOC, GPIO11, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(err_out,     GPIOC, GPIO6,  PinMode_OUTPUT)      \
    DIG_IO_ENTRY(ovtmp_out,   GPIOC, GPIO11, PinMode_OUTPUT)      \
    DIG_IO_ENTRY(prec_out,    GPIOD, GPIO6,  PinMode_OUTPUT)      \
    DIG_IO_ENTRY(led_out,     GPIOC, GPIO7,  PinMode_OUTPUT)      \
    DIG_IO_ENTRY(cruise_in,   GPIOB, GPIO5,  PinMode_INPUT_PD)    \
    DIG_IO_ENTRY(start_in,    GPIOD, GPIO6,  PinMode_INPUT_PD)    \
    DIG_IO_ENTRY(brake_in,    GPIOG, GPIO2,  PinMode_INPUT_PD)    \
    DIG_IO_ENTRY(mprot_in,    GPIOG, GPIO3,  PinMode_INPUT_PD)    \
    DIG_IO_ENTRY(fwd_in,      GPIOG, GPIO10, PinMode_INPUT_PD)    \
    DIG_IO_ENTRY(rev_in,      GPIOG, GPIO11, PinMode_INPUT_PD)    \
    DIG_IO_ENTRY(emcystop_in, GPIOC, GPIO7,  PinMode_INPUT_PD)    \
    DIG_IO_ENTRY(bms_in,      GPIOA, GPIO11, PinMode_INPUT_PD)    \


#elif (HWCONFIG == HWCONFIG_OLIMEX)

#define DIG_IO_LIST \
    DIG_IO_ENTRY(dcsw_out,    GPIOC, GPIO13, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(err_out,     GPIOC, GPIO10, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(vtg_out,     GPIOC, GPIO11, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(prec_out,    GPIOB, GPIO1,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(led_out,     GPIOC, GPIO12, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(cruise_in,   GPIOB, GPIO5,  PinMode::INPUT_FLT)    \
    DIG_IO_ENTRY(start_in,    GPIOB, GPIO6,  PinMode::INPUT_FLT)    \
    DIG_IO_ENTRY(brake_in,    GPIOA, GPIO2,  PinMode::INPUT_FLT)    \
    DIG_IO_ENTRY(mprot_in,    GPIOA, GPIO3,  PinMode::INPUT_FLT)    \
    DIG_IO_ENTRY(fwd_in,      GPIOA, GPIO4,  PinMode::INPUT_FLT)    \
    DIG_IO_ENTRY(rev_in,      GPIOC, GPIO6,  PinMode::INPUT_FLT)    \
    DIG_IO_ENTRY(emcystop_in, GPIOC, GPIO7,  PinMode::INPUT_FLT)    \
    DIG_IO_ENTRY(bk_in,       GPIOB, GPIO12, PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(bms_in,      GPIOC, GPIO8,  PinMode::INPUT_FLT)    \
    DIG_IO_ENTRY(speed_out,   GPIOC, GPIO9,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(brk_out,     GPIOA, GPIO7,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(north_in,    GPIOA, GPIO0,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(dir_in,      GPIOA, GPIO7,  PinMode::INPUT_FLT)   \

#endif

#endif // PinMode_PRJ_H_INCLUDED
