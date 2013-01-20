#ifndef DIGIO_PRJ_H_INCLUDED
#define DIGIO_PRJ_H_INCLUDED

#include "hwdefs.h"

#if (HWCONFIG == HWCONFIG_TUMANAKO_KIWIAC)

#define DIG_IO_LIST \
    DIG_IO_ENTRY(dcsw_out,    GPIOC, GPIO11, MODE_OUTPUT)      \
    DIG_IO_ENTRY(err_out,     GPIOC, GPIO6,  MODE_OUTPUT)      \
    DIG_IO_ENTRY(ovtmp_out,   GPIOC, GPIO11, MODE_OUTPUT)      \
    DIG_IO_ENTRY(prec_out,    GPIOD, GPIO6,  MODE_OUTPUT)      \
    DIG_IO_ENTRY(led_out,     GPIOC, GPIO7,  MODE_OUTPUT)      \
    DIG_IO_ENTRY(on_in,       GPIOB, GPIO5,  MODE_INPUT_PD)    \
    DIG_IO_ENTRY(start_in,    GPIOD, GPIO6,  MODE_INPUT_PD)    \
    DIG_IO_ENTRY(brake_in,    GPIOG, GPIO2,  MODE_INPUT_PD)    \
    DIG_IO_ENTRY(mprot_in,    GPIOG, GPIO3,  MODE_INPUT_PD)    \
    DIG_IO_ENTRY(fwd_in,      GPIOG, GPIO10, MODE_INPUT_PD)    \
    DIG_IO_ENTRY(rev_in,      GPIOG, GPIO11, MODE_INPUT_PD)    \
    DIG_IO_ENTRY(emcystop_in, GPIOC, GPIO7,  MODE_INPUT_PD)    \
    DIG_IO_ENTRY(bms_in,      GPIOA, GPIO11, MODE_INPUT_PD)    \


#elif (HWCONFIG == HWCONFIG_OLIMEX)

#define DIG_IO_LIST \
    DIG_IO_ENTRY(dcsw_out,    GPIOC, GPIO13, MODE_OUTPUT)      \
    DIG_IO_ENTRY(err_out,     GPIOC, GPIO10, MODE_OUTPUT)      \
    DIG_IO_ENTRY(ovtmp_out,   GPIOC, GPIO11, MODE_OUTPUT)      \
    DIG_IO_ENTRY(prec_out,    GPIOB, GPIO1,  MODE_OUTPUT)      \
    DIG_IO_ENTRY(led_out,     GPIOC, GPIO12, MODE_OUTPUT)      \
    DIG_IO_ENTRY(on_in,       GPIOB, GPIO5,  MODE_INPUT_PU)    \
    DIG_IO_ENTRY(start_in,    GPIOB, GPIO6,  MODE_INPUT_PU)    \
    DIG_IO_ENTRY(brake_in,    GPIOA, GPIO2,  MODE_INPUT_PU)    \
    DIG_IO_ENTRY(mprot_in,    GPIOA, GPIO3,  MODE_INPUT_PU)    \
    DIG_IO_ENTRY(fwd_in,      GPIOA, GPIO4,  MODE_INPUT_PU)    \
    DIG_IO_ENTRY(rev_in,      GPIOC, GPIO6,  MODE_INPUT_PU)    \
    DIG_IO_ENTRY(emcystop_in, GPIOC, GPIO7,  MODE_INPUT_PU)    \
    DIG_IO_ENTRY(bms_in,      GPIOC, GPIO8,  MODE_INPUT_PU)    \

#endif

#endif // DIGIO_PRJ_H_INCLUDED
