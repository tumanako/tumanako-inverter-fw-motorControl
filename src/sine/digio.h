#ifndef DIGIO_H_INCLUDED
#define DIGIO_H_INCLUDED

#define STM32F1
#include <libopencm3/stm32/gpio.h>
#include "digio_prj.h"

typedef enum
{
    MODE_INPUT_PD,
    MODE_INPUT_PU,
    MODE_INPUT_FLT,
    MODE_INPUT_AIN,
    MODE_OUTPUT,
    MODE_LAST
} DIG_IO_MODE;


#define DIG_IO_ENTRY(name, port, pin, mode) name,
enum DigIos
{
    DIG_IO_LIST
    DIG_IO_LAST
};
#undef DIG_IO_ENTRY

#ifdef __cplusplus
extern "C"
{
#endif

void digio_init(void);
u16  digio_get(enum DigIos);
void digio_set(enum DigIos);
void digio_clear(enum DigIos);
void digio_toggle(enum DigIos);

#ifdef __cplusplus
}
#endif

#endif // DIGIO_H_INCLUDED
