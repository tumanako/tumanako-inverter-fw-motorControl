#ifndef STM32_SINE_H_INCLUDED
#define STM32_SINE_H_INCLUDED

#include <stdint.h>

/*
 * Global interface used to set inverter frequency,
 * -ve = backwards, 0 = stop, +ve = forward
*/
extern int16_t STM32_SINE_SineSpeed;

/* Used to initailise the stm32_sine module*/
void STM32_SINE_Init(void);

#endif // STM32_SINE_H_INCLUDED
