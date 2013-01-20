/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2010 Johannes Huebner <contact@johanneshuebner.com>
 * Copyright (C) 2010 Edward Cheeseman <cheesemanedward@gmail.com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#define STM32F1
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/f1/dma.h>
#include "anain.h"

#if NUM_SAMPLES == 1
#elif NUM_SAMPLES == 2
#define SHIFT_AM 1
#elif NUM_SAMPLES == 4
#define SHIFT_AM 2
#elif NUM_SAMPLES == 8
#define SHIFT_AM 3
#elif NUM_SAMPLES == 16
#define SHIFT_AM 4
#else
#error NUM_SAMPLES must be a multiple of 2
#endif

enum ports
{
   PORTA = 0,
   PORTB,
   PORTC,
   PORT_LAST
};

enum pins
{
   PIN0, PIN1, PIN2, PIN3, PIN4, PIN5, PIN6, PIN7,
   PIN8, PIN9,PIN10,PIN11,PIN12,PIN13,PIN14,PIN15,
   PIN_LAST
};

struct AnaIn
{
   u8 port;
   u8 pin;
};

#define ANA_IN_ENTRY(name, port, pin) { port, pin },
static const struct AnaIn ins[] =
{
   ANA_IN_LIST
   { PORT_LAST, PIN_LAST }
};

#define NUM_CHAN (sizeof(ins) / sizeof(ins[0]) - 1)

static u16 values[NUM_SAMPLES*NUM_CHAN];

static u8 adcchfromport(int command_port, int command_bit);

void anain_init(void)
{
   const struct AnaIn *pCur;
   u8 channel_array[16];
   u8 numChan = 0;

   adc_off(ADC1);
   adc_enable_scan_mode(ADC1);
   adc_set_continous_conversion_mode(ADC1);
   adc_set_right_aligned(ADC1);
   adc_set_conversion_time_on_all_channels(ADC1, ADC_SMPR_SMP_239DOT5CYC);

   adc_on(ADC1);
   /* wait for adc starting up*/
   for (volatile int i = 0; i < 80000; i++);

   adc_reset_calibration(ADC1);
   adc_calibration(ADC1);

   for (pCur = ins; pCur->port != PORT_LAST; pCur++)
   {
      switch (pCur->port)
      {
         case PORTA:
            gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, 1 << pCur->pin);
            break;
         case PORTB:
            gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, 1 << pCur->pin);
            break;
         case PORTC:
            gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, 1 << pCur->pin);
            break;
         default:
            break;
      }
      channel_array[numChan] = adcchfromport(pCur->port, pCur->pin);
      values[numChan] = 0;
      numChan++;
   }
   adc_set_regular_sequence(ADC1, numChan, channel_array);
   adc_enable_dma(ADC1);

   DMA1_CPAR1 = (u32)&ADC_DR(ADC1);
   DMA1_CMAR1 = (u32)values;
   DMA1_CNDTR1 = NUM_SAMPLES * numChan;
   DMA1_CCR1 |= DMA_CCR1_MSIZE_16BIT << DMA_CCR1_MSIZE_LSB;
   DMA1_CCR1 |= DMA_CCR1_PSIZE_16BIT << DMA_CCR1_PSIZE_LSB;
   DMA1_CCR1 |= DMA_CCR1_MINC;
   DMA1_CCR1 |= DMA_CCR1_CIRC;
   DMA1_CCR1 |= DMA_CCR1_EN;


   adc_start_conversion_regular(ADC1);
   adc_on(ADC1);
}

u16  anain_get(enum AnaIns in)
{
   #if NUM_SAMPLES == 1
   return values[in];
   #else
   uint32_t val = 0;
   u16 *curVal = &values[in];

   for (int i = 0; i < NUM_SAMPLES; i++, curVal += NUM_CHAN)
      val += *curVal;

   return (u16)(val >> SHIFT_AM);
   #endif
}

static u8 adcchfromport(int command_port, int command_bit)
{
    /*
     PA0 ADC12_IN0
     PA1 ADC12_IN1
     PA2 ADC12_IN2
     PA3 ADC12_IN3
     PA4 ADC12_IN4
     PA5 ADC12_IN5
     PA6 ADC12_IN6
     PA7 ADC12_IN7
     PB0 ADC12_IN8
     PB1 ADC12_IN9
     PC0 ADC12_IN10
     PC1 ADC12_IN11
     PC2 ADC12_IN12
     PC3 ADC12_IN13
     PC4 ADC12_IN14
     PC5 ADC12_IN15
     temp ADC12_IN16
     */
    switch (command_port)
    {
    case 0: /* port A */
        if (command_bit<8) return command_bit;
        break;
    case 1: /* port B */
        if (command_bit<2) return command_bit+8;
        break;
    case 2: /* port C */
        if (command_bit<6) return command_bit+10;
        break;
    }
    return 16;
}
