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
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/adc.h>
#include "anain.h"
#include "my_math.h"

#define ANA_IN_ENTRY(name, port, pin) { AnaIn::port, AnaIn::pin },
const AnaIn::AnaInfo AnaIn::ins[] =
{
   ANA_IN_LIST
   { PORT_LAST, PIN_LAST }
};

#define NUM_CHAN (sizeof(ins) / sizeof(ins[0]) - 1)
#define MEDIAN3_FROM_ADC_ARRAY(a) median3(*a, *(a + NUM_CHAN), *(a + 2*NUM_CHAN))

uint16_t AnaIn::values[NUM_SAMPLES*NUM_CHAN];

/**
* Initialize ADC hardware and start DMA based conversion process
*/
void AnaIn::Init()
{
   const struct AnaInfo *pCur;
   uint8_t channel_array[16];
   uint8_t numChan = 0;

   adc_power_off(ADC1);
   adc_enable_scan_mode(ADC1);
   adc_set_continuous_conversion_mode(ADC1);
   adc_set_right_aligned(ADC1);
   adc_set_sample_time_on_all_channels(ADC1, SAMPLE_TIME);

   adc_power_on(ADC1);
   /* wait for adc starting up*/
   for (volatile int i = 0; i < 80000; i++);

   adc_reset_calibration(ADC1);
   adc_calibrate(ADC1);

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
      channel_array[numChan] = AdcChFromPort(pCur->port, pCur->pin);
      values[numChan] = 0;
      numChan++;
   }
   adc_set_regular_sequence(ADC1, numChan, channel_array);
   adc_enable_dma(ADC1);

   DMA1_CPAR1 = (uint32_t)&ADC_DR(ADC1);
   DMA1_CMAR1 = (uint32_t)values;
   DMA1_CNDTR1 = NUM_SAMPLES * numChan;
   DMA1_CCR1 |= DMA_CCR_MSIZE_16BIT;
   DMA1_CCR1 |= DMA_CCR_PSIZE_16BIT;
   DMA1_CCR1 |= DMA_CCR_MINC;
   DMA1_CCR1 |= DMA_CCR_CIRC;
   DMA1_CCR1 |= DMA_CCR_EN;

   adc_start_conversion_regular(ADC1);
   adc_start_conversion_direct(ADC1);
}

/**
* Get filtered value of given channel
*
*  - NUM_SAMPLES = 1: Most recent raw value is returned
*  - NUM_SAMPLES = 3: Median of last 3 values is returned
*  - NUM_SAMPLES = 9: Median of last 3 medians is returned
*  - NUM_SAMPLES = 12: Average of last 4 medians is returned
*
* @param[in] in channel index
* @return Filtered value
*/
uint16_t AnaIn::Get(AnaIn::AnaIns in)
{
   #if NUM_SAMPLES == 1
   return values[in];
   #elif NUM_SAMPLES == 3
   uint16_t *curVal = &values[in];
   return MEDIAN3_FROM_ADC_ARRAY(curVal);
   #elif NUM_SAMPLES == 9
   uint16_t *curVal = &values[in];
   uint16_t med[3];

   for (int i = 0; i < 3; i++, curVal += 3*NUM_CHAN)
   {
      med[i] = MEDIAN3_FROM_ADC_ARRAY(curVal);
   }

   return median3(med[0], med[1], med[2]);
   #elif NUM_SAMPLES == 12
   uint16_t *curVal = &values[in];
   uint16_t med[4];

   for (int i = 0; i < 4; i++, curVal += 3*NUM_CHAN)
   {
      med[i] = MEDIAN3_FROM_ADC_ARRAY(curVal);
   }

   return (med[0] + med[1] + med[2] + med[3]) >> 2;
   #else
   #error NUM_SAMPLES must be 1, 3 or 9
   #endif
}

int AnaIn::median3(int a, int b, int c)
{
   int med;
   if (a > b)
   {
      if (b > c) med = b;
      else if (a > c) med = c;
      else med = a;
   }
   else
   {
      if (a > c) med = a;
      else if (b > c) med = c;
      else med = b;
   }

   return med;
}

uint8_t AnaIn::AdcChFromPort(int command_port, int command_bit)
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
