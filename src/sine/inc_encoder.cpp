/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2011 Johannes Huebner <dev@johanneshuebner.com>
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
#include "hwdefs.h"
#include "inc_encoder.h"

#define NUM_CTR_VAL (sizeof(timdata) / sizeof(timdata[0]))
#define TWO_PI 65536

static void dma_setup();
static void tim_setup();
static uint16_t GetPulseTimeFiltered();

static uint16_t timdata[10];
static uint16_t angle = 0;
static uint16_t last_pulse_timespan;
static uint16_t filter = 0;
static uint32_t angle_per_pulse = 0;
static uint16_t imp_per_rev;
static uint32_t overflow = 0;

void enc_init(void)
{
   rcc_peripheral_enable_clock(&RCC_APB1ENR, REV_CNT_RCC_ENR);
   dma_setup();
   tim_setup();
   last_pulse_timespan = 1 << 15;
}

/** Returns current angle of motor shaft to some arbitrary 0-axis
 * @return angle in digit (2Pi=65536)
*/
uint16_t enc_get_angle()
{
   uint32_t numPulses = GetPulseTimeFiltered();
   uint32_t time_since_last_pulse = timer_get_counter(REV_CNT_TIMER);
   uint16_t interpolated_angle = (angle_per_pulse * time_since_last_pulse) / last_pulse_timespan;

   if (overflow)
      return angle;

   angle += numPulses * angle_per_pulse;

   return angle + interpolated_angle;
}

/** Get current speed in rpm */
uint32_t enc_get_speed()
{
   return 60000000 / (last_pulse_timespan * imp_per_rev);
}

/** set number of impulses per shaft rotation
  */
void enc_set_imp_per_rev(uint16_t imp)
{
   imp_per_rev = imp;
   angle_per_pulse = TWO_PI / imp;
}

/** set filter constant of filter after pulse counter */
void enc_set_filter_const(uint8_t flt)
{
   filter = flt;
}

static void dma_setup()
{
   REV_CNT_DMA_CPAR = (u32)&REV_CNT_CCR;
   REV_CNT_DMA_CMAR = (u32)timdata;
   REV_CNT_DMA_CNDTR = NUM_CTR_VAL;
   REV_CNT_DMA_CCR |= DMA_CCR4_MSIZE_16BIT << DMA_CCR4_MSIZE_LSB;
   REV_CNT_DMA_CCR |= DMA_CCR4_PSIZE_16BIT << DMA_CCR4_PSIZE_LSB;
   REV_CNT_DMA_CCR |= DMA_CCR4_MINC;
   REV_CNT_DMA_CCR |= DMA_CCR4_CIRC;
   REV_CNT_DMA_CCR |= DMA_CCR4_EN;
}

static void tim_setup()
{
   //prescaler to 35 -> divides by 36 -> 1MHz
   timer_set_prescaler(REV_CNT_TIMER, 35);
   timer_set_period(REV_CNT_TIMER, 65535);
   timer_update_on_overflow(REV_CNT_TIMER);
   timer_direction_up(REV_CNT_TIMER);

   /* Reset counter on input pulse */
   TIM_SMCR(REV_CNT_TIMER) = TIM_SMCR_SMS_RM | TIM_SMCR_TS_ETRF | TIM_SMCR_ETP | TIM_SMCR_ETF_DTS_DIV_32_N_8;
   /* Save timer value on input pulse with smaller filter constant */
   TIM_CCMR2(REV_CNT_TIMER) = REV_CNT_CCMR2;
   TIM_CCER(REV_CNT_TIMER) = REV_CNT_CCER;

   timer_enable_irq(REV_CNT_TIMER, REV_CNT_DMAEN);
   timer_enable_irq(REV_CNT_TIMER, TIM_DIER_UIE);
   timer_set_dma_on_compare_event(REV_CNT_TIMER);

   timer_generate_event(REV_CNT_TIMER, TIM_EGR_UG);
   timer_enable_counter(REV_CNT_TIMER);
}

static uint16_t GetPulseTimeFiltered()
{
   uint16_t numData = NUM_CTR_VAL - REV_CNT_DMA_CNDTR;

   for (int i = 0; i < numData; i++)
   {
      // filtered = (newVal + ((2^C) - 1) * lastVal) / 2^C
      // x / 2^C = x >> C
      // 2^C * x = x << C
      // (2^C - 1) * x = (x << C) - x
      last_pulse_timespan = (timdata[i] + (last_pulse_timespan << filter) - last_pulse_timespan) >> filter;
      overflow = 0;
   }

   REV_CNT_DMA_CCR &= ~DMA_CCR4_EN;
   REV_CNT_DMA_CNDTR = NUM_CTR_VAL;
   REV_CNT_DMA_CCR |= DMA_CCR4_EN;

   return numData;
}

void rev_timer_isr()
{
   timer_clear_flag(REV_CNT_TIMER, TIM_SR_UIF);
   overflow = 1;
}
