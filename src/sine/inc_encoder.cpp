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
#include "my_math.h"

#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include "errormessage.h"

#define TWO_PI 65536
#define MAX_CNT 65535
#define NUM_CTR_VAL 100

static void tim_setup();
static void dma_setup();
static int GetPulseTimeFiltered();

static volatile uint16_t timdata[NUM_CTR_VAL];
static volatile uint16_t angle = 0;
static uint32_t last_pulse_timespan = 0xffff;
static uint16_t filter = 0;
static uint32_t angle_per_pulse = 0;
static uint16_t imp_per_rev;
static bool ignore = true;
static volatile bool seenNorthSignal = false;
static uint16_t northIgnore = 0;

void Encoder::Init(void)
{
   rcc_peripheral_enable_clock(&RCC_APB1ENR, REV_CNT_RCC_ENR);
   tim_setup();
   dma_setup();
   last_pulse_timespan = MAX_CNT;
   EnableSyncMode();
}

void Encoder::EnableSyncMode()
{
	/* Configure the EXTI subsystem. */
	seenNorthSignal = false;
	exti_select_source(EXTI0, GPIOA);
	exti_set_trigger(EXTI0, EXTI_TRIGGER_RISING);
	exti_enable_request(EXTI0);
}

void Encoder::DisableSyncMode()
{
	/* Disable EXTI0 interrupt. */
	seenNorthSignal = false;
	northIgnore = 0;
	exti_disable_request(EXTI0);
}

extern "C" void exti0_isr(void)
{
	exti_reset_request(EXTI0);

	if (!northIgnore)
	{
	   angle = 0;
	   northIgnore = 2;
   }
   else
      northIgnore--;
 	gpio_toggle(GPIOC, GPIO12);
	seenNorthSignal = true;
}

/** Since power up, have we seen the north marker? */
bool Encoder::SeenNorthSignal()
{
   return seenNorthSignal;
}

void Encoder::Update()
{
   uint16_t numPulses = GetPulseTimeFiltered();

   angle += (int16_t)(numPulses * angle_per_pulse);
}

/** Returns current angle of motor shaft to some arbitrary 0-axis
 * @return angle in digit (2Pi=65536)
*/
uint16_t Encoder::GetAngle()
{
   uint32_t time_since_last_pulse = timer_get_counter(REV_CNT_TIMER);
   uint16_t interpolated_angle = ignore?0:(angle_per_pulse * time_since_last_pulse) / last_pulse_timespan;

   return angle + interpolated_angle;
}

u32fp Encoder::GetFrq()
{
   if (ignore) return 0;
   return FP_FROMINT(1000000) / (last_pulse_timespan * imp_per_rev);
}

/** Get current speed in rpm */
uint32_t Encoder::GetSpeed()
{
   if (ignore) return 0;
   return 60000000 / (last_pulse_timespan * imp_per_rev);
}

/** set number of impulses per shaft rotation
  */
void Encoder::SetImpulsesPerTurn(uint16_t imp)
{
   imp_per_rev = imp;
   angle_per_pulse = TWO_PI / imp;
}

/** set filter constant of filter after pulse counter */
void Encoder::SetFilterConst(uint8_t flt)
{
   filter = flt;
}

static void dma_setup()
{
   //We use DMA only for counting events, the values are ignored
   REV_CNT_DMA_CPAR = (uint32_t)&REV_CNT_CCR;
   REV_CNT_DMA_CMAR = (uint32_t)timdata;
   REV_CNT_DMA_CNDTR = NUM_CTR_VAL;
   REV_CNT_DMA_CCR |= DMA_CCR_MSIZE_16BIT;
   REV_CNT_DMA_CCR |= DMA_CCR_PSIZE_16BIT;
   REV_CNT_DMA_CCR |= DMA_CCR_MINC;
   REV_CNT_DMA_CCR |= DMA_CCR_CIRC;
   REV_CNT_DMA_CCR |= DMA_CCR_EN;
}

static void tim_setup()
{
   //Some explanation: HCLK=72MHz
   //APB1-Prescaler is 2 -> 36MHz
   //Timer clock source is ABP1*2 because APB1 prescaler > 1
   //So clock source is 72MHz (again)
   //We want the timer to run at 1MHz = 72MHz/72
   //Prescaler is div-1 => 71
   timer_set_prescaler(REV_CNT_TIMER, 71);
   timer_set_period(REV_CNT_TIMER, MAX_CNT);
   timer_update_on_overflow(REV_CNT_TIMER);
   timer_direction_up(REV_CNT_TIMER);

   /* Reset counter on input pulse. Filter constant must be larger than that of the capture input
      So that the counter value is first saved, then reset */
   TIM_SMCR(REV_CNT_TIMER) = TIM_SMCR_SMS_RM | TIM_SMCR_TS_ETRF | TIM_SMCR_ETP | TIM_SMCR_ETF_DTS_DIV_32_N_8;
   /* Save timer value on input pulse with smaller filter constant */
   TIM_CCMR2(REV_CNT_TIMER) = REV_CNT_CCMR2 | TIM_CCMR2_IC3F_DTF_DIV_32_N_6;
   TIM_CCER(REV_CNT_TIMER) |= REV_CNT_CCER;

   timer_enable_irq(REV_CNT_TIMER, REV_CNT_DMAEN);
   timer_set_dma_on_compare_event(REV_CNT_TIMER);

   timer_generate_event(REV_CNT_TIMER, TIM_EGR_UG);
   timer_enable_counter(REV_CNT_TIMER);
}

static int GetPulseTimeFiltered()
{
   static uint16_t lastN = NUM_CTR_VAL;
   static uint16_t encFails = 0;
   uint16_t n = REV_CNT_DMA_CNDTR;
   uint16_t measTm = REV_CNT_CCR;
   int pulses = n <= lastN?lastN - n:lastN + NUM_CTR_VAL - n;

   if (pulses > 0)
   {
      ignore = false;
      if (timer_get_flag(REV_CNT_TIMER, TIM_SR_UIF))
      {
         timer_clear_flag(REV_CNT_TIMER, TIM_SR_CC3IF);
         timer_clear_flag(REV_CNT_TIMER, TIM_SR_UIF);
         ignore = true;
         northIgnore = 0;
         seenNorthSignal = false;
      }
   }
   //Ignore pulses when time is less than quarter of the last measurement (debouncing)
   if (measTm < (last_pulse_timespan / 16) && !ignore)
   {
      pulses = 0;
      encFails++;
      if (encFails > 10)
         ErrorMessage::Post(ERR_ENCODER);
   }
   else
   {
      last_pulse_timespan = ignore?MAX_CNT:IIRFILTER(last_pulse_timespan, measTm, filter);
   }
   lastN = n;
   return pulses;
}
