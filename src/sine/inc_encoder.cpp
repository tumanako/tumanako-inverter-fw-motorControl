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
#include "params.h"

#define TWO_PI 65536
#define MAX_CNT 65535
#define MAX_REVCNT_VALUES 6

static void InitTimerSingleChannelMode();
static void InitTimerABZMode();
static void DMASetup();
static int GetPulseTimeFiltered();
static void GetMinMaxTime(int& min, int& max);

static volatile uint16_t timdata[MAX_REVCNT_VALUES];
static volatile uint16_t angle = 0;
static uint32_t lastPulseTimespan = 0;
static uint16_t filter = 0;
static uint32_t anglePerPulse = 0;
static uint16_t pulsesPerTurn = 0;
static uint32_t turnsSinceLastSample = 0;
static u32fp lastFrequency = 0;
static bool ignore = true;
static bool abzMode;
static bool syncMode;
static bool seenNorthSignal = false;
static uint32_t minPulseTime = 0;

void Encoder::Init(void)
{
   rcc_peripheral_enable_clock(&RCC_APB1ENR, REV_CNT_RCC_ENR);
   abzMode = true; //Make sure SetMode does something
   SetMode(false, false);
}

/** Since power up, have we seen the north marker? */
bool Encoder::SeenNorthSignal()
{
   return seenNorthSignal;
}

/** Use pulse timing (single channel mode) or ABZ encoder mode
 * @param useAbzMode use ABZ mode, single channel otherwise
 * @param useSyncMode reset counter on first edge of ETR */
void Encoder::SetMode(bool useAbzMode, bool useSyncMode)
{
   if (useAbzMode == abzMode && useSyncMode == syncMode) return;

   abzMode = useAbzMode;
   syncMode = useSyncMode;
   if (abzMode)
      InitTimerABZMode();
   else
      InitTimerSingleChannelMode();
}

void Encoder::SetMinPulseTime(uint32_t time)
{
   minPulseTime = time;
}

/** set number of impulses per shaft rotation
  */
void Encoder::SetImpulsesPerTurn(uint16_t imp)
{
   if (imp == pulsesPerTurn) return;

   pulsesPerTurn = imp;
   anglePerPulse = TWO_PI / imp;

   if (abzMode)
      InitTimerABZMode();
}

/** set filter constant of filter after pulse counter */
void Encoder::SetFilterConst(uint8_t flt)
{
   filter = flt;
}

void Encoder::UpdateRotorAngle(int dir)
{
   if (abzMode)
   {
      static uint16_t lastAngle = 0;
      uint32_t cntVal = timer_get_counter(REV_CNT_TIMER);
      cntVal *= TWO_PI;
      cntVal /= pulsesPerTurn * 4;
      angle = (uint16_t)cntVal;

      uint16_t angleDiff = (angle - lastAngle) & 0xFFFF;
      if ((TIM_CR1(REV_CNT_TIMER) & TIM_CR1_DIR_DOWN))
         angleDiff = (lastAngle - angle) & 0xFFFF;
      lastAngle = angle;
      turnsSinceLastSample += angleDiff;
   }
   else
   {
      int16_t numPulses = GetPulseTimeFiltered();

      angle += (int16_t)(dir * numPulses * anglePerPulse);
   }
}

/** Return rotor frequency in Hz.
 * This function must be called at an interval that is short enough to obtain
 * a recent sample of the rotor frequency and long enough to allow a sufficient
 * number of pulses to be generated even at low speeds.
 * @param timeBase calling frequency in ms */
void Encoder::UpdateRotorFrequency(int timeBase)
{
   if (abzMode && timeBase > 0)
   {
      //65536 is one turn
      lastFrequency = ((1000 * turnsSinceLastSample) / timeBase) >> (16 - FRAC_DIGITS);
      turnsSinceLastSample = 0;
   }
}

/** Returns current angle of motor shaft to some arbitrary 0-axis
 * @return angle in digit (2Pi=65536)
*/
uint16_t Encoder::GetRotorAngle(int dir)
{
   if (abzMode)
   {
      return angle;
   }
   else
   {
      uint32_t timeSinceLastPulse = timer_get_counter(REV_CNT_TIMER);
      uint16_t interpolatedAngle = ignore ? (anglePerPulse * timeSinceLastPulse) / lastPulseTimespan : 0;

      return angle + dir * interpolatedAngle;
   }
}


/** Return rotor frequency in Hz
 * @pre in ABZ encoder mode UpdateRotorFrequency must be called at a regular interval */
u32fp Encoder::GetRotorFrequency()
{
   if (abzMode)
   {
      return lastFrequency;
   }
   else
   {
      if (ignore) return 0;
      return FP_FROMINT(1000000) / (lastPulseTimespan * pulsesPerTurn);
   }
}

/** Get current speed in rpm */
uint32_t Encoder::GetSpeed()
{
   if (abzMode)
   {
      return FP_TOINT(60 * lastFrequency);
   }
   else
   {
      if (ignore) return 0;
      return 60000000 / (lastPulseTimespan * pulsesPerTurn);
   }
}

static void DMASetup()
{
   //We use DMA only for counting events, the values are ignored
   dma_disable_channel(DMA1, REV_CNT_DMACHAN);
   dma_set_peripheral_address(DMA1, REV_CNT_DMACHAN, (uint32_t)&REV_CNT_CCR);
   dma_set_memory_address(DMA1, REV_CNT_DMACHAN, (uint32_t)timdata);
   dma_set_peripheral_size(DMA1, REV_CNT_DMACHAN, DMA_CCR_PSIZE_16BIT);
   dma_set_memory_size(DMA1, REV_CNT_DMACHAN, DMA_CCR_MSIZE_16BIT);
   dma_set_number_of_data(DMA1, REV_CNT_DMACHAN, MAX_REVCNT_VALUES);
   dma_enable_memory_increment_mode(DMA1, REV_CNT_DMACHAN);
   dma_enable_circular_mode(DMA1, REV_CNT_DMACHAN);
   dma_enable_channel(DMA1, REV_CNT_DMACHAN);
}

static void InitTimerSingleChannelMode()
{
   timer_reset(REV_CNT_TIMER);
   //Some explanation: HCLK=72MHz
   //APB1-Prescaler is 2 -> 36MHz
   //Timer clock source is ABP1*2 because APB1 prescaler > 1
   //So clock source is 72MHz (again)
   //We want the timer to run at 1MHz = 72MHz/72
   //Prescaler is div-1 => 71
   timer_set_prescaler(REV_CNT_TIMER, 71);
   timer_set_period(REV_CNT_TIMER, MAX_CNT);
   timer_direction_up(REV_CNT_TIMER);

   /* Reset counter on input pulse. Filter constant must be larger than that of the capture input
      So that the counter value is first saved, then reset */
   timer_slave_set_mode(REV_CNT_TIMER, TIM_SMCR_SMS_RM); // reset mode
   timer_slave_set_polarity(REV_CNT_TIMER, TIM_ET_FALLING);
   timer_slave_set_trigger(REV_CNT_TIMER, TIM_SMCR_TS_ETRF);
   timer_slave_set_filter(REV_CNT_TIMER, TIM_IC_DTF_DIV_32_N_8);

   /* Save timer value on input pulse with smaller filter constant */
   timer_ic_set_filter(REV_CNT_TIMER, REV_CNT_IC, TIM_IC_DTF_DIV_32_N_6);
   timer_ic_set_input(REV_CNT_TIMER, REV_CNT_IC, TIM_IC_IN_TI1);
   TIM_CCER(REV_CNT_TIMER) |= REV_CNT_CCER; //No API function yet available
   timer_ic_enable(REV_CNT_TIMER, REV_CNT_IC);

   timer_enable_irq(REV_CNT_TIMER, REV_CNT_DMAEN);
   timer_set_dma_on_compare_event(REV_CNT_TIMER);

   timer_generate_event(REV_CNT_TIMER, TIM_EGR_UG);
   timer_enable_counter(REV_CNT_TIMER);
   DMASetup();
}

static void InitTimerABZMode()
{
   timer_reset(REV_CNT_TIMER);
	timer_set_period(REV_CNT_TIMER, 4 * pulsesPerTurn); //2 channels and 2 edges -> 4 times the number of base pulses

	//In sync mode start out in reset mode and switch to encoder
	//mode once the north marker has been detected
	if (syncMode)
   {
      exti_select_source(EXTI2, GPIOD);
      nvic_enable_irq(NVIC_EXTI2_IRQ);
      exti_set_trigger(EXTI2, EXTI_TRIGGER_RISING);
      exti_enable_request(EXTI2);
   }
   else
   {
      nvic_disable_irq(NVIC_EXTI2_IRQ);
   }

   timer_slave_set_mode(REV_CNT_TIMER, TIM_SMCR_SMS_EM3); // encoder mode
	timer_ic_set_input(REV_CNT_TIMER, TIM_IC1, TIM_IC_IN_TI1);
	timer_ic_set_input(REV_CNT_TIMER, TIM_IC2, TIM_IC_IN_TI2);
	timer_ic_set_filter(REV_CNT_TIMER, TIM_IC1, TIM_IC_DTF_DIV_32_N_8);
	timer_ic_set_filter(REV_CNT_TIMER, TIM_IC2, TIM_IC_DTF_DIV_32_N_8);
   timer_ic_enable(REV_CNT_TIMER, TIM_IC1);
   timer_ic_enable(REV_CNT_TIMER, TIM_IC2);
	timer_enable_counter(REV_CNT_TIMER);
}

extern "C" void exti2_isr(void)
{
   timer_set_counter(REV_CNT_TIMER, 0);
	exti_reset_request(EXTI2);
}

static int GetPulseTimeFiltered()
{
   static int lastN = 0;
   static int noMovement = 0;
   uint16_t n = REV_CNT_DMA_CNDTR;
   uint16_t measTm = REV_CNT_CCR;
   int pulses = n <= lastN ? lastN - n : lastN + MAX_REVCNT_VALUES - n;
   int max = 0;
   int min = 0xFFFF;
   lastN = n;


   GetMinMaxTime(min, max);

   if (pulses > 0)
   {
      noMovement = 0;
      ignore = false;
   }
   else
   {
      noMovement++;
   }

   //If we haven't seen movement for 1000 cycles, we assume the motor is stopped
   //The time that 1000 cycles corresponds to is dependent from the calling
   //frequency, i.e. the PWM frequency. The results proved ok for 8.8kHz
   if (noMovement > 1000)
   {
      ignore = true;
   }

   //spike detection, a factor of 4 between adjacent pulses is most likely caused by interference
   if (max > (4 * min) && min > 0)
   {
      ignore = true;
      pulses = 0;

      ErrorMessage::Post(ERR_ENCODER);
   }
   //a factor of 2 is still not stable, use the maximum
   else if (max > (2 * min))
   {
      lastPulseTimespan = max;
   }
   else
   {
      lastPulseTimespan = IIRFILTER(lastPulseTimespan, measTm, filter);
   }

   return pulses;
}

static void GetMinMaxTime(int& min, int& max)
{
   for (int i = 0; i < MAX_REVCNT_VALUES; i++)
   {
      min = MIN(min, timdata[i]);
      max = MAX(max, timdata[i]);
   }
}
