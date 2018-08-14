/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2015 Johannes Huebner <dev@johanneshuebner.com>
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
#include <libopencm3/stm32/timer.h>
#include "pwmgeneration.h"
#include "hwdefs.h"
#include "params.h"
#include "inc_encoder.h"
#include "sine_core.h"
#include "fu.h"
#include "errormessage.h"
#include "digio.h"
#include "anain.h"
#include "my_math.h"

#define SHIFT_180DEG (uint16_t)32768
#define SHIFT_90DEG  (uint16_t)16384
#define FRQ_TO_ANGLE(frq) FP_TOINT((frq << SineCore::BITS) / pwmfrq)

static uint8_t  pwmdigits;
static uint16_t pwmfrq;
static volatile uint16_t angle;
//static uint16_t angle2;
static s32fp ampnom;
static uint16_t slipIncr;
static s32fp fslip;
static s32fp frq;
static uint8_t shiftForTimer;
static int opmode;

/*********/
static void CalcNextAngleSync(int dir);
static void CalcNextAngle(int dir);
static void CalcNextAngleConstant(int dir);
static void Charge();
static void AcHeat();

uint16_t PwmGeneration::GetAngle()
{
   return angle;
}

void PwmGeneration::SetAmpnom(s32fp amp)
{
   ampnom = amp;
}

void PwmGeneration::SetFslip(s32fp _fslip)
{
   slipIncr = FRQ_TO_ANGLE(_fslip);
   fslip = _fslip;
}

void PwmGeneration::SetOpmode(int _opmode)
{
   opmode = _opmode;

   switch (opmode)
   {
      default:
      case MOD_OFF:
         DisableOutput();
         break;
      case MOD_ACHEAT:
         EnableOutput();
         timer_disable_oc_output(PWM_TIMER, TIM_OC3);
         timer_disable_oc_output(PWM_TIMER, TIM_OC3N);
         break;
      case MOD_BOOST:
         DisableOutput();
         timer_enable_oc_output(PWM_TIMER, TIM_OC2N);
         break;
      case MOD_BUCK:
         DisableOutput();
         timer_enable_oc_output(PWM_TIMER, TIM_OC2);
         break;
      case MOD_MANUAL:
      case MOD_RUN:
      case MOD_SINE:
         EnableOutput();
         break;
   }
}

extern "C" void tim1_brk_isr(void)
{
   timer_disable_irq(PWM_TIMER, TIM_DIER_BIE);
   Param::SetDig(Param::opmode, MOD_OFF);
   DigIo::Set(Pin::err_out);

   if (!DigIo::Get(Pin::emcystop_in))
      ErrorMessage::Post(ERR_EMCYSTOP);
   else if (!DigIo::Get(Pin::mprot_in))
      ErrorMessage::Post(ERR_MPROT);
   else
      ErrorMessage::Post(ERR_OVERCURRENT);
}

extern "C" void pwm_timer_isr(void)
{
   int last = timer_get_counter(PWM_TIMER);
   /* Clear interrupt pending flag */
   TIM_SR(PWM_TIMER) &= ~TIM_SR_UIF;

   if (opmode == MOD_MANUAL || opmode == MOD_RUN || opmode == MOD_SINE)
   {
      int dir = Param::GetInt(Param::dir);
      uint8_t shiftForTimer = SineCore::BITS - pwmdigits;

      Encoder::UpdateRotorAngle(dir);

      if (opmode == MOD_SINE)
         CalcNextAngleConstant(dir);
      else if (Encoder::IsSyncMode())
         CalcNextAngleSync(dir);
      else
         CalcNextAngle(dir);

      uint32_t amp = MotorVoltage::GetAmpPerc(frq, FP_TOINT(ampnom));

      SineCore::SetAmp(amp);
      Param::SetDig(Param::amp, amp);
      Param::SetFlt(Param::fstat, frq);
      SineCore::Calc(angle);

      /* Match to PWM resolution */
      SineCore::DutyCycles[0] >>= shiftForTimer;
      SineCore::DutyCycles[1] >>= shiftForTimer;
      SineCore::DutyCycles[2] >>= shiftForTimer;

      /* Shut down PWM on zero voltage request */
      if (0 == amp || 0 == dir)
      {
         //SineCore::DutyCycles[0] = SineCore::DutyCycles[1] = SineCore::DutyCycles[2] = 0;
         timer_disable_break_main_output(PWM_TIMER);
      }
      else
      {
         timer_enable_break_main_output(PWM_TIMER);
      }

      timer_set_oc_value(PWM_TIMER, TIM_OC1, SineCore::DutyCycles[0]);
      timer_set_oc_value(PWM_TIMER, TIM_OC2, SineCore::DutyCycles[1]);
      timer_set_oc_value(PWM_TIMER, TIM_OC3, SineCore::DutyCycles[2]);
   }
   else if (opmode == MOD_BOOST || opmode == MOD_BUCK)
   {
      Charge();
   }
   else if (opmode == MOD_ACHEAT)
   {
      AcHeat();
   }
   int time = timer_get_counter(PWM_TIMER) - last;
   time = ABS(time) / 72;
   Param::SetDig(Param::tm_meas, time);
}

void PwmGeneration::PwmInit(void)
{
   pwmdigits = MIN_PWM_DIGITS + Param::GetInt(Param::pwmfrq);
   pwmfrq = TimerSetup(Param::GetInt(Param::deadtime), Param::GetInt(Param::pwmpol));
   slipIncr = FRQ_TO_ANGLE(fslip);
   shiftForTimer = SineCore::BITS - pwmdigits;
}

/**
* Enable timer PWM output
*/
void PwmGeneration::EnableOutput()
{
   timer_enable_oc_output(PWM_TIMER, TIM_OC1);
   timer_enable_oc_output(PWM_TIMER, TIM_OC2);
   timer_enable_oc_output(PWM_TIMER, TIM_OC3);
   timer_enable_oc_output(PWM_TIMER, TIM_OC1N);
   timer_enable_oc_output(PWM_TIMER, TIM_OC2N);
   timer_enable_oc_output(PWM_TIMER, TIM_OC3N);
}

/**
* Disable timer PWM output
*/
void PwmGeneration::DisableOutput()
{
   timer_disable_oc_output(PWM_TIMER, TIM_OC1);
   timer_disable_oc_output(PWM_TIMER, TIM_OC2);
   timer_disable_oc_output(PWM_TIMER, TIM_OC3);
   timer_disable_oc_output(PWM_TIMER, TIM_OC1N);
   timer_disable_oc_output(PWM_TIMER, TIM_OC2N);
   timer_disable_oc_output(PWM_TIMER, TIM_OC3N);
}

/*----- Private methods ----------------------------------------- */
static void CalcNextAngleSync(int dir)
{
   if (Encoder::SeenNorthSignal())
   {
      uint32_t polePairs = Param::GetInt(Param::encmode) == Encoder::RESOLVER ? 1 : Param::GetInt(Param::polepairs);
      uint16_t syncOfs = Param::GetInt(Param::syncofs);
      uint16_t rotorAngle = Encoder::GetRotorAngle(0);

      if (dir < 0)
      {
         syncOfs += SHIFT_180DEG;
      }

      angle = polePairs * rotorAngle + syncOfs;
      frq = polePairs * Encoder::GetRotorFrequency();
      Param::SetDig(Param::angle, angle);

   }
   else
   {
      frq = fslip;
      angle += dir * FRQ_TO_ANGLE(fslip);
   }
}

static void CalcNextAngle(int dir)
{
   static uint16_t slipAngle = 0;
   uint32_t polePairs = Param::GetInt(Param::polepairs);
   uint16_t rotorAngle = Encoder::GetRotorAngle(dir);

   frq = polePairs * Encoder::GetRotorFrequency() + fslip;
   slipAngle += dir * slipIncr;

   if (frq < 0) frq = 0;

   angle = polePairs * rotorAngle + slipAngle;
}

static void CalcNextAngleConstant(int dir)
{
   frq = fslip;
   angle += dir * slipIncr;

   if (frq < 0) frq = 0;
}

static void Charge()
{
   int dc = ampnom * (1 << pwmdigits);
   dc = FP_TOINT(dc) / 100;

   if (dc > ((1 << pwmdigits) - 100))
      dc = (1 << pwmdigits) - 100;
   if (dc < 0)
      dc = 0;

   Param::SetDig(Param::amp, dc);

   timer_set_oc_value(PWM_TIMER, TIM_OC2, dc);
}

static void AcHeat()
{
   frq = fslip;
   angle += slipIncr;

   if (frq < 0) frq = 0;

   Param::SetFlt(Param::fstat, frq);
   //Param::SetDig(Param::angle, angle);
   int val1 = SineCore::Sine(angle);
   int val2 = -SineCore::Sine(angle);

   val1 = FP_TOINT((val1 * ampnom) / 100) + 32768;
   val2 = FP_TOINT((val2 * ampnom) / 100) + 32768;
   val1 >>= shiftForTimer;
   val2 >>= shiftForTimer;

   timer_set_oc_value(PWM_TIMER, TIM_OC1, val1);
   timer_set_oc_value(PWM_TIMER, TIM_OC2, val2);
}

/**
* Setup main PWM timer
*
* @param[in] deadtime Deadtime between bottom and top (coded value, consult STM32 manual)
* @param[in] pwmpol Output Polarity. 0=Active High, 1=Active Low
* @return PWM frequency
*/
uint16_t PwmGeneration::TimerSetup(uint16_t deadtime, int pwmpol)
{
   const uint16_t pwmmax = 1U << pwmdigits;
   uint8_t outputMode;

   timer_reset(PWM_TIMER);
   /* disable timer */
   timer_disable_counter(PWM_TIMER);
   /* Center aligned PWM */
   timer_set_alignment(PWM_TIMER, TIM_CR1_CMS_CENTER_1);
   timer_enable_preload(PWM_TIMER);
   /* PWM mode 1 and preload enable */
   TIM_CCMR1(PWM_TIMER) = TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE |
                          TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE;
   TIM_CCMR2(PWM_TIMER) = TIM_CCMR2_OC3M_PWM1 | TIM_CCMR2_OC3PE;

   if (pwmpol)
   {
      timer_set_oc_polarity_low(PWM_TIMER, TIM_OC1);
      timer_set_oc_polarity_low(PWM_TIMER, TIM_OC2);
      timer_set_oc_polarity_low(PWM_TIMER, TIM_OC3);
      timer_set_oc_polarity_low(PWM_TIMER, TIM_OC1N);
      timer_set_oc_polarity_low(PWM_TIMER, TIM_OC2N);
      timer_set_oc_polarity_low(PWM_TIMER, TIM_OC3N);
      outputMode = GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN;
   }
   else
   {
      timer_set_oc_polarity_high(PWM_TIMER, TIM_OC1);
      timer_set_oc_polarity_high(PWM_TIMER, TIM_OC2);
      timer_set_oc_polarity_high(PWM_TIMER, TIM_OC3);
      timer_set_oc_polarity_high(PWM_TIMER, TIM_OC1N);
      timer_set_oc_polarity_high(PWM_TIMER, TIM_OC2N);
      timer_set_oc_polarity_high(PWM_TIMER, TIM_OC3N);
      outputMode = GPIO_CNF_OUTPUT_ALTFN_PUSHPULL;
   }

   timer_disable_break_automatic_output(PWM_TIMER);
   timer_enable_break_main_output(PWM_TIMER);
   timer_set_break_polarity_high(PWM_TIMER);
   timer_enable_break(PWM_TIMER);
   timer_set_enabled_off_state_in_run_mode(PWM_TIMER);
   timer_set_enabled_off_state_in_idle_mode(PWM_TIMER);

   timer_set_deadtime(PWM_TIMER, deadtime);

   timer_set_oc_idle_state_unset(PWM_TIMER, TIM_OC1);
   timer_set_oc_idle_state_unset(PWM_TIMER, TIM_OC2);
   timer_set_oc_idle_state_unset(PWM_TIMER, TIM_OC3);
   timer_set_oc_idle_state_unset(PWM_TIMER, TIM_OC1N);
   timer_set_oc_idle_state_unset(PWM_TIMER, TIM_OC2N);
   timer_set_oc_idle_state_unset(PWM_TIMER, TIM_OC3N);

   timer_generate_event(PWM_TIMER, TIM_EGR_UG);

   timer_clear_flag(PWM_TIMER, TIM_SR_BIF);
   timer_enable_irq(PWM_TIMER, TIM_DIER_UIE);
   timer_enable_irq(PWM_TIMER, TIM_DIER_BIE);

   timer_set_prescaler(PWM_TIMER, 0);
   /* PWM frequency */
   timer_set_period(PWM_TIMER, pwmmax);
   timer_set_repetition_counter(PWM_TIMER, 1);

   timer_set_oc_value(PWM_TIMER, TIM_OC1, 0);
   timer_set_oc_value(PWM_TIMER, TIM_OC2, 0);
   timer_set_oc_value(PWM_TIMER, TIM_OC3, 0);

   timer_enable_counter(PWM_TIMER);

   gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, outputMode, GPIO8 | GPIO9 | GPIO10);
   gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, outputMode, GPIO13 | GPIO14 | GPIO15);

   return PERIPH_CLK / (uint32_t)pwmmax;
}
