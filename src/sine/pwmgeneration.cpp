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

#define SHIFT_180DEG 32768
#define FRQ_TO_ANGLE(frq) FP_TOINT((frq << SineCore::BITS) / pwmfrq)

static uint8_t  pwmdigits;
static uint16_t pwmfrq;
static volatile uint16_t angle;
//static uint16_t angle2;
static int ampnom;
static uint16_t slipIncr;
static s32fp fslip;
static s32fp frq;

/*********/
static void CalcNextAngleSync();
static void CalcNextAngle();
static void Boost();


uint16_t PwmGeneration::GetAngle()
{
   return angle;
}

void PwmGeneration::SetAmpnom(int amp)
{
   ampnom = amp;
}

void PwmGeneration::SetFslip(s32fp _fslip)
{
   slipIncr = FRQ_TO_ANGLE(_fslip);
   fslip = _fslip;
}

extern "C" void tim1_brk_isr(void)
{
   timer_disable_irq(PWM_TIMER, TIM_DIER_BIE);
   parm_SetDig(VALUE_opmode, MOD_OFF);
}

extern "C" void pwm_timer_isr(void)
{
   uint16_t last = timer_get_counter(PWM_TIMER);
   int opmode = parm_GetInt(VALUE_opmode);
   /* Clear interrupt pending flag */
   TIM_SR(PWM_TIMER) &= ~TIM_SR_UIF;

   if (opmode == MOD_MANUAL || opmode == MOD_RUN)
   {
      s32fp dir = parm_Get(VALUE_dir);
      uint8_t shiftCor = SineCore::BITS - pwmdigits;

      Encoder::Update();

      if (parm_GetInt(PARAM_syncmode))
         CalcNextAngleSync();
      else
         CalcNextAngle();

      uint32_t amp = MotorVoltage::GetAmpPerc(frq, ampnom);
      SineCore::SetAmp(amp);
      parm_SetDig(VALUE_amp, amp);
      parm_SetFlt(VALUE_fstat, frq);
      //parm_SetDig(VALUE_angle, angle);
      SineCore::Calc(angle);

      /* Match to PWM resolution */
      SineCore::DutyCycles[0] >>= shiftCor;
      SineCore::DutyCycles[1] >>= shiftCor;
      SineCore::DutyCycles[2] >>= shiftCor;

      if (0 == parm_GetInt(VALUE_amp))
      {
          SineCore::DutyCycles[0] = SineCore::DutyCycles[1] = SineCore::DutyCycles[2] = 0;
      }

      timer_set_oc_value(PWM_TIMER, TIM_OC1, SineCore::DutyCycles[0]);

      if (dir == 0 )
      {
         timer_set_oc_value(PWM_TIMER, TIM_OC2, SineCore::DutyCycles[0]);
         timer_set_oc_value(PWM_TIMER, TIM_OC3, SineCore::DutyCycles[0]);
      }
      else if (dir > 0 )
      {
         timer_set_oc_value(PWM_TIMER, TIM_OC2, SineCore::DutyCycles[1]);
         timer_set_oc_value(PWM_TIMER, TIM_OC3, SineCore::DutyCycles[2]);
      }
      else if (dir < 0 )
      {
         timer_set_oc_value(PWM_TIMER, TIM_OC2, SineCore::DutyCycles[2]);
         timer_set_oc_value(PWM_TIMER, TIM_OC3, SineCore::DutyCycles[1]);
      }
   }
   else if (opmode == MOD_BOOST)
   {
      Boost();
   }
   parm_SetDig(VALUE_tm_meas, (timer_get_counter(PWM_TIMER) - last)/72);
}

void PwmGeneration::PwmInit(void)
{
   pwmdigits = MIN_PWM_DIGITS + parm_GetInt(PARAM_pwmfrq);
   pwmfrq = TimerSetup(parm_GetInt(PARAM_deadtime), parm_GetInt(PARAM_pwmpol));
   slipIncr = FRQ_TO_ANGLE(fslip);
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
static void CalcNextAngleSync()
{
   if (Encoder::SeenNorthSignal())
   {
      uint32_t polePairs = parm_GetInt(PARAM_polepairs);
      s32fp potNom = parm_Get(VALUE_potnom);
      uint16_t syncOfs = parm_GetInt(PARAM_syncofs);
      uint16_t motorAngle = Encoder::GetAngle();

      parm_SetDig(VALUE_angle, motorAngle);

      //For regen let the stator field lag the rotor
      //-> Phase shift by 180°
      /*if (potNom < 0)
      {
         syncOfs += SHIFT_180DEG;
      }*/

      angle = polePairs * motorAngle + syncOfs;
      frq = polePairs * Encoder::GetFrq();
   }
   else
   {
      frq = fslip;
      angle += FRQ_TO_ANGLE(fslip);
   }
}

static void CalcNextAngle()
{
   static uint16_t slipAngle = 0;
   uint32_t polePairs = parm_GetInt(PARAM_polepairs);
   uint16_t motorAngle = Encoder::GetAngle();

   frq = polePairs * Encoder::GetFrq() + fslip;
   slipAngle += slipIncr;

   if (frq < 0) frq = 0;

   angle = polePairs * motorAngle + slipAngle;
}

static void Boost()
{
   uint32_t ampnom = parm_GetInt(PARAM_ampnom);
   ampnom *= 1 << pwmdigits;
   ampnom /= 100;
   timer_disable_oc_output(PWM_TIMER, TIM_OC1);
   timer_disable_oc_output(PWM_TIMER, TIM_OC1N);
   timer_disable_oc_output(PWM_TIMER, TIM_OC2);
   timer_disable_oc_output(PWM_TIMER, TIM_OC3);
   timer_disable_oc_output(PWM_TIMER, TIM_OC3N);
   timer_set_oc_value(PWM_TIMER, TIM_OC1, 0);
   timer_set_oc_value(PWM_TIMER, TIM_OC2, ampnom);
   timer_set_oc_value(PWM_TIMER, TIM_OC3, 0);
}

/**
* Setup main PWM timer and timer for generating over current
* reference values and external PWM
*
* @param[in] pwmdigits Number of binary digits to use (determines PWM frequency)
* @param[in] deadtime Deadtime between bottom and top (coded value, consult STM32 manual)
* @param[in] pwmpol Output Polarity. 0=Active High, 1=Active Low
* @return PWM frequency
*/
uint16_t PwmGeneration::TimerSetup(uint16_t deadtime, int pwmpol)
{
   const uint16_t pwmmax = 1U << pwmdigits;
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
   }
   else
   {
      timer_set_oc_polarity_high(PWM_TIMER, TIM_OC1);
      timer_set_oc_polarity_high(PWM_TIMER, TIM_OC2);
      timer_set_oc_polarity_high(PWM_TIMER, TIM_OC3);
   }

   EnableOutput();

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

   return PERIPH_CLK / (uint32_t)pwmmax;
}
