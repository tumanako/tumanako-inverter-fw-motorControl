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

/*
 * History:
 * Example from libopencm3 expanded to make tumanako arm tester
 * Added sine wave generation, removed testing code
 */
#define STM32F1

#include <stdint.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include "stm32_sine.h"
#include "stm32_timsched.h"
#include "terminal.h"
#include "params.h"
#include "hwdefs.h"
#include "digio.h"
#include "sine_core.h"
#include "fu.h"
#include "hwinit.h"
#include "anain.h"
#include "temp_meas.h"
#include "param_save.h"
#include "inc_encoder.h"
#include "foc.h"
#include "throttle.h"
#include "my_math.h"

#define MOD_OFF    0
#define MOD_RUN    1
#define MOD_MANUAL 2
#define MOD_BOOST  3

#define PWM_FUNC_TMPM  0
#define PWM_FUNC_TMPHS 1
#define PWM_FUNC_SPEED 2

#define RMS_SAMPLES 256
#define SQRT2OV1 0.707106781187

static uint8_t  pwmdigits;
static uint16_t pwmfrq;
static uint16_t angle;
//Current sensor offsets are determined at runtime
static s32fp ilofs[2] = { 0, 0 };

extern "C" void tim1_brk_isr(void)
{
   timer_disable_irq(PWM_TIMER, TIM_DIER_BIE);
   parm_SetDig(VALUE_opmode, MOD_OFF);
   DigIo::Set(Pin::err_out);
}

static void CalcNextAngle()
{
   static uint16_t slipAngle = 0;

   uint16_t polePairs = parm_GetInt(PARAM_polepairs);
   uint32_t ampnom = parm_GetInt(PARAM_ampnom);
   s32fp fslip = parm_Get(PARAM_fslipspnt);
   s32fp fmax = parm_Get(PARAM_fmax);

   Encoder::Update();
   uint16_t motorAngle = Encoder::GetAngle();
   s32fp frq = polePairs * Encoder::GetFrq() + fslip;

   if (frq > fmax)
      fslip = 0;

   slipAngle += FP_TOINT((fslip << SineCore::BITS) / pwmfrq);

   if (frq < 0) frq = 0;
   angle = polePairs * motorAngle + slipAngle;

   uint32_t amp = MotorVoltage::GetAmpPerc(frq, ampnom);
   SineCore::SetAmp(amp);
   parm_SetDig(VALUE_amp, amp);
   parm_SetFlt(VALUE_fstat, frq);
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

extern "C" void pwm_timer_isr(void)
{
   uint16_t last = timer_get_counter(PWM_TIMER);
   int opmode = parm_GetInt(VALUE_opmode);
   /* Clear interrupt pending flag */
   TIM_SR(PWM_TIMER) &= ~TIM_SR_UIF;

   if (opmode == MOD_MANUAL || opmode == MOD_RUN)
   {
      s32fp dir = parm_GetInt(VALUE_dir);
      uint8_t shiftCor = SineCore::BITS - pwmdigits;

      CalcNextAngle();
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

static void PwmInit(void)
{
   pwmdigits = MIN_PWM_DIGITS + parm_GetInt(PARAM_pwmfrq);
   pwmfrq = tim_setup(pwmdigits, parm_GetInt(PARAM_deadtime), parm_GetInt(PARAM_pwmpol));
}

static void Ms100Task(void)
{
   int32_t dir = parm_GetInt(VALUE_dir);

   DigIo::Toggle(Pin::led_out);

   parm_SetDig(VALUE_speed, Encoder::GetSpeed());

   /* Only change direction when below certain rev */
   if (Encoder::GetSpeed() < 100)
   {
      if (DigIo::Get(Pin::fwd_in) && !DigIo::Get(Pin::rev_in))
      {
         dir = 1;
      }
      else if (!DigIo::Get(Pin::fwd_in) && DigIo::Get(Pin::rev_in))
      {
         dir = -1;
      }
   }

   if (!DigIo::Get(Pin::fwd_in) && !DigIo::Get(Pin::rev_in))
   {
      dir = 0;
   }

   FOC::SetDirection(dir);

   //If break pin is high and both mprot and emcystop are high than it must be over current
   if (DigIo::Get(Pin::emcystop_in) && DigIo::Get(Pin::mprot_in) && DigIo::Get(Pin::bk_in))
   {
      parm_SetDig(VALUE_din_ocur, 1);
   }
   else
   {
      parm_SetDig(VALUE_din_ocur, 0);
   }

   if (DigIo::Get(Pin::cruise_in) && !DigIo::Get(Pin::brake_in))
   {
      Throttle::cruiseSpeed = Encoder::GetSpeed();
   }

   if (DigIo::Get(Pin::brake_in))
   {
      Throttle::cruiseSpeed = -1;
   }

   parm_SetDig(VALUE_dir, dir);
   parm_SetDig(VALUE_din_on, DigIo::Get(Pin::cruise_in));
   parm_SetDig(VALUE_din_start, DigIo::Get(Pin::start_in));
   parm_SetDig(VALUE_din_brake, DigIo::Get(Pin::brake_in));
   parm_SetDig(VALUE_din_mprot, DigIo::Get(Pin::mprot_in));
   parm_SetDig(VALUE_din_forward, DigIo::Get(Pin::fwd_in));
   parm_SetDig(VALUE_din_reverse, DigIo::Get(Pin::rev_in));
   parm_SetDig(VALUE_din_emcystop, DigIo::Get(Pin::emcystop_in));
   parm_SetDig(VALUE_din_bms, DigIo::Get(Pin::bms_in));
}

static void CalcAmpAndSlip(void)
{
   s32fp fslipmin = parm_Get(PARAM_fslipmin);
   s32fp fslipmax = parm_Get(PARAM_fslipmax);
   s32fp ampmin = parm_Get(PARAM_ampmin);
   s32fp potnom = parm_Get(VALUE_potnom);
   s32fp ampnom;
   s32fp fslipspnt;
   u32fp brkrampstr = (u32fp)parm_Get(PARAM_brkrampstr);

   if (potnom >= 0)
   {
      ampnom = ampmin + 2 * potnom;

      if (potnom >= FP_FROMINT(50))
      {
         fslipspnt = fslipmin + (FP_MUL(fslipmax-fslipmin, 2 * (potnom - FP_FROMINT(50))) / 100);
      }
      else
      {
         fslipspnt = fslipmin;
      }
   }
   else
   {
      ampnom = -potnom;
      fslipspnt = -fslipmin;
      if (Encoder::GetFrq() < brkrampstr)
      {
         ampnom = FP_TOINT(FP_DIV(Encoder::GetFrq(), brkrampstr) * ampnom);
      }
   }
   if (ampnom > FP_FROMINT(100))
   {
      ampnom = FP_FROMINT(100);
   }

   parm_SetFlt(PARAM_ampnom, IIRFILTER(parm_Get(PARAM_ampnom), ampnom, 3));
   parm_SetFlt(PARAM_fslipspnt, IIRFILTER(parm_Get(PARAM_fslipspnt), fslipspnt, 3));
}

static void CalcAndOutputTemp()
{
   static int temphs = 0;
   static int tempm = 0;
   int pwmgain = parm_GetInt(PARAM_pwmgain);
   int pwmofs = parm_GetInt(PARAM_pwmofs);
   int pwmfunc = parm_GetInt(PARAM_pwmfunc);
   int tmpout;
   TempMeas::Sensors snshs = (TempMeas::Sensors)parm_GetInt(PARAM_snshs);
   TempMeas::Sensors snsm = (TempMeas::Sensors)parm_GetInt(PARAM_snsm);

   temphs = IIRFILTER(AnaIn::Get(Pin::tmphs), temphs, 15);
   tempm = IIRFILTER(AnaIn::Get(Pin::tmpm), tempm, 18);

   s32fp tmpmf = TempMeas::Lookup(tempm, snsm);
   s32fp tmphsf = TempMeas::Lookup(temphs, snshs);

   switch (pwmfunc)
   {
      default:
      case PWM_FUNC_TMPM:
         tmpout = FP_TOINT(tmpmf) * pwmgain + pwmofs;
         break;
      case PWM_FUNC_TMPHS:
         tmpout = FP_TOINT(tmphsf) * pwmgain + pwmofs;
         break;
      case PWM_FUNC_SPEED:
         tmpout = FP_TOINT(tmpmf) * pwmgain + pwmofs;
         break;
   }

   tmpout = MIN(0xFFFF, MAX(0, tmpout));

   timer_set_oc_value(OVER_CUR_TIMER, TIM_OC4, tmpout);

   parm_SetFlt(VALUE_tmphs, tmphsf);
   parm_SetFlt(VALUE_tmpm, tmpmf);
}

static s32fp ProcessUdc()
{
   static int32_t udc = 0;
   s32fp udcfp;
   s32fp udcmin = parm_Get(PARAM_udcmin);
   s32fp udcmax = parm_Get(PARAM_udcmax);
   s32fp udclim = parm_Get(PARAM_udclim);
   s32fp udcgain = parm_Get(PARAM_udcgain);

   udc = IIRFILTER(udc, AnaIn::Get(Pin::udc), 5);
   udcfp = FP_DIV(FP_FROMINT(udc), udcgain);

   if (udcfp < udcmin || udcfp > udcmax)
      DigIo::Set(Pin::vtg_out);
   else
      DigIo::Clear(Pin::vtg_out);

   if (udcfp > udclim)
   {
      parm_SetDig(VALUE_opmode, MOD_OFF);
      DigIo::Set(Pin::err_out);
   }

   parm_SetFlt(VALUE_udc, udcfp);

   return udcfp;
}

static void CalcFancyValues()
{
   const s32fp twoPi = FP_FROMFLT(2*3.141593);
   s32fp amp = parm_Get(VALUE_amp);
   s32fp speed = parm_Get(VALUE_speed);
   s32fp il1 = parm_Get(VALUE_il1);
   s32fp il2 = parm_Get(VALUE_il2);
   s32fp udc = parm_Get(VALUE_udc);
   s32fp fac = FP_DIV(amp, FP_FROMINT(SineCore::MAXAMP));
   s32fp uac = FP_MUL(fac, FP_MUL(udc, FP_FROMFLT(0.7071)));
   s32fp idc, is, p, q, s, pf, t;

   FOC::ParkClarke(il1, il2, angle);
   is = fp_sqrt(FP_MUL(FOC::id,FOC::id)+FP_MUL(FOC::iq,FOC::iq));
   s = FP_MUL(fac, FP_MUL(uac, is) / 1000);
   p = FP_MUL(fac, FP_MUL(uac, FOC::id));
   t = FP_DIV(p, FP_MUL(twoPi, speed / 60));
   p/= 1000;
   q = FP_MUL(fac, FP_MUL(uac, FOC::iq) / 1000);
   pf = MIN(FP_FROMINT(1), MAX(0, FP_DIV(FOC::id, is)));
   idc = FP_MUL(FP_MUL(fac, FOC::id), FP_FROMFLT(0.7071));

   parm_SetFlt(VALUE_id, FOC::id);
   parm_SetFlt(VALUE_iq, FOC::iq);
   parm_SetFlt(VALUE_idc, idc);
   parm_SetFlt(VALUE_uac, uac);
   parm_SetFlt(VALUE_pf, pf);
   parm_SetFlt(VALUE_p, p);
   parm_SetFlt(VALUE_q, q);
   parm_SetFlt(VALUE_s, s);
   parm_SetFlt(VALUE_t, t);
}

static void CalcThrottle()
{
   int potval = AnaIn::Get(Pin::throttle1);
   int throtSpnt, idleSpnt, cruiseSpnt, derateSpnt, finalSpnt;

   parm_SetDig(VALUE_pot, potval);

   /* Error light on implausible value */
   if (!Throttle::CheckAndLimitRange(&potval))
   {
      DigIo::Set(Pin::err_out);
   }

   throtSpnt = Throttle::CalcThrottle(potval, parm_GetInt(VALUE_din_brake));
   idleSpnt = Throttle::CalcIdleSpeed(Encoder::GetSpeed());
   derateSpnt = Throttle::TemperatureDerate(parm_Get(VALUE_tmphs));

   finalSpnt = MAX(throtSpnt, idleSpnt);

   if (Throttle::cruiseSpeed > 0 && Throttle::cruiseSpeed > Throttle::idleSpeed)
   {
      cruiseSpnt = Throttle::CalcCruiseSpeed(Encoder::GetSpeed());
      if (throtSpnt < 0)
         finalSpnt = cruiseSpnt;
      else if (throtSpnt > 0)
         finalSpnt = MAX(cruiseSpnt, throtSpnt);
   }

   if (parm_GetInt(VALUE_din_bms))
   {
      if (finalSpnt >= 0)
         finalSpnt = MIN(finalSpnt, parm_GetInt(PARAM_bmslimhigh));
      else
         finalSpnt = MAX(finalSpnt, parm_GetInt(PARAM_bmslimlow));
   }

   if (derateSpnt < 100)
   {
      if (finalSpnt >= 0)
         finalSpnt = MIN(finalSpnt, derateSpnt);
      else
         finalSpnt = MAX(finalSpnt, -derateSpnt);
      DigIo::Set(Pin::err_out);
   }

   parm_SetDig(VALUE_potnom, finalSpnt);
}

static void Ms10Task(void)
{
   static int initWait = 0;
   int opmode = parm_GetInt(VALUE_opmode);
   s32fp udc = ProcessUdc();

   CalcThrottle();
   CalcAndOutputTemp();

   if (MOD_MANUAL != opmode && MOD_BOOST != opmode)
   {
      CalcAmpAndSlip();
   }

   /* switch on DC switch above threshold but only if
    * - throttle is not pressed
    * - start pin is high
    * - motor protection switch and emcystop is high (=inactive)
    */
   if (udc >= parm_Get(PARAM_udcsw) && parm_GetInt(VALUE_potnom) <= 0)
   {
      if (DigIo::Get(Pin::start_in) &&
          DigIo::Get(Pin::emcystop_in) &&
          DigIo::Get(Pin::mprot_in))
      {
         DigIo::Set(Pin::dcsw_out);
         DigIo::Clear(Pin::err_out);
         DigIo::Clear(Pin::prec_out);
         parm_SetDig(VALUE_opmode, MOD_RUN);
      }
   }

   if (MOD_OFF == opmode)
   {
      initWait = 50;

      parm_SetDig(VALUE_amp, 0);
      tim_output_disable();
      DigIo::Clear(Pin::dcsw_out);
      Throttle::cruiseSpeed = -1;
   }
   else if (0 == initWait)
   {
      PwmInit();
      DigIo::Clear(Pin::err_out);
      initWait = -1;
   }
   else if (initWait > 0)
   {
      initWait--;
   }
}

static void SetCurrentLimitThreshold()
{
   s32fp ocurlim = parm_Get(PARAM_ocurlim);
   //We use the average offset and gain values because we only
   //have one reference channel per polarity
   s32fp iofs = (ilofs[0] + ilofs[1]) / 2;
   s32fp igain= (parm_Get(PARAM_il1gain) + parm_Get(PARAM_il2gain)) / 2;

   ocurlim = FP_MUL(igain, ocurlim);
   int limNeg = FP_TOINT(iofs-ocurlim);
   int limPos = FP_TOINT(iofs+ocurlim);
   limNeg = MAX(0, limNeg);
   limPos = MIN(OCURMAX, limPos);

   timer_set_oc_value(OVER_CUR_TIMER, TIM_OC2, limNeg);
   timer_set_oc_value(OVER_CUR_TIMER, TIM_OC3, limPos);
}

static void Ms1Task(void)
{
   static s32fp ilrms[2] = { 0, 0 };
   static int samples = 0;
   static int speedCnt = 0;

   for (int i = 0; i < 2; i++)
   {
      s32fp il;
      il = FP_FROMINT(AnaIn::Get((Pin::AnaIns)(Pin::il1+i)));

      if (MOD_OFF == parm_GetInt(VALUE_opmode))
      {
         ilofs[i] = il;
         SetCurrentLimitThreshold();
      }

      il -= ilofs[i];

      il = FP_DIV(il, parm_Get((PARAM_NUM)(PARAM_il1gain+i)));

      parm_SetFlt((PARAM_NUM)(VALUE_il1+i), il);

      il = il<0?-il:il;

      ilrms[i] = MAX(il, ilrms[i]);

      if (samples == 0)
      {
         ilrms[i] = FP_MUL(ilrms[i], FP_FROMFLT(SQRT2OV1));
         parm_SetFlt((PARAM_NUM)(VALUE_il1rms+i), ilrms[i]);
         ilrms[i] = 0;
      }
   }

   if (samples == 0)
   {
      CalcFancyValues();
   }
   samples = (samples + 1) & (RMS_SAMPLES-1);

   if (speedCnt == 0)
   {
      DigIo::Toggle(Pin::speed_out);
      speedCnt = parm_GetInt(PARAM_speedgain) / (2 * parm_GetInt(VALUE_speed));
   }
   else
   {
      speedCnt--;
   }
}

/** This function is called when the user changes a parameter */
extern void parm_Change(PARAM_NUM ParamNum)
{
   (void)ParamNum;
   SetCurrentLimitThreshold();

   Encoder::SetFilterConst(parm_GetInt(PARAM_encflt));
   Encoder::SetImpulsesPerTurn(parm_GetInt(PARAM_numimp));

   MotorVoltage::SetWeakeningFrq(parm_Get(PARAM_fweak));
   MotorVoltage::SetBoost(parm_GetInt(PARAM_boost));
   MotorVoltage::SetMinFrq(parm_Get(PARAM_fmin));
   SineCore::SetMinPulseWidth(parm_GetInt(PARAM_minpulse));

   Throttle::potmin = parm_GetInt(PARAM_potmin);
   Throttle::potmax = parm_GetInt(PARAM_potmax);
   Throttle::brknom = parm_GetInt(PARAM_brknom);
   Throttle::brknompedal = parm_GetInt(PARAM_brknompedal);
   Throttle::brkmax = parm_GetInt(PARAM_brkmax);
   Throttle::idleSpeed = parm_GetInt(PARAM_idlespeed);
   Throttle::speedkp = parm_Get(PARAM_speedkp);
   Throttle::speedflt = parm_GetInt(PARAM_speedflt);
}

extern "C" int main(void)
{
   clock_setup();
   DigIo::Init();
   usart_setup();
   nvic_setup();
   PwmInit();
   init_timsched();
   AnaIn::Init();
   Encoder::Init();
   parm_load();
   parm_Change(PARAM_LAST);
   MotorVoltage::SetMaxAmp(SineCore::MAXAMP);

   create_task(Ms100Task, PRIO_GRP1, 0, 100);
   create_task(Ms10Task,  PRIO_GRP1, 1, 10);
   create_task(Ms1Task,   PRIO_GRP1, 2, 1);

   DigIo::Set(Pin::prec_out);

   term_Run(TERM_USART);

   return 0;
}

