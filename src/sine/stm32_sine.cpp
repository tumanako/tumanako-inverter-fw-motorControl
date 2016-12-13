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
#include <stdint.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include "stm32_sine.h"
#include "stm32_timsched.h"
#include "stm32_can.h"
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
#include "errormessage.h"
#include "pwmgeneration.h"

#define RMS_SAMPLES 256
#define SQRT2OV1 0.707106781187

static volatile uint32_t timeTick = 0; //ms since system start
//Current sensor offsets are determined at runtime
static s32fp ilofs[2] = { 0, 0 };
//Precise control of executing the boost controller
static bool runBoostControl = false;

static void CruiseControl()
{
   static bool lastState = false;

   //Always disable cruise control when brake pedal is pressed
   if (DigIo::Get(Pin::brake_in))
    {
      Throttle::cruiseSpeed = -1;
   }
   else
   {
      if (parm_GetInt(PARAM_cruisemode) == BUTTON)
      {
         //Enable/update cruise control when button is pressed
         if (DigIo::Get(Pin::cruise_in))
         {
            Throttle::cruiseSpeed = Encoder::GetSpeed();
         }
      }
      else //if cruiseMode == SWITCH
      {
         //Enable/update cruise control when switch is toggled on
         if (DigIo::Get(Pin::cruise_in) && !lastState)
         {
            Throttle::cruiseSpeed = Encoder::GetSpeed();
         }

         //Disable cruise control when switch is off
         if (!DigIo::Get(Pin::cruise_in))
         {
            Throttle::cruiseSpeed = -1;
         }
      }
   }

   lastState = DigIo::Get(Pin::cruise_in);
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

   if (!(DigIo::Get(Pin::fwd_in) ^ DigIo::Get(Pin::rev_in)))
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

   CruiseControl();

   parm_SetDig(VALUE_dir, dir);
   parm_SetDig(VALUE_din_cruise, DigIo::Get(Pin::cruise_in));
   parm_SetDig(VALUE_din_start, DigIo::Get(Pin::start_in));
   parm_SetDig(VALUE_din_brake, DigIo::Get(Pin::brake_in));
   parm_SetDig(VALUE_din_mprot, DigIo::Get(Pin::mprot_in));
   parm_SetDig(VALUE_din_forward, DigIo::Get(Pin::fwd_in));
   parm_SetDig(VALUE_din_reverse, DigIo::Get(Pin::rev_in));
   parm_SetDig(VALUE_din_emcystop, DigIo::Get(Pin::emcystop_in));
   parm_SetDig(VALUE_din_bms, DigIo::Get(Pin::bms_in));

   s32fp data[2] = { parm_Get(VALUE_speed), parm_Get(VALUE_udc) };

   can_send(0x180, (uint8_t*)data, sizeof(data));
}

static void CalcAmpAndSlip(void)
{
   s32fp fslipmin = parm_Get(PARAM_fslipmin);
   s32fp ampmin = parm_Get(PARAM_ampmin);
   s32fp potnom = parm_Get(VALUE_potnom);
   s32fp ampnom;
   s32fp fslipspnt;

   if (potnom >= 0)
   {
      ampnom = ampmin + 2 * potnom;

      if (potnom >= FP_FROMINT(50))
      {
         s32fp fslipmax = parm_Get(PARAM_fslipmax);
         s32fp fpconst =  parm_Get(PARAM_fpconst);
         s32fp fstat = parm_Get(VALUE_fstat);
         s32fp fweak = parm_Get(PARAM_fweak);
         s32fp fslipdiff = fslipmax - fslipmin;
         //Derate the slip frequency above fpconst and uprate above fweak
         s32fp ffac = 2*(fstat > fweak?fpconst + fstat - 2 * fweak:fpconst - fstat);
         s32fp fslipdrt = FP_MUL(FP_DIV(ffac, fweak), fslipdiff) + fslipmax;

         fslipspnt = fslipmin + (FP_MUL(fslipdiff, 2 * (potnom - FP_FROMINT(50))) / 100);
         fslipspnt = MIN(fslipspnt, fslipdrt);
      }
      else
      {
         fslipspnt = fslipmin;
      }
      DigIo::Clear(Pin::brk_out);
   }
   else
   {
      u32fp brkrampstr = (u32fp)parm_Get(PARAM_brkrampstr);

      ampnom = -potnom;
      fslipspnt = -fslipmin;
      if (Encoder::GetFrq() < brkrampstr)
      {
         ampnom = FP_TOINT(FP_DIV(Encoder::GetFrq(), brkrampstr) * ampnom);
      }
      //This works because ampnom = -potnom
      if (ampnom >= -parm_Get(PARAM_brkout))
         DigIo::Set(Pin::brk_out);
      else
         DigIo::Clear(Pin::brk_out);
   }

   ampnom = MIN(ampnom, FP_FROMINT(100));

   parm_SetFlt(PARAM_ampnom, IIRFILTER(parm_Get(PARAM_ampnom), ampnom, 3));
   parm_SetFlt(PARAM_fslipspnt, IIRFILTER(parm_Get(PARAM_fslipspnt), fslipspnt, 3));
   PwmGeneration::SetAmpnom(parm_Get(PARAM_ampnom));
   PwmGeneration::SetFslip(parm_Get(PARAM_fslipspnt));
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
         tmpout = FP_TOINT(tmpmf) * parm_Get(VALUE_speed) + pwmofs;
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

   FOC::ParkClarke(il1, il2, PwmGeneration::GetAngle());
   is = fp_sqrt(FP_MUL(FOC::id,FOC::id)+FP_MUL(FOC::iq,FOC::iq));
   s = FP_MUL(fac, FP_MUL(uac, is) / 1000);
   p = FP_MUL(fac, FP_MUL(uac, FOC::id));
   t = FP_DIV(p, FP_MUL(twoPi, speed / 60));
   p/= 1000;
   q = FP_MUL(fac, FP_MUL(uac, FOC::iq) / 1000);
   pf = MIN(FP_FROMINT(1), MAX(0, FP_DIV(FOC::id, is)));

   if (parm_GetInt(VALUE_opmode) != MOD_BOOST)
   {
      idc = FP_MUL(FP_MUL(fac, FOC::id), FP_FROMFLT(0.7071));
      parm_SetFlt(VALUE_idc, idc);
   }

   parm_SetFlt(VALUE_id, FOC::id);
   parm_SetFlt(VALUE_iq, FOC::iq);
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
   int pot2val = AnaIn::Get(Pin::throttle2);
   int throtSpnt, idleSpnt, cruiseSpnt, derateSpnt, finalSpnt;


   parm_SetDig(VALUE_pot, potval);
   parm_SetDig(VALUE_pot2, pot2val);

   /* Error light on implausible value */
   if (!Throttle::CheckAndLimitRange(&potval, 0))
   {
      DigIo::Set(Pin::err_out);
      ErrorMessage::Post(ERR_THROTTLE1);
   }

   Throttle::CheckAndLimitRange(&pot2val, 1);

   throtSpnt = Throttle::CalcThrottle(potval, pot2val, DigIo::Get(Pin::brake_in));
   idleSpnt = Throttle::CalcIdleSpeed(Encoder::GetSpeed());
   derateSpnt = Throttle::TemperatureDerate(parm_Get(VALUE_tmphs));
   cruiseSpnt = Throttle::CalcCruiseSpeed(Encoder::GetSpeed());

   if (parm_GetInt(PARAM_idlemode) == IDLE_MODE_ALWAYS || !DigIo::Get(Pin::brake_in))
      finalSpnt = MAX(throtSpnt, idleSpnt);
   else
      finalSpnt = throtSpnt;

   if (Throttle::cruiseSpeed > 0 && Throttle::cruiseSpeed > Throttle::idleSpeed)
   {
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

   if (MOD_RUN == opmode)
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
      if (DigIo::Get(Pin::emcystop_in) &&
          DigIo::Get(Pin::mprot_in))
      {
         /* Switch to charge mode if
          * - Charge mode is enabled
          * - Fwd AND Rev are high
          */
         if (DigIo::Get(Pin::fwd_in) && DigIo::Get(Pin::rev_in) && parm_Get(PARAM_chargena))
         {
            DigIo::Set(Pin::dcsw_out);
            DigIo::Clear(Pin::err_out);
            DigIo::Clear(Pin::prec_out);
            parm_SetDig(VALUE_opmode, MOD_BOOST);
            ErrorMessage::UnpostAll();
         }
         else if (DigIo::Get(Pin::start_in))
         {
            DigIo::Set(Pin::dcsw_out);
            DigIo::Clear(Pin::err_out);
            DigIo::Clear(Pin::prec_out);
            parm_SetDig(VALUE_opmode, MOD_RUN);
            ErrorMessage::UnpostAll();
            runBoostControl = false;
         }
      }
   }

   if (MOD_OFF == opmode)
   {
      initWait = 50;

      parm_SetDig(VALUE_amp, 0);
      PwmGeneration::SetOpmode(MOD_OFF);
      DigIo::Clear(Pin::dcsw_out);
      Throttle::cruiseSpeed = -1;
      runBoostControl = false;
   }
   else if (0 == initWait)
   {
      PwmGeneration::PwmInit(); //this applies new deadtime and pwmfrq
      PwmGeneration::SetOpmode(opmode);
      runBoostControl = opmode == MOD_BOOST;
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
   static s32fp ilFlt = 0;
   static s32fp iSpntFlt = 0;
   s32fp ilMax = 0;

   timeTick++;
   ErrorMessage::SetTime(timeTick);

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

      il = ABS(il);

      ilrms[i] = MAX(il, ilrms[i]);

      if (samples == 0)
      {
         ilrms[i] = FP_MUL(ilrms[i], FP_FROMFLT(SQRT2OV1));
         parm_SetFlt((PARAM_NUM)(VALUE_il1rms+i), ilrms[i]);
         ilrms[i] = 0;
      }
      il <<= 8;
      ilMax = MAX(ilMax, il);
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

   if (runBoostControl)
   {
      ilFlt = IIRFILTER(ilFlt, ilMax, parm_GetInt(PARAM_chargeflt));
      iSpntFlt = IIRFILTER(iSpntFlt, parm_Get(PARAM_chargecur) << 8, 11);

      s32fp ampnom = FP_MUL(parm_GetInt(PARAM_chargekp), (iSpntFlt - ilFlt));
      parm_SetFlt(PARAM_ampnom, ampnom);
      parm_SetFlt(VALUE_idc, (FP_MUL((FP_FROMINT(100) - ampnom), ilFlt) / 100) >> 8);
      PwmGeneration::SetAmpnom(ampnom);
   }
   else
   {
      iSpntFlt = 0;
   }
}

/** This function is called when the user changes a parameter */
extern void parm_Change(PARAM_NUM paramNum)
{
   if (PARAM_fslipspnt == paramNum)
      PwmGeneration::SetFslip(parm_Get(PARAM_fslipspnt));
   else if (PARAM_ampnom == paramNum)
      PwmGeneration::SetAmpnom(parm_Get(PARAM_ampnom));
   else if (PARAM_syncmode == paramNum)
   {
      if (parm_GetInt(PARAM_syncmode))
         Encoder::EnableSyncMode();
      else
         Encoder::DisableSyncMode();
   }
   else
   {
      SetCurrentLimitThreshold();

      Encoder::SetFilterConst(parm_GetInt(PARAM_encflt));
      Encoder::SetImpulsesPerTurn(parm_GetInt(PARAM_numimp));

      MotorVoltage::SetWeakeningFrq(parm_Get(PARAM_fweak));
      MotorVoltage::SetBoost(parm_GetInt(PARAM_boost));
      MotorVoltage::SetMinFrq(parm_Get(PARAM_fmin));
      MotorVoltage::SetMaxFrq(parm_Get(PARAM_fmax));
      SineCore::SetMinPulseWidth(parm_GetInt(PARAM_minpulse));

      Throttle::potmin[0] = parm_GetInt(PARAM_potmin);
      Throttle::potmax[0] = parm_GetInt(PARAM_potmax);
      Throttle::potmin[1] = parm_GetInt(PARAM_pot2min);
      Throttle::potmax[1] = parm_GetInt(PARAM_pot2max);
      Throttle::brknom = parm_GetInt(PARAM_brknom);
      Throttle::brknompedal = parm_GetInt(PARAM_brknompedal);
      Throttle::brkPedalRamp = parm_GetInt(PARAM_brkpedalramp);
      Throttle::brkmax = parm_GetInt(PARAM_brkmax);
      Throttle::idleSpeed = parm_GetInt(PARAM_idlespeed);
      Throttle::speedkp = parm_Get(PARAM_speedkp);
      Throttle::speedflt = parm_GetInt(PARAM_speedflt);
      Throttle::idleThrotLim = parm_Get(PARAM_idlethrotlim);
   }
}

extern "C" int main(void)
{
   clock_setup();
   tim_setup();
   DigIo::Init();
   usart_setup();
   nvic_setup();
   PwmGeneration::PwmInit();
   init_timsched();
   AnaIn::Init();
   Encoder::Init();
   term_Init(TERM_USART);
   can_setup();
   parm_load();
   parm_Change(PARAM_LAST);
   MotorVoltage::SetMaxAmp(SineCore::MAXAMP);

   create_task(Ms100Task, PRIO_GRP1, 0, 100);
   create_task(Ms10Task,  PRIO_GRP1, 1, 10);
   create_task(Ms1Task,   PRIO_GRP1, 2, 1);

   DigIo::Set(Pin::prec_out);

   term_Run();

   return 0;
}

