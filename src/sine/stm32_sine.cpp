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
#include "printf.h"

#define RMS_SAMPLES 256
#define SQRT2OV1 0.707106781187

static volatile uint32_t timeTick = 0; //ms since system start
//Current sensor offsets are determined at runtime
 s32fp ilofs[2] = { 0, 0 };
//Precise control of executing the boost controller
static bool runChargeControl = false;

#ifdef HWCONFIG_TESLA
static void GetTemps(s32fp& tmphs, s32fp &tmpm)
{
   static int tmphsMax = 0, tmpmMax = 0;
   static int input = 0;

   int tmphsi = AnaIn::Get(AnaIn::tmphs);
   int tmpmi = AnaIn::Get(AnaIn::tmpm);

   switch (input)
   {
      case 0:
         DigIo::Clear(Pin::temp0_out);
         DigIo::Clear(Pin::temp1_out);
         input = 1;
         //Handle mux inputs 11
         tmphs = 0; //not connected
         tmpm = TempMeas::Lookup(tmpmi, TempMeas::TEMP_TESLA_100K);
         break;
      case 1:
         DigIo::Set(Pin::temp0_out);
         DigIo::Clear(Pin::temp1_out);
         tmphsMax = 0;
         tmpmMax = 0;
         input = 2;
         //Handle mux inputs 00
         tmphs = TempMeas::Lookup(tmphsi, TempMeas::TEMP_TESLA_52K);
         tmpm = 0; //don't know yet
         break;
      case 2:
         DigIo::Clear(Pin::temp0_out);
         DigIo::Set(Pin::temp1_out);
         input = 3;
         //Handle mux inputs 01
         tmphs = TempMeas::Lookup(tmphsi, TempMeas::TEMP_TESLA_52K);
         tmpm = 0; //don't know yet
         break;
      case 3:
         DigIo::Set(Pin::temp0_out);
         DigIo::Set(Pin::temp1_out);
         input = 0;
         //Handle mux inputs 10
         tmphs = TempMeas::Lookup(tmphsi, TempMeas::TEMP_TESLA_52K);
         tmpm = TempMeas::Lookup(tmpmi, TempMeas::TEMP_TESLA_100K);
         break;
   }

   tmphs = tmphsMax = MAX(tmphsMax, tmphs);
   tmpm = tmpmMax = MAX(tmpmMax, tmpm);
}
#else
static void GetTemps(s32fp& tmphs, s32fp &tmpm)
{
   TempMeas::Sensors snshs = (TempMeas::Sensors)Param::GetInt(Param::snshs);
   TempMeas::Sensors snsm = (TempMeas::Sensors)Param::GetInt(Param::snsm);

   int tmphsi = AnaIn::Get(AnaIn::tmphs);
   int tmpmi = AnaIn::Get(AnaIn::tmpm);

   tmpm = TempMeas::Lookup(tmpmi, snsm);
   tmphs = TempMeas::Lookup(tmphsi, snshs);
}
#endif // HWCONFIG_TESLA

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
      if (Param::GetInt(Param::cruisemode) == BUTTON)
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
   int32_t dir = Param::GetInt(Param::dir);

   DigIo::Toggle(Pin::led_out);

   Param::SetDig(Param::speed, Encoder::GetSpeed());

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
      Param::SetDig(Param::din_ocur, 1);
   }
   else
   {
      Param::SetDig(Param::din_ocur, 0);
   }

   CruiseControl();

   Param::SetDig(Param::dir, dir);
   Param::SetDig(Param::din_cruise, DigIo::Get(Pin::cruise_in));
   Param::SetDig(Param::din_start, DigIo::Get(Pin::start_in));
   Param::SetDig(Param::din_brake, DigIo::Get(Pin::brake_in));
   Param::SetDig(Param::din_mprot, DigIo::Get(Pin::mprot_in));
   Param::SetDig(Param::din_forward, DigIo::Get(Pin::fwd_in));
   Param::SetDig(Param::din_reverse, DigIo::Get(Pin::rev_in));
   Param::SetDig(Param::din_emcystop, DigIo::Get(Pin::emcystop_in));
   Param::SetDig(Param::din_bms, DigIo::Get(Pin::bms_in));

   //s32fp data[2] = { Param::Get(Param::speed), Param::Get(Param::udc) };

   //can_send(0x180, (uint8_t*)data, sizeof(data));
   Can::SendAll();
}

static void CalcAmpAndSlip(void)
{
   s32fp fslipmin = Param::Get(Param::fslipmin);
   s32fp ampmin = Param::Get(Param::ampmin);
   s32fp potnom = Param::Get(Param::potnom);
   s32fp ampnom;
   s32fp fslipspnt;

   if (potnom >= 0)
   {
      ampnom = ampmin + 2 * potnom;

      if (potnom >= FP_FROMINT(50))
      {
         s32fp fslipmax = Param::Get(Param::fslipmax);
         s32fp fpconst =  Param::Get(Param::fpconst);
         s32fp fstat = Param::Get(Param::fstat);
         s32fp fweak = Param::Get(Param::fweak);
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
      u32fp brkrampstr = (u32fp)Param::Get(Param::brkrampstr);

      ampnom = -potnom;
      fslipspnt = -fslipmin;
      if (Encoder::GetRotorFrequency() < brkrampstr)
      {
         ampnom = FP_TOINT(FP_DIV(Encoder::GetRotorFrequency(), brkrampstr) * ampnom);
      }
      //This works because ampnom = -potnom
      if (ampnom >= -Param::Get(Param::brkout))
         DigIo::Set(Pin::brk_out);
      else
         DigIo::Clear(Pin::brk_out);
   }

   ampnom = MIN(ampnom, FP_FROMINT(100));

   Param::SetFlt(Param::ampnom, IIRFILTER(Param::Get(Param::ampnom), ampnom, 3));
   Param::SetFlt(Param::fslipspnt, IIRFILTER(Param::Get(Param::fslipspnt), fslipspnt, 3));
   PwmGeneration::SetAmpnom(Param::Get(Param::ampnom));
   PwmGeneration::SetFslip(Param::Get(Param::fslipspnt));
}

static void CalcAndOutputTemp()
{
   static int temphsFlt = 0;
   static int tempmFlt = 0;
   int pwmgain = Param::GetInt(Param::pwmgain);
   int pwmofs = Param::GetInt(Param::pwmofs);
   int pwmfunc = Param::GetInt(Param::pwmfunc);
   int tmpout;
   s32fp tmphs = 0, tmpm = 0;

   GetTemps(tmphs, tmpm);

   temphsFlt = IIRFILTER(tmphs, temphsFlt, 15);
   tempmFlt = IIRFILTER(tmpm, tempmFlt, 18);

   switch (pwmfunc)
   {
      default:
      case PWM_FUNC_TMPM:
         tmpout = FP_TOINT(tmpm) * pwmgain + pwmofs;
         break;
      case PWM_FUNC_TMPHS:
         tmpout = FP_TOINT(tmphs) * pwmgain + pwmofs;
         break;
      case PWM_FUNC_SPEED:
         tmpout = Param::Get(Param::speed) * pwmgain + pwmofs;
         break;
   }

   tmpout = MIN(0xFFFF, MAX(0, tmpout));

   timer_set_oc_value(OVER_CUR_TIMER, TIM_OC4, tmpout);

   Param::SetFlt(Param::tmphs, tmphs);
   Param::SetFlt(Param::tmpm, tmpm);
}

static s32fp ProcessUdc()
{
   static int32_t udc = 0;
   s32fp udcfp;
   s32fp udcmin = Param::Get(Param::udcmin);
   s32fp udcmax = Param::Get(Param::udcmax);
   s32fp udclim = Param::Get(Param::udclim);
   s32fp udcgain = Param::Get(Param::udcgain);
   s32fp udcnom = Param::Get(Param::udcnom);
   s32fp fweak = Param::Get(Param::fweak);
   s32fp boost = Param::Get(Param::boost);
   int udcofs = Param::GetInt(Param::udcofs);

   //Calculate "12V" supply voltage from voltage divider on mprot pin
   //1.2/(4.7+1.2)/3.34*4095 = 249 -> make it a bit less for pin losses etc
   Param::SetFlt(Param::uaux, FP_DIV(AnaIn::Get(AnaIn::uaux), 248));
   udc = IIRFILTER(udc, AnaIn::Get(AnaIn::udc), 2);
   udcfp = FP_DIV(FP_FROMINT(udc - udcofs), udcgain);

   if (udcfp < udcmin || udcfp > udcmax)
      DigIo::Set(Pin::vtg_out);
   else
      DigIo::Clear(Pin::vtg_out);

   if (udcfp > udclim)
   {
      Param::SetDig(Param::opmode, MOD_OFF);
      DigIo::Set(Pin::err_out);
      ErrorMessage::Post(ERR_OVERVOLTAGE);
   }

   if (udcnom > 0)
   {
      s32fp udcdiff = udcfp - udcnom;
      //increase fweak on voltage above nominal
      fweak = FP_MUL(fweak, (FP_FROMINT(1) + FP_DIV(udcdiff, udcnom)));
      //decrease boost on voltage below nominal
      boost = FP_MUL(boost, (FP_FROMINT(1) - FP_DIV(udcdiff, udcnom)));
   }

   Param::SetFlt(Param::udc, udcfp);
   Param::SetFlt(Param::fweakcalc, fweak);
   Param::SetFlt(Param::boostcalc, boost);
   MotorVoltage::SetWeakeningFrq(fweak);
   MotorVoltage::SetBoost(FP_TOINT(boost));

   return udcfp;
}

static void CalcFancyValues()
{
   const s32fp twoPi = FP_FROMFLT(2*3.141593);
   s32fp amp = Param::Get(Param::amp);
   s32fp speed = Param::Get(Param::speed);
   s32fp il1 = Param::Get(Param::il1);
   s32fp il2 = Param::Get(Param::il2);
   s32fp udc = Param::Get(Param::udc);
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

   if (Param::GetInt(Param::opmode) < MOD_BOOST)
   {
      idc = FP_MUL(FP_MUL(fac, FOC::id), FP_FROMFLT(0.7071));
      Param::SetFlt(Param::idc, idc);
   }

   Param::SetFlt(Param::id, FOC::id);
   Param::SetFlt(Param::iq, FOC::iq);
   Param::SetFlt(Param::uac, uac);
   Param::SetFlt(Param::pf, pf);
   Param::SetFlt(Param::p, p);
   Param::SetFlt(Param::q, q);
   Param::SetFlt(Param::s, s);
   Param::SetFlt(Param::t, t);
}

static void CalcThrottle()
{
   int potval = AnaIn::Get(AnaIn::throttle1);
   int pot2val = AnaIn::Get(AnaIn::throttle2);
   bool brake = DigIo::Get(Pin::brake_in);
   int throtSpnt, idleSpnt, cruiseSpnt, derateSpnt, finalSpnt;

   Param::SetDig(Param::pot, potval);
   Param::SetDig(Param::pot2, pot2val);

   /* Error light on implausible value */
   if (!Throttle::CheckAndLimitRange(&potval, 0))
   {
      DigIo::Set(Pin::err_out);
      ErrorMessage::Post(ERR_THROTTLE1);
   }

   Throttle::CheckAndLimitRange(&pot2val, 1);

   throtSpnt = Throttle::CalcThrottle(potval, pot2val, brake);
   idleSpnt = Throttle::CalcIdleSpeed(Encoder::GetSpeed());
   derateSpnt = Throttle::TemperatureDerate(Param::Get(Param::tmphs));
   cruiseSpnt = Throttle::CalcCruiseSpeed(Encoder::GetSpeed());

   if (Param::GetInt(Param::idlemode) == IDLE_MODE_ALWAYS ||
       (Param::GetInt(Param::idlemode) == IDLE_MODE_NOBRAKE && !brake) ||
       (Param::GetInt(Param::idlemode) == IDLE_MODE_CRUISE && !brake && DigIo::Get(Pin::cruise_in)))
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
#ifndef HWCONFIG_TESLA
   if (Param::GetInt(Param::din_bms))
   {
      if (finalSpnt >= 0)
         finalSpnt = (finalSpnt * Param::GetInt(Param::bmslimhigh)) / 100;
      else
         finalSpnt = -(finalSpnt * Param::GetInt(Param::bmslimlow)) / 100;
   }
#endif
   if (derateSpnt < 100)
   {
      if (finalSpnt >= 0)
         finalSpnt = MIN(finalSpnt, derateSpnt);
      else
         finalSpnt = MAX(finalSpnt, -derateSpnt);
      DigIo::Set(Pin::err_out);
   }

   Param::SetDig(Param::potnom, finalSpnt);
}

static void Ms10Task(void)
{
   static int initWait = 0;
   int opmode = Param::GetInt(Param::opmode);
   int chargemode = Param::GetInt(Param::chargemode);
   s32fp udc = ProcessUdc();

   CalcThrottle();
   CalcAndOutputTemp();
   Encoder::UpdateRotorFrequency(10);

   if (MOD_RUN == opmode)
   {
      CalcAmpAndSlip();
   }

   /* switch on DC switch above threshold but only if
    * - throttle is not pressed
    * - start pin is high
    * - motor protection switch and emcystop is high (=inactive)
    */
   if (udc >= Param::Get(Param::udcsw) && Param::GetInt(Param::potnom) <= 0)
   {
      if (DigIo::Get(Pin::emcystop_in) &&
          DigIo::Get(Pin::mprot_in))
      {
         /* Switch to charge mode if
          * - Charge mode is enabled
          * - Fwd AND Rev are high
          */
         if (DigIo::Get(Pin::fwd_in) && DigIo::Get(Pin::rev_in) && !DigIo::Get(Pin::bms_in) && chargemode >= MOD_BOOST)
         {
            //Do NOT close main relay in buck mode!!
            if (chargemode == MOD_BOOST)
               DigIo::Set(Pin::dcsw_out);
            DigIo::Clear(Pin::err_out);
            DigIo::Clear(Pin::prec_out);
            Param::SetDig(Param::opmode, chargemode);
            ErrorMessage::UnpostAll();
         }
         else if (DigIo::Get(Pin::start_in))
         {
            DigIo::Set(Pin::dcsw_out);
            DigIo::Clear(Pin::err_out);
            DigIo::Clear(Pin::prec_out);
            Param::SetDig(Param::opmode, MOD_RUN);
            ErrorMessage::UnpostAll();
            runChargeControl = false;
         }
      }
   }

#ifndef HWCONFIG_TESLA
   if (opmode >= MOD_BOOST && DigIo::Get(Pin::bms_in))
   {
      opmode = MOD_OFF;
      Param::SetDig(Param::opmode, opmode);
   }
#endif

   if (MOD_OFF == opmode)
   {
      initWait = 50;

      Param::SetDig(Param::amp, 0);
      PwmGeneration::SetOpmode(MOD_OFF);
      DigIo::Clear(Pin::dcsw_out);
      Throttle::cruiseSpeed = -1;
      runChargeControl = false;
   }
   else if (0 == initWait)
   {
      PwmGeneration::PwmInit(); //this applies new deadtime and pwmfrq
      PwmGeneration::SetOpmode(opmode); //this enables the outputs for the given mode
      runChargeControl = (opmode == MOD_BOOST || opmode == MOD_BUCK);
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
   s32fp ocurlim = Param::Get(Param::ocurlim);
   //We use the average offset and gain values because we only
   //have one reference channel per polarity
   s32fp iofs = (ilofs[0] + ilofs[1]) / 2;
   s32fp igain= (Param::Get(Param::il1gain) + Param::Get(Param::il2gain)) / 2;

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
   int opmode = Param::GetInt(Param::opmode);

   timeTick++;
   ErrorMessage::SetTime(timeTick);

   for (int i = 0; i < 2; i++)
   {
      s32fp il;
      il = FP_FROMINT(AnaIn::Get((AnaIn::AnaIns)(AnaIn::il1+i)));

      if (MOD_OFF == opmode)
      {
         ilofs[i] = il;
         SetCurrentLimitThreshold();
      }

      il -= ilofs[i];

      il = FP_DIV(il, Param::Get((Param::PARAM_NUM)(Param::il1gain+i)));

      Param::SetFlt((Param::PARAM_NUM)(Param::il1+i), il);

      il = ABS(il);

      ilrms[i] = MAX(il, ilrms[i]);

      if (samples == 0)
      {
         ilrms[i] = FP_MUL(ilrms[i], FP_FROMFLT(SQRT2OV1));
         Param::SetFlt((Param::PARAM_NUM)(Param::il1rms+i), ilrms[i]);
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
      speedCnt = Param::GetInt(Param::speedgain) / (2 * Param::GetInt(Param::speed));
   }
   else
   {
      speedCnt--;
   }

   if (runChargeControl)
   {
      ilFlt = IIRFILTER(ilFlt, ilMax, Param::GetInt(Param::chargeflt));
      iSpntFlt = IIRFILTER(iSpntFlt, Param::Get(Param::chargecur) << 8, 11);

      s32fp ampnom = FP_MUL(Param::GetInt(Param::chargekp), (iSpntFlt - ilFlt));
      Param::SetFlt(Param::ampnom, ampnom);
      if (opmode == MOD_BOOST)
         Param::SetFlt(Param::idc, (FP_MUL((FP_FROMINT(100) - ampnom), ilFlt) / 100) >> 8);
      else
         Param::SetFlt(Param::idc, ilFlt >> 8);
      ampnom = MIN(ampnom, FP_FROMINT(Throttle::TemperatureDerate(Param::Get(Param::tmphs))));
      ampnom = MAX(0, ampnom);
      ampnom = MIN(FP_FROMINT(96), ampnom);
      PwmGeneration::SetAmpnom(ampnom);
   }
   else
   {
      iSpntFlt = 0;
   }
}

/** This function is called when the user changes a parameter */
extern void parm_Change(Param::PARAM_NUM paramNum)
{
   if (Param::fslipspnt == paramNum)
      PwmGeneration::SetFslip(Param::Get(Param::fslipspnt));
   else if (Param::ampnom == paramNum)
      PwmGeneration::SetAmpnom(Param::Get(Param::ampnom));
   else
   {
      //Allow ~20% more
      int minPulseTime = (800000 * Param::GetInt(Param::polepairs)) / (Param::GetInt(Param::fmax) * Param::GetInt(Param::numimp));
      SetCurrentLimitThreshold();

      Encoder::SetFilterConst(Param::GetInt(Param::encflt));
      Encoder::SetMode((bool)Param::GetInt(Param::encmode), (bool)Param::GetInt(Param::syncmode));
      Encoder::SetImpulsesPerTurn(Param::GetInt(Param::numimp));
      Encoder::SetMinPulseTime(minPulseTime);

      MotorVoltage::SetMinFrq(Param::Get(Param::fmin));
      MotorVoltage::SetMaxFrq(Param::Get(Param::fmax));
      SineCore::SetMinPulseWidth(Param::GetInt(Param::minpulse));

      Throttle::potmin[0] = Param::GetInt(Param::potmin);
      Throttle::potmax[0] = Param::GetInt(Param::potmax);
      Throttle::potmin[1] = Param::GetInt(Param::pot2min);
      Throttle::potmax[1] = Param::GetInt(Param::pot2max);
      Throttle::brknom = Param::GetInt(Param::brknom);
      Throttle::brknompedal = Param::GetInt(Param::brknompedal);
      Throttle::brkPedalRamp = Param::GetInt(Param::brkpedalramp);
      Throttle::brkmax = Param::GetInt(Param::brkmax);
      Throttle::idleSpeed = Param::GetInt(Param::idlespeed);
      Throttle::speedkp = Param::Get(Param::speedkp);
      Throttle::speedflt = Param::GetInt(Param::speedflt);
      Throttle::idleThrotLim = Param::Get(Param::idlethrotlim);
   }
}

extern "C" int main(void)
{
   clock_setup();
   tim_setup();
   AnaIn::Init();
   DigIo::Init();
   usart_setup();
   nvic_setup();
   init_timsched();
   Encoder::Init();
   term_Init(TERM_USART);
   Can::Setup();
   parm_load();
   parm_Change(Param::PARAM_LAST);
   MotorVoltage::SetMaxAmp(SineCore::MAXAMP);

   create_task(Ms100Task, PRIO_GRP1, 0, 100);
   create_task(Ms10Task,  PRIO_GRP1, 1, 10);
   create_task(Ms1Task,   PRIO_GRP1, 2, 1);

   //Always enable precharge except in buck mode.
   if (!(DigIo::Get(Pin::fwd_in) && DigIo::Get(Pin::rev_in) && Param::GetInt(Param::chargemode) == MOD_BUCK))
      DigIo::Set(Pin::prec_out);

   term_Run();

   return 0;
}

