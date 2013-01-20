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

#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/f1/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/scb.h>
#include <libopencm3/stm32/f1/dma.h>
#include <libopencm3/stm32/nvic.h>
#include "stm32_sine.h"
#include "stm32_timsched.h"
#include "terminal.h"
#include "params.h"
#include "hwdefs.h"
#include "digio.h"
#include "sine_core.h"
#include "slip_ctl.h"
#include "fu.h"
#include "hwinit.h"
#include "anain.h"
#include "temp_meas.h"
#include "param_save.h"

#define MOD_OFF   0
#define MOD_SLIP  1
#define MOD_CTRL  2
#define MOD_FSPNT 3

//for average
#define HIS_SIZE 20

#define NUM_CTR_VAL (sizeof(timdata) / sizeof(timdata[0]))

static u8  pwmdigits;
static u16 pwmfrq;
static u16 timdata[10];

void tim1_brk_isr(void)
{
   timer_disable_irq(PWM_TIMER, TIM_DIER_BIE);
   parm_SetDig(VALUE_opmode, 0);
   digio_set(err_out);
}

static void UpdFrq(u32fp frq)
{
   int ampnom = parm_GetInt(VALUE_ampnom);
   int _amp = fu_GetAmpPerc(frq, ampnom);
   sin_SetAmp(_amp);
   parm_SetDig(VALUE_amp, _amp);
   parm_SetFlt(VALUE_fstat, frq);
   sin_SetFrq(frq);
}

static u32 GetImpulsesFiltered(u32 last)
{
   u16 numData = NUM_CTR_VAL - REV_CNT_DMA_CNDTR;
   int filter = parm_GetInt(PARAM_speedflt);

   for (int i = 0; i < numData; i++)
   {
      last = (timdata[i] + (filter - 1) * last) / filter;
   }

   REV_CNT_DMA_CCR &= ~DMA_CCR4_EN;
   REV_CNT_DMA_CNDTR = NUM_CTR_VAL;
   REV_CNT_DMA_CCR |= DMA_CCR4_EN;
   parm_SetDig(VALUE_ctr, last);

   return last;
}

/* Calculate dutycycles */
void pwm_timer_isr(void)
{
   u16 last = timer_get_counter(PWM_TIMER);

   /* Clear interrupt pending flag */
   TIM_SR(PWM_TIMER) &= ~TIM_SR_UIF;

   u32 DutyCycle[3];
   u8 shiftCor = SINE_DIGITS - pwmdigits;
   static u16 timval = 0;
   timval = GetImpulsesFiltered(timval);
   s32fp curFrq = parm_Get(VALUE_fstat);
   u32fp newFrq = ControlSlip(timval, curFrq);

   if (MOD_SLIP == parm_GetInt(VALUE_opmode))
   {
      UpdFrq(newFrq);
   }

   sin_Step(DutyCycle);
   /* Match to PWM resolution */
   DutyCycle[0] >>= shiftCor;
   DutyCycle[1] >>= shiftCor;
   DutyCycle[2] >>= shiftCor;

   timer_set_oc_value(PWM_TIMER, TIM_OC1, DutyCycle[0]);
   timer_set_oc_value(PWM_TIMER, TIM_OC2, DutyCycle[1]);
   timer_set_oc_value(PWM_TIMER, TIM_OC3, DutyCycle[2]);

   parm_SetDig(VALUE_tm_meas, (timer_get_counter(PWM_TIMER) - last)/36);
}

static void PwmInit(void)
{
   pwmdigits = MIN_PWM_DIGITS + parm_GetInt(PARAM_pwmfrq);
   pwmfrq = tim_setup(pwmdigits, parm_GetInt(PARAM_deadtime));
   sin_SetPwmFrq(pwmfrq);
}

void Ms100Task(void)
{
   digio_toggle(led_out);

   parm_SetFlt(VALUE_slip, GetSlip());
   parm_SetFlt(VALUE_speed, GetSpeed());


   /* Only change direction when below certain rev */
   if (GetSpeed() < FP_FROMINT(100))
   {
      if (digio_get(fwd_in) && !digio_get(rev_in))
      {
         parm_SetDig(VALUE_dir, 1);
      }
      else if (!digio_get(fwd_in) && digio_get(rev_in))
      {
         parm_SetDig(VALUE_dir, -1);
      }
   }

   if (!digio_get(fwd_in) && !digio_get(rev_in))
   {
      parm_SetDig(VALUE_dir, 0);
   }

   sin_SetDir(parm_GetInt(VALUE_dir));

   parm_SetDig(VALUE_din_on, digio_get(on_in));
   parm_SetDig(VALUE_din_start, digio_get(start_in));
   parm_SetDig(VALUE_din_brake, digio_get(brake_in));
   parm_SetDig(VALUE_din_mprot, digio_get(mprot_in));
   parm_SetDig(VALUE_din_forward, digio_get(fwd_in));
   parm_SetDig(VALUE_din_reverse, digio_get(rev_in));
   parm_SetDig(VALUE_din_emcystop, digio_get(emcystop_in));
   parm_SetDig(VALUE_din_bms, digio_get(bms_in));
}

void RampFrq(s32fp _frq)
{
    s32fp frqcalc = parm_Get(VALUE_fstat);

    if (frqcalc < _frq)
    {
        frqcalc += 1;
    }
    else if (frqcalc > _frq)
    {
        frqcalc -= 1;
    }
    sin_SetFrq(frqcalc);
    parm_SetFlt(VALUE_fstat, frqcalc);
}

static int CalcThrottle(int potval)
{
    int pot_min = parm_GetInt(PARAM_potmin);
    int pot_max = parm_GetInt(PARAM_potmax);
    int brknom  = parm_GetInt(PARAM_brknom);
    int pot_nom;
    int res = 0;

    parm_SetDig(VALUE_pot, potval);

    if ((potval < pot_min) || (potval > pot_max))
    {
        res = -1;
    }
    else
    {
        pot_nom = potval - pot_min;
        pot_nom = ((100 + brknom) * pot_nom) / (pot_max-pot_min);
        pot_nom -= brknom;
        parm_SetDig(VALUE_potnom, pot_nom);
    }

    return res;
}

static void CalcAmpAndSlip(void)
{
   s32fp potnom = parm_Get(VALUE_potnom);
   s32fp maxslip = parm_Get(PARAM_slipmax);
   s32fp ampmin = parm_Get(PARAM_ampmin);
   s32fp slipspnt = FP_MUL(maxslip, potnom) / 100;
   s32fp ampnom;

   if (potnom >= 0)
   {
      ampnom = ampmin + FP_MUL(FP_FROMINT(100) - ampmin, potnom) / 100;
   }
   else
   {
      ampnom = ampmin + FP_MUL(FP_FROMINT(100) - ampmin, -potnom) / 100;
   }
   parm_SetFlt(VALUE_ampnom, ampnom);

   SetSlipSpnt(slipspnt);
   parm_SetFlt(VALUE_slipspnt, slipspnt);
}

void Ms1Task(void)
{
   //asm volatile ("cpsie i");

   static s32fp il1rms = 0;
   static s32fp il2rms = 0;
   static int samples = 0;

   s32fp il1 = FP_FROMINT(anain_get(AIN_il1));
   s32fp il2 = FP_FROMINT(anain_get(AIN_il2));

   il1 -= parm_Get(PARAM_il1ofs);
   il2 -= parm_Get(PARAM_il2ofs);

   il1 = FP_DIV(il1, parm_Get(PARAM_il1gain));
   il2 = FP_DIV(il2, parm_Get(PARAM_il2gain));

   parm_SetFlt(VALUE_il1, il1);
   parm_SetFlt(VALUE_il2, il2);

   il1 = il1<0?-il1:il1;
   il2 = il2<0?-il2:il2;

   il1rms += il1;
   il2rms += il2;
   samples++;

   if (samples == 256)
   {
      il1rms >>= 8;
      il2rms >>= 8;

      parm_SetFlt(VALUE_il1rms, il1rms);
      parm_SetFlt(VALUE_il2rms, il2rms);
      il1rms = 0;
      il2rms = 0;
      samples = 0;
   }
}

void Ms10Task(void)
{
    static int InitDone = 0;
    static int throttle = 0;
    static int udc = 0;
    int temphs;
    int tempm;
    int throtCur;
    int udcCur;
    int opmode = parm_GetInt(VALUE_opmode);

    //asm volatile ("cpsie i");

    temphs   = anain_get(AIN_tmphs);
    tempm    = anain_get(AIN_tmpm);
    throtCur = anain_get(AIN_throttle1);
    udcCur   = anain_get(AIN_udc);

    throttle = (throtCur + throttle * 31) / 32;
    udc = (udcCur + udc * 31) / 32;

    /* Shut down on implausible throttle value */
    if (CalcThrottle(throttle) < 0)
    {
        parm_SetDig(VALUE_opmode, 0);
    }

    CalcAmpAndSlip();

    /* switch on DC switch above threshold */
    if (udc > parm_GetInt(PARAM_udcsw))
    {
        if (digio_get(start_in))
        {
           digio_set(dcsw_out);
           digio_clear(err_out);
           digio_clear(prec_out);
           parm_SetDig(VALUE_opmode, MOD_SLIP);
        }
    }
    else
    {
       digio_set(prec_out);
    }

    parm_SetFlt(VALUE_tmphs, temp_JCurve(temphs));
    parm_SetFlt(VALUE_tmpm, temp_KTY83(tempm));
    parm_SetFlt(VALUE_udc, FP_DIV(FP_FROMINT(udc), parm_Get(PARAM_udcgain)));

    if (MOD_CTRL == opmode)
    {
        RampFrq(throttle);
        int _amp = fu_GetAmp(parm_Get(VALUE_fstat));
        sin_SetAmp(_amp);
        parm_SetDig(VALUE_amp, _amp);
    }
    if (MOD_FSPNT == opmode)
    {
       RampFrq(parm_Get(PARAM_fspnt));
       int _amp = fu_GetAmp(parm_Get(VALUE_fstat));
       sin_SetAmp(_amp);
       parm_SetDig(VALUE_amp, _amp);
    }
    if (MOD_OFF == opmode)
    {
        //asm volatile ("cpsid i");
        InitDone = 0;

        parm_SetDig(VALUE_amp, 0);
        timer_disable_oc_output(PWM_TIMER, TIM_OC1);
        timer_disable_oc_output(PWM_TIMER, TIM_OC2);
        timer_disable_oc_output(PWM_TIMER, TIM_OC3);
        timer_disable_oc_output(PWM_TIMER, TIM_OC1N);
        timer_disable_oc_output(PWM_TIMER, TIM_OC2N);
        timer_disable_oc_output(PWM_TIMER, TIM_OC3N);
        digio_clear(dcsw_out);
        digio_set(err_out);
        //asm volatile ("cpsie i");
    }
    else if (!InitDone)
    {
        PwmInit();
        digio_clear(err_out);
        InitDone = 1;
    }
}

extern void parm_Change(PARAM_NUM ParamNum)
{
   ParamNum = ParamNum;
   SetImpulsePerRev(parm_GetInt(PARAM_num_imp));
   SetMotorPolePairs(2);
   SetMaxFrq(parm_Get(PARAM_fmax));
   SetControlKp(parm_Get(PARAM_slipkp));
   SetMinSpeed(parm_Get(PARAM_minspeed));

   timer_set_oc_value(OVER_CUR_TIMER, TIM_OC2, parm_GetInt(PARAM_ucurlim));
   timer_set_oc_value(OVER_CUR_TIMER, TIM_OC3, parm_GetInt(PARAM_ocurlim));

   fu_SetWeakeningFrq(parm_Get(PARAM_fweak));
   fu_SetBoost(parm_GetInt(PARAM_boost));
   fu_SetMaxAmp(parm_GetInt(PARAM_ampmax));
   sin_SetMinPulseWidth(parm_GetInt(PARAM_minpulse));
   sin_SetPwmMode(parm_GetInt(PARAM_pwmmode));
}

int main(void)
{
   clock_setup();
   digio_init();
   usart_setup();
   nvic_setup();
   dma_setup(timdata, NUM_CTR_VAL);
   PwmInit();
   init_timsched();
   anain_init();
   parm_load();
   parm_Change(0);

   parm_SetDig(VALUE_opmode, 0);

   create_task(Ms100Task, PRIO_GRP1, 0, 100);
   create_task(Ms10Task,  PRIO_GRP1, 1, 10);
   create_task(Ms1Task,   PRIO_GRP1, 2, 1);

   term_Run(TERM_USART);

   return 0;
}

