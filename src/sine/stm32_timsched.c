/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2010 Johannes Huebner <dev@johanneshuebner.com>
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
#define STM32F1

#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/f1/rcc.h>
#include "stm32_timsched.h"

void (*Functions[MAX_TASKS]) (void);

u16 Periods[MAX_TASKS];

/* Helper macro */
#define APPEND(a,b) a##b

/* Map Priority groups to timers */
#define PRIOGRP_ENTRY(t,c,g) TIM##t,
const u32 GrpToTimer[] = { PRIORITY_GROUPS };
#undef PRIOGRP_ENTRY

/* Map Priority groups to nvic lines */
#define PRIOGRP_ENTRY(t,c,g) APPEND(NVIC_TIM##t,_IRQ),
const u32 GrpToNvic[] = { PRIORITY_GROUPS };
#undef PRIOGRP_ENTRY

/* Map Priority groups to RCC lines */
#define PRIOGRP_ENTRY(t,c,g) APPEND(RCC_APB1ENR_TIM##t,EN),
const u32 GrpToRCC[] = { PRIORITY_GROUPS };
#undef PRIOGRP_ENTRY

/* return CCRc of TIMt */
#define TIM_CCR(t,c) (*(volatile u32 *)(&TIM_CCR1(t) + (c)))

/* We use preprocessor code generation instead of actual functions
   to have less function calls in the ISRs */

/* Template for Timer channel ISR */
/* Check for event flags, clear event flags */
/* increase the CCR by the task period and call the task */
#define TIM_ISR(tn,c)                                       \
     if (TIM_SR(TIM##tn) & (1 << (1+c)))                    \
     {                                                      \
         TIM_SR (TIM##tn)   &= ~(1 << (1+c));               \
         Functions[FUNCIDX_TIMx(tn,c)]();                   \
         TIM_CCR(TIM##tn,c) += Periods[FUNCIDX_TIMx(tn,c)]; \
     }

/* Template for 4 channel timer ISR */
#define TIM_ISR_4CHAN(tn) \
    TIM_ISR(tn,0)         \
    TIM_ISR(tn,1)         \
    TIM_ISR(tn,2)         \
    TIM_ISR(tn,3)

void tim2_isr(void)
{
  TIM_ISR_4CHAN(2)
}

static void nofunc(void)
{
}

s8 create_task(void (*Function)(void), SCHED_PRIOGRP PrioGrp, u8 PrioInGrp, u16 Period)
{
    volatile u32 *CCMR;
    volatile u32 *CCR;
             u32  CCMRFlag;


    if (PrioGrp >= PRIO_GRPLAST)
    {
        return -1;
    }
    /* TODO: check for channels on that timer */
    switch(PrioInGrp)
    {
        case 0:
            CCMR     = &TIM_CCMR1(GrpToTimer[PrioGrp]);
            CCMRFlag =  TIM_CCMR1_OC1M_ACTIVE;
            CCR      = &TIM_CCR1 (GrpToTimer[PrioGrp]);
            break;
        case 1:
            CCMR     = &TIM_CCMR1(GrpToTimer[PrioGrp]);
            CCMRFlag =  TIM_CCMR1_OC2M_ACTIVE;
            CCR      = &TIM_CCR2 (GrpToTimer[PrioGrp]);
            break;
        case 2:
            CCMR     = &TIM_CCMR2(GrpToTimer[PrioGrp]);
            CCMRFlag =  TIM_CCMR2_OC3M_ACTIVE;
            CCR      = &TIM_CCR3 (GrpToTimer[PrioGrp]);
            break;
        case 3:
            CCMR     = &TIM_CCMR2(GrpToTimer[PrioGrp]);
            CCMRFlag =  TIM_CCMR2_OC4M_ACTIVE;
            CCR      = &TIM_CCR4 (GrpToTimer[PrioGrp]);
            break;
        default:
            return -1;
    }

    /* Disable timer */
    TIM_CR1  (GrpToTimer[PrioGrp])&= ~TIM_CR1_CEN;
    *CCMR                         |= CCMRFlag;
    *CCR                           = Period;
    /* Enable interrupt for that channel */
    TIM_DIER (GrpToTimer[PrioGrp])|= TIM_DIER_CC1IE << PrioInGrp;

    /* Assign task function and period */
    Functions[FUNCIDX_GRPx(PrioGrp,PrioInGrp)] = Function;
    Periods  [FUNCIDX_GRPx(PrioGrp,PrioInGrp)] = Period * 20;

    /* Reset counter */
    TIM_CNT (GrpToTimer[PrioGrp]) = 0;

    /* Enable timer */
    TIM_CR1  (GrpToTimer[PrioGrp])|= TIM_CR1_CEN;

    return 0;
}

void change_interval(SCHED_PRIOGRP PrioGrp, u8 PrioInGrp, u16 Period)
{
    Periods[FUNCIDX_GRPx(PrioGrp,PrioInGrp)] = Period * 10;
}

void init_timsched(void)
{
    SCHED_PRIOGRP PrioGrp;
    int Idx;

    for (PrioGrp = PRIO_GRP1; PrioGrp < PRIO_GRPLAST; PrioGrp++)
    {
        /* Enable TIMx clock */
        /* TODO: timers on APB2 */
        rcc_peripheral_enable_clock(&RCC_APB1ENR,GrpToRCC[PrioGrp]);
        /* Enable interrupt */
        nvic_enable_irq(GrpToNvic[PrioGrp]);
        /* Set priority */
        nvic_set_priority(GrpToNvic[PrioGrp], 0xf << 4);
        /* Setup timers upcounting and auto preload enable */
        TIM_CR1  (GrpToTimer[PrioGrp]) = TIM_CR1_DIR_UP | TIM_CR1_ARPE;
        /* Set prescaler to count at 10 kHz = 36 MHz/3600 */
        TIM_PSC  (GrpToTimer[PrioGrp]) = 3600;
        /* Maximum counter value */
        TIM_ARR  (GrpToTimer[PrioGrp]) = 0xFFFF;
   }

   /* It seems that status flags are set even when
      the output compare unit isn't active. Thus,
      we initialize all function pointers to
      the empty function nofunc */
   for (Idx = 0; Idx < MAX_TASKS; Idx++)
   {
       Periods[Idx] = 0xFFFF;
       Functions[Idx] = nofunc;
   }
}
