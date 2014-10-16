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
#define STM32F1
#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include "hwdefs.h"
#include "hwinit.h"

void clock_setup(void)
{
   RCC_CLOCK_SETUP();

	rcc_set_ppre1(RCC_CFGR_PPRE1_HCLK_DIV2);
	rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV8);

   //The reset value for PRIGROUP (=0) is not actually a defined
   //value. Explicitly set 16 preemtion priorities
   SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_PRIGROUP_GROUP16_NOSUB;

   /* Enable all present GPIOx clocks. (whats with GPIO F and G?)*/
   rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
   rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
   rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN);
   rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPDEN);
   rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPEEN);
   rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPGEN);

   #if (HWCONFIG==HWCONFIG_OLIMEX)
   rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USART3EN);
   #elif (HWCONFIG == HWCONFIG_SB5COM)
   rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_USART1EN);
   #endif

   /* Enable TIM1 clock */
   rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_TIM1EN);

   /* Enable TIM3 clock */
   rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM3EN);

   /* Enable TIM4 clock */
   rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM4EN);

   /* Enable DMA1 clock */
   rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA1EN);

   /* Enable ADC clock */
   rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);

   /* Enable CRC clock */
   rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_CRCEN);
}

void usart_setup(void)
{
    gpio_set_mode(TERM_USART_TXPORT, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, TERM_USART_TXPIN);

    /* Setup UART parameters. */
    usart_set_baudrate(TERM_USART, USART_BAUDRATE);
    usart_set_databits(TERM_USART, 8);
    usart_set_stopbits(TERM_USART, USART_STOPBITS_2);
    usart_set_mode(TERM_USART, USART_MODE_TX_RX);
    usart_set_parity(TERM_USART, USART_PARITY_NONE);
    usart_set_flow_control(TERM_USART, USART_FLOWCONTROL_NONE);
    //USART_CR1(TERM_USART) |= USART_CR1_RXNEIE;

    /* Finally enable the USART. */
    usart_enable(TERM_USART);
}

/* Enable timer interrupts */
void nvic_setup(void)
{
   nvic_enable_irq(PWM_TIMER_IRQ);
   nvic_set_priority(PWM_TIMER_IRQ, 0 << 4);

   nvic_enable_irq(NVIC_TIM1_BRK_IRQ);
   nvic_set_priority(NVIC_TIM1_BRK_IRQ, 0 << 4);

   /*nvic_enable_irq(REV_CNT_IRQ);
   nvic_set_priority(REV_CNT_IRQ, 1 << 4);

   nvic_enable_irq(NVIC_USART1_IRQ);
   nvic_set_priority(NVIC_USART1_IRQ, 3 << 4);*/
}

uint16_t tim_setup(uint16_t pwmdigits, uint16_t deadtime, int pwmpol)
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

   //clear CC1P (capture compare enable register, active high)
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

   /* Output enable */
   timer_enable_oc_output(PWM_TIMER, TIM_OC1);
   timer_enable_oc_output(PWM_TIMER, TIM_OC2);
   timer_enable_oc_output(PWM_TIMER, TIM_OC3);
   timer_enable_oc_output(PWM_TIMER, TIM_OC1N);
   timer_enable_oc_output(PWM_TIMER, TIM_OC2N);
   timer_enable_oc_output(PWM_TIMER, TIM_OC3N);

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

   /* start out with 50:50 duty cycle */
   timer_set_oc_value(PWM_TIMER, TIM_OC1, pwmmax / 2);
   timer_set_oc_value(PWM_TIMER, TIM_OC2, pwmmax / 2);
   timer_set_oc_value(PWM_TIMER, TIM_OC3, pwmmax / 2);

   timer_enable_counter(PWM_TIMER);

   /*** Setup over/undercurrent timer */
   timer_disable_counter(OVER_CUR_TIMER);
   //edge aligned PWM
   timer_set_alignment(OVER_CUR_TIMER, TIM_CR1_CMS_EDGE);
   timer_enable_preload(OVER_CUR_TIMER);
   /* PWM mode 1 and preload enable */
   timer_set_oc_mode(OVER_CUR_TIMER, TIM_OC2, TIM_OCM_PWM1);
   timer_set_oc_mode(OVER_CUR_TIMER, TIM_OC3, TIM_OCM_PWM1);
   timer_set_oc_mode(OVER_CUR_TIMER, TIM_OC4, TIM_OCM_PWM1);
   timer_enable_oc_preload(OVER_CUR_TIMER, TIM_OC2);
   timer_enable_oc_preload(OVER_CUR_TIMER, TIM_OC3);
   timer_enable_oc_preload(OVER_CUR_TIMER, TIM_OC4);

   timer_set_oc_polarity_high(OVER_CUR_TIMER, TIM_OC2);
   timer_set_oc_polarity_high(OVER_CUR_TIMER, TIM_OC3);
   timer_set_oc_polarity_high(OVER_CUR_TIMER, TIM_OC4);
   timer_enable_oc_output(OVER_CUR_TIMER, TIM_OC2);
   timer_enable_oc_output(OVER_CUR_TIMER, TIM_OC3);
   timer_enable_oc_output(OVER_CUR_TIMER, TIM_OC4);
   timer_generate_event(OVER_CUR_TIMER, TIM_EGR_UG);
   timer_set_prescaler(OVER_CUR_TIMER, 0);
   /* PWM frequency */
   timer_set_period(OVER_CUR_TIMER, OCURMAX);
   timer_enable_counter(OVER_CUR_TIMER);

   /** setup gpio */
   gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO8 | GPIO9 | GPIO10);
   gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO13 | GPIO14 | GPIO15 | GPIO7 | GPIO8 | GPIO9);

   return PERIPH_CLK / (uint32_t)pwmmax;
}
