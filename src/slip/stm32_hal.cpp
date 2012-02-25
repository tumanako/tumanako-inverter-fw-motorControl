
extern "C" {
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/nvic.h>
#include <libopencm3/stm32/timer.h>
}

#include "stm32_hal.hpp"
#include "stm32_irq.hpp"
#include "hwdefs.h"


Stm32MotorControlHW::Stm32MotorControlHW()
{
   timerHandler = 0;
   lastRevTicks = 0;
   Stm32Irq::Register(IRQ_TIM1_UP, this);
   clock_setup();
   gpio_setup();
   tim_setup();
}

void Stm32MotorControlHW::SetDutyCycles(u16 DutyCycles[3])
{
   volatile u32 *pTimCcr = &TIM_CCR1(PWM_TIMER);

   for (int Idx = 0; Idx < 3; Idx++, pTimCcr++)
   {
      *pTimCcr = DutyCycles[Idx];
   }
}

u16 Stm32MotorControlHW::GetRevTicks()
{
   u16 curTicks = TIM_CNT(REV_CNT_TIMER);
   u16 tickDiff = curTicks - lastRevTicks;
   lastRevTicks = curTicks;
   return tickDiff;
}

u16 Stm32MotorControlHW::GetB6Temp()
{
   return 0;
}

u16 Stm32MotorControlHW::GetThrottle()
{
   return 0;
}

u16 Stm32MotorControlHW::GetBusVoltage()
{
   return 0;
}

bool Stm32MotorControlHW::IsReverseDrivingSelected()
{
   return false;
}

bool Stm32MotorControlHW::IsBrakePedalPressed()
{
   return false;
}

void Stm32MotorControlHW::SetMainBreaker(bool Close)
{
}

void Stm32MotorControlHW::SetPrechargeRelay(bool Close)
{
}

void Stm32MotorControlHW::ToggleLed()
{
    gpio_toggle(red_led);
}

void Stm32MotorControlHW::SetTrigger(enum TrigState stt)
{
   if (TRIGGER_RISE == stt)
   {
      gpio_set(blue_led);
   }
   else
   {
      gpio_clear(blue_led);
   }
}

/* IRQ interface */
void Stm32MotorControlHW::Isr()
{
   TIM_SR(PWM_TIMER) &= ~TIM_SR_UIF;

   if (0 != timerHandler)
      timerHandler->TimerInterrupt();
}

/* Helper functions */
void Stm32MotorControlHW::clock_setup(void)
{
    RCC_CLOCK_SETUP();

	rcc_set_ppre1(RCC_CFGR_PPRE1_HCLK_DIV2);

    /* Enable all present GPIOx clocks. (whats with GPIO F and G?)*/
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPDEN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPEEN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPGEN);

    /* Enable clock for USART1. */
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_USART1EN);
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USART2EN);

    /* Enable TIM1 clock */
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_TIM1EN);

    /* Enable TIM3 clock */
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM3EN);

    /* Enable TIM4 clock */
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM4EN);

    /* Enable TIM8 clock */
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_TIM8EN);

}

void Stm32MotorControlHW::gpio_setup(void)
{
    /* Set LEDs (in GPIO port C) to 'output push-pull'. */
    gpio_set_mode(red_led_port, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, red_led_pin);
    gpio_set_mode(blue_led_port, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, blue_led_pin);

    /* Timer 4 Ch 1,2,3 GPIO */
#if (HWCONFIG == HWCONFIG_SB5COM)
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO0 | GPIO1);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO6 | GPIO7 | GPIO8);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO7);
#endif
#if (HWCONFIG==HWCONFIG_OLIMEX)
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO13 | GPIO14 | GPIO15);
#endif

    /* DC switch */
    gpio_set_mode(dcsw_out_port, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, dcsw_out_pin);
}

void Stm32MotorControlHW::tim_setup(void)
{
   pwmdigits = 11;//(MIN_PWM_DIGITS + parm_Get(PARAM_pwmfrq));
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
   timer_set_oc_polarity_high(PWM_TIMER, TIM_OC1);
   timer_set_oc_polarity_high(PWM_TIMER, TIM_OC2);
   timer_set_oc_polarity_high(PWM_TIMER, TIM_OC3);

   /* Output enable */
   timer_enable_oc_output(PWM_TIMER, TIM_OC1);
   timer_enable_oc_output(PWM_TIMER, TIM_OC2);
   timer_enable_oc_output(PWM_TIMER, TIM_OC3);
   timer_enable_oc_output(PWM_TIMER, TIM_OC1N);
   timer_enable_oc_output(PWM_TIMER, TIM_OC2N);
   timer_enable_oc_output(PWM_TIMER, TIM_OC3N);

   timer_disable_break_automatic_output(PWM_TIMER);
   timer_set_break_polarity_low(PWM_TIMER);
   timer_set_enabled_off_state_in_run_mode(PWM_TIMER);
   timer_set_enabled_off_state_in_idle_mode(PWM_TIMER);

   timer_set_deadtime(PWM_TIMER, TIM_DEADTIME);

   timer_set_oc_idle_state_unset(PWM_TIMER, TIM_OC1);
   timer_set_oc_idle_state_unset(PWM_TIMER, TIM_OC2);
   timer_set_oc_idle_state_unset(PWM_TIMER, TIM_OC3);
   timer_set_oc_idle_state_unset(PWM_TIMER, TIM_OC1N);
   timer_set_oc_idle_state_unset(PWM_TIMER, TIM_OC2N);
   timer_set_oc_idle_state_unset(PWM_TIMER, TIM_OC3N);

   timer_generate_event(PWM_TIMER, TIM_EGR_UG);

   timer_enable_irq(PWM_TIMER, TIM_DIER_UIE);

   timer_set_prescaler(PWM_TIMER, 0);
   /* PWM frequency */
   timer_set_period(PWM_TIMER, PWM_MAX);
   timer_set_repetition_counter(PWM_TIMER, 1);

   /* start out with 50:50 duty cycle */
   timer_set_oc_value(PWM_TIMER, TIM_OC1, PWM_MAX / 2);
   timer_set_oc_value(PWM_TIMER, TIM_OC2, PWM_MAX / 2);
   timer_set_oc_value(PWM_TIMER, TIM_OC3, PWM_MAX / 2);

   timer_enable_counter(PWM_TIMER);

   //timer_enable_break_main_output(PWM_TIMER);

   nvic_enable_irq(PWM_TIMER_IRQ);
   nvic_set_priority(PWM_TIMER_IRQ, 0);

   /* setup capture timer */
   /* Filter input signal */
   TIM_SMCR(REV_CNT_TIMER) = TIM_SMCR_ECE | TIM_SMCR_ETP | TIM_SMCR_ETF_DTS_DIV_32_N_8;

   timer_enable_counter(REV_CNT_TIMER);
}
