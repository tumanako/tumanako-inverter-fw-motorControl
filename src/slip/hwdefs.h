#ifndef HWDEFS_H_INCLUDED
#define HWDEFS_H_INCLUDED

#define HWCONFIG_SB5COM 0
#define HWCONFIG_OLIMEX 1

#define HWCONFIG HWCONFIG_OLIMEX

#if (HWCONFIG == HWCONFIG_SB5COM)
#define RCC_CLOCK_SETUP rcc_clock_setup_in_hse_16mhz_out_72mhz
#define red_led_port  GPIOC
#define red_led_pin   GPIO12
#define blue_led_port GPIOD
#define blue_led_pin  GPIO6
#define	red_led		red_led_port, red_led_pin	/* PC12 */
#define	blue_led	blue_led_port, blue_led_pin	/* PD6 */

#define dcsw_out_port GPIOC
#define dcsw_out_pin  GPIO11
#define dcsw_out      dcsw_out_port, dcsw_out_pin /* PC11 */

#define dir_port      GPIOG
#define dir_bit       10
#define dir_pin       GPIO10 /* must match dir_bit */
#define dir_in        dir_port, dir_pin           /* PC10 */

#define PWM_TIMER     TIM8  /* TIM1 or TIM8 */
#define PWM_TIMER_IRQ NVIC_TIM8_UP_IRQ
#define pwm_timer_isr tim8_up_isr

#define REV_CNT_TIMER TIM1

#define TERM_USART USART1
#define TERM_USART_TXPIN GPIO_USART1_TX
#define USART_BAUDRATE  115200
#endif

#if (HWCONFIG==HWCONFIG_OLIMEX)
#define RCC_CLOCK_SETUP rcc_clock_setup_in_hse_8mhz_out_72mhz

#define red_led_port  GPIOC
#define red_led_pin   GPIO12
#define blue_led_port GPIOC
#define blue_led_pin  GPIO10
#define	red_led		red_led_port, red_led_pin	/* PC12 */
#define	blue_led	blue_led_port, blue_led_pin	/* PC10 */

#define dcsw_out_port GPIOC
#define dcsw_out_pin  GPIO11
#define dcsw_out      dcsw_out_port, dcsw_out_pin /* PC11 */

#define dir_port      GPIOC
#define dir_bit       9
#define dir_pin       GPIO9 /* must match dir_bit */
#define dir_in        dir_port, dir_pin           /* PC10 */

#define PWM_TIMER     TIM1  /* TIM1 or TIM8 */
#define PWM_TIMER_IRQ NVIC_TIM1_UP_IRQ
#define pwm_timer_isr tim1_up_isr

#define REV_CNT_TIMER TIM2

#define TERM_USART USART1
#define TERM_USART_TXPIN GPIO_USART1_TX
#define USART_BAUDRATE  115200//230400 //not actually because USART2 is on APB2 with half system clock
#endif

#define TIM_DEADTIME 28
#define PWM_MAX 2048

#endif // HWDEFS_H_INCLUDED
