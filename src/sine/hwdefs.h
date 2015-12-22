#ifndef HWDEFS_H_INCLUDED
#define HWDEFS_H_INCLUDED

#define HWCONFIG_TUMANAKO_KIWIAC 5
#define HWCONFIG_OLIMEX 6

#define HWCONFIG HWCONFIG_OLIMEX

#if (HWCONFIG == HWCONFIG_TUMANAKO_KIWIAC)
#define RCC_CLOCK_SETUP rcc_clock_setup_in_hse_16mhz_out_72mhz

#define PWM_TIMER     TIM1
#define PWM_TIMER_IRQ NVIC_TIM1_UP_IRQ
#define pwm_timer_isr tim1_up_isr

#define REV_CNT_TIMER      TIM?
#define REV_CNT_IRQ        NVIC_TIM?_IRQ
#define REV_CNT_CCR        TIM1_CCR?
#define REV_CNT_CCMR2      TIM_CCMR2_CC4S_IN_TI2 | TIM_CCMR2_IC4F_DTF_DIV_32_N_6
#define REV_CNT_CCER       TIM_CCER_CC4E
#define REV_CNT_DMAEN      TIM_DIER_CC4DE
#define REV_CNT_DMA_CPAR   DMA1_CPAR4
#define REV_CNT_DMA_CMAR   DMA1_CMAR4
#define REV_CNT_DMA_CNDTR  DMA1_CNDTR4
#define REV_CNT_DMA_CCR    DMA1_CCR4
#define REV_CNT_RCC_ENR    RCC_APB1ENR_TIM3EN
#define rev_timer_isr  tim3_isr

#define OVER_CUR_TIMER TIM4

#define TERM_USART USART1
#define TERM_USART_TXPIN GPIO_USART1_TX
#define TERM_USART_TXPORT GPIOA
#endif

#if (HWCONFIG==HWCONFIG_OLIMEX)
#define RCC_CLOCK_SETUP rcc_clock_setup_in_hse_8mhz_out_72mhz

#define PWM_TIMER     TIM1
#define PWM_TIMER_IRQ NVIC_TIM1_UP_IRQ
#define pwm_timer_isr tim1_up_isr

#define REV_CNT_TIMER      TIM3
#define REV_CNT_IC         TIM_IC3
#define REV_CNT_IRQ        NVIC_TIM3_IRQ
#define REV_CNT_CCR        TIM3_CCR3
#define REV_CNT_CCMR2      TIM_CCMR2_CC3S_IN_TI2
#define REV_CNT_CCER       TIM_CCER_CC3E | TIM_CCER_CC3P
#define REV_CNT_DMAEN      TIM_DIER_CC3DE
#define REV_CNT_DMA_CPAR   DMA1_CPAR2
#define REV_CNT_DMA_CMAR   DMA1_CMAR2
#define REV_CNT_DMA_CNDTR  DMA1_CNDTR2
#define REV_CNT_DMA_CCR    DMA1_CCR2
#define REV_CNT_RCC_ENR    RCC_APB1ENR_TIM3EN
#define rev_timer_isr      tim3_isr

#define OVER_CUR_TIMER TIM4

#define TERM_USART USART3
#define TERM_USART_TXPIN GPIO_USART3_TX
#define TERM_USART_TXPORT GPIOB
//Address of parameter block in flash
#define PARAM_ADDRESS 0x0801FC00
#define PARAM_BLKSIZE 1024
#endif

//Maximum value for over current limit timer
#define OCURMAX 4096

#define USART_BAUDRATE  115200
//Maximum PWM frequency is 36MHz/2^MIN_PWM_DIGITS
#define MIN_PWM_DIGITS 11

#define PERIPH_CLK      ((uint32_t)36000000)


#endif // HWDEFS_H_INCLUDED
