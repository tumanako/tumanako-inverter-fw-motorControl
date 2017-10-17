#ifndef HWDEFS_H_INCLUDED
#define HWDEFS_H_INCLUDED


//Common for any config

//Maximum value for over current limit timer
#define OCURMAX 4096
#define USART_BAUDRATE  115200
//Maximum PWM frequency is 36MHz/2^MIN_PWM_DIGITS
#define MIN_PWM_DIGITS 11
#define PERIPH_CLK      ((uint32_t)36000000)

#define RCC_CLOCK_SETUP rcc_clock_setup_in_hse_8mhz_out_72mhz

#define PWM_TIMER     TIM1
#define PWM_TIMER_IRQ NVIC_TIM1_UP_IRQ
#define pwm_timer_isr tim1_up_isr

#define REV_CNT_RCC_ENR    RCC_APB1ENR_TIM3EN
#define rev_timer_isr      tim3_isr
#define REV_CNT_TIMER      TIM3
#define OVER_CUR_TIMER TIM4

#define TERM_USART USART3
#define TERM_USART_TXPIN GPIO_USART3_TX
#define TERM_USART_TXPORT GPIOB
//Address of parameter block in flash
#define PARAM_ADDRESS 0x0801FC00
#define PARAM_BLKSIZE 1024
#define CANMAP_ADDRESS 0x0801F800

#ifdef HWCONFIG_REV1
#define REV_CNT_IC         TIM_IC3
#define REV_CNT_CCR        TIM3_CCR3
#define REV_CNT_CCER       TIM_CCER_CC3P
#define REV_CNT_SR         TIM_SR_CC3IF
#define REV_CNT_DMAEN      TIM_DIER_CC3DE
#define REV_CNT_DMACHAN    2
#define REV_CNT_DMA_CNDTR  DMA1_CNDTR2
#endif
#if defined(HWCONFIG_REV2) || defined(HWCONFIG_TESLA)
#define REV_CNT_IC         TIM_IC1
#define REV_CNT_CCR        TIM3_CCR1
#define REV_CNT_CCER       TIM_CCER_CC1P
#define REV_CNT_SR         TIM_SR_CC1IF
#define REV_CNT_DMAEN      TIM_DIER_CC1DE
#define REV_CNT_DMACHAN    6
#define REV_CNT_DMA_CNDTR  DMA1_CNDTR6
#endif

#endif // HWDEFS_H_INCLUDED
