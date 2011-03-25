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
 * Example from libopenstm32 expanded to make tumanako arm tester
 * Added sine wave generation, removed testing code
 */

//#define TUMANAKO_KIWIAC  //KiwiAC uses TIM1 alternate function (comment out for TIM4)
#define USE_THROTTLE_POT 1

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/nvic.h>
#include <libopencm3/stm32/scb.h>

#include <stdio.h>

#include "stm32_sine.h"
#include "stm32_timsched.h"

static const u16 SinTab[] = { SINTAB };/* sine LUT */
static       u16 analog_data;  			/* used to set inverter frequency */
             u16 DutyCycle[3]; 			/* global for debugging */
static       u16 CtrVal;
static       u8  mode;
static       s16 frq = 650;
static       s16 slip;

/** @todo calculate this instead of using a constant */
#define PERIPH_CLK      ((u32)36000000)

#define SINTAB_ARGDIGITS 8
#define SINTAB_ENTRIES  (1 << SINTAB_ARGDIGITS)
/* Value range of sine lookup table */
#define SINTAB_DIGITS    16
#define SINTAB_MAX      (1 << SINTAB_DIGITS)
/* Domain of lookup function */
#define SINLU_ARGDIGITS  16
#define SINLU_ONEREV    (1 << SINLU_ARGDIGITS)
#define PWM_DIGITS       12
#define PWM_MAX         (1 << PWM_DIGITS)
#define PHASE_SHIFT120  ((u32)(     SINLU_ONEREV / 3))
#define PHASE_SHIFT240  ((u32)(2 * (SINLU_ONEREV / 3)))

#define PWM_FREQ           (PERIPH_CLK / (u32)PWM_MAX)
#define HZ_PER_DIGIT_8    ((256 * PWM_FREQ) / SINLU_ONEREV)
#define DIGIT_TO_N_8       (HZ_PER_DIGIT_8 / POLE_PAIRS)
#define NUM_IMPULSE_PER_REV 32
#define POLE_PAIRS           2
#define SAMPLE_INTERVAL    100
#define RPM_FACT           (6000/NUM_IMPULSE_PER_REV * 1000 / SAMPLE_INTERVAL)
#define HZ_FACT            (POLE_PAIRS * RPM_FACT/6)

#define MOD_SLIP 1
#define MOD_CTRL 2

#ifdef TUMANAKO_KIWIAC  //i.e. TIM1 alternate function

#define	red_led_port  GPIOC
#define	red_led_bit   GPIO6
#define	blue_led_port GPIOC
#define	blue_led_bit  GPIO7

#define TUMANAKO_PWM_TIM_CCR1 TIM1_CCR1
#define TUMANAKO_PWM_TIM_CCR2 TIM1_CCR2
#define TUMANAKO_PWM_TIM_CCR3 TIM1_CCR3

#define TUMANAKO_TIMX_SR TIM1_SR

#define	TUMANAKO_PWM_RCC_APBXENR RCC_APB2ENR
#define	TUMANAKO_PWM_RCC_APBXENR_TIMXEN (RCC_APB2ENR_TIM1EN | RCC_APB2ENR_AFIOEN)

#define TUMANAKO_PWM_TIM_PORT GPIOE
//#define TUMANAKO_PWM_TIM_CHANNELS (GPIO_TIM1_CH1_REMAP | GPIO_TIM1_CH2_REMAP | GPIO_TIM1_CH3_REMAP | GPIO_TIM1_CH1N_REMAP | GPIO_TIM1_CH2N_REMAP | GPIO_TIM1_CH3N_REMAP)
#define TUMANAKO_PWM_TIM_CHANNELS (GPIO9 | GPIO11 | GPIO13 | GPIO8 | GPIO10 | GPIO12)

#define TUMANAKO_NVIC_PWM_IRQ NVIC_TIM1_UP_IRQ

#define TUMANAKO_PWM_TIM_EGR TIM1_EGR
#define TUMANAKO_PWM_TIM_DIER TIM1_DIER

#define TUMANAKO_PWM_CR1 TIM1_CR1
#define TUMANAKO_PWM_CCMR1 TIM1_CCMR1
#define TUMANAKO_PWM_CCMR2 TIM1_CCMR2
#define TUMANAKO_PWM_CCER TIM1_CCER

#define TUMANAKO_PWM_PSC TIM1_PSC
#define TUMANAKO_PWM_ARR TIM1_ARR

#define TUMANAKO_ROT_TIM_SCMR //define speed sensor timer here
#define TUMANAKO_ROT_TIM_CR1 
#define TUMANAKO_ROT_TIM_CNT

#else  //TIM4 (e.g. olimex board)

#define	red_led_port  GPIOC
#define	red_led_bit   GPIO12
#define	blue_led_port GPIOD
#define	blue_led_bit  GPIO6

#define TUMANAKO_PWM_TIM_CCR1           TIM4_CCR1
#define TUMANAKO_PWM_TIM_CCR2           TIM4_CCR2
#define TUMANAKO_PWM_TIM_CCR3           TIM4_CCR3

#define TUMANAKO_TIMX_SR                TIM4_SR

#define	TUMANAKO_PWM_RCC_APBXENR        RCC_APB1ENR
#define	TUMANAKO_PWM_RCC_APBXENR_TIMXEN RCC_APB1ENR_TIM4EN

#define TUMANAKO_PWM_TIM_PORT           GPIOB
#define TUMANAKO_PWM_TIM_CHANNELS       (GPIO_TIM4_CH1 | GPIO_TIM4_CH2 | GPIO_TIM4_CH3) 

#define TUMANAKO_NVIC_PWM_IRQ           NVIC_TIM4_IRQ

#define TUMANAKO_PWM_TIM_EGR            TIM4_EGR
#define TUMANAKO_PWM_TIM_DIER           TIM4_DIER

#define TUMANAKO_PWM_CR1                TIM4_CR1
#define TUMANAKO_PWM_CCMR1              TIM4_CCMR1
#define TUMANAKO_PWM_CCMR2              TIM4_CCMR2
#define TUMANAKO_PWM_CCER               TIM4_CCER

#define TUMANAKO_PWM_PSC                TIM4_PSC
#define TUMANAKO_PWM_ARR                TIM4_ARR

#define TUMANAKO_ROT_TIM_SCMR           TIM1_SMCR
#define TUMANAKO_ROT_TIM_CR1            TIM1_CR1
#define TUMANAKO_ROT_TIM_CNT            TIM1_CNT

#endif

#define max(a,b,c) (a>b && a>c)?a:(b>a && b>c)?b:c

#define min(a,b,c) (a<b && a<c)?a:(b<a && b<c)?b:c

/* Performs a lookup in the sine table */
/* 0 = 0, 2Pi = 65535 */
u16 SineLookup(u16 Arg)
{
    /* No interpolation for now */
    /* We divide arg by 2^(SINTAB_ARGDIGITS) */
    /* No we can directly address the lookup table */
    Arg >>= SINLU_ARGDIGITS - SINTAB_ARGDIGITS;
    return SinTab[Arg];
}

/* 0 = 0, 1 = 32767 */
u16 MultiplyAmplitude(u16 Amplitude, u16 Baseval)
{
    u32 Temp;
    Temp = (u32)Amplitude * (u32)Baseval;
    /* Divide by 32768 */
    /* -> Allow overmodulation, for SVPWM or FTPWM */
    Temp >>= (SINTAB_DIGITS - 1);
    /* Match to PWM resolution */
    Temp >>= (SINTAB_DIGITS - PWM_DIGITS);
    return Temp;
}

s16 CalcSVPWMOffset(u16 a, u16 b, u16 c)
{
    /* Formular for svpwm:
       Offset = 1/2 * (min{a,b,c} + max{a,b,c}) */
    /* this is valid only for a,b,c in [-32768,32767] */
    /* we calculate from [0, 65535], thus we need to subtract 32768 to be symmetric */
    s16 acor = a - (SINTAB_MAX >> 1);
    s16 bcor = b - (SINTAB_MAX >> 1);
    s16 ccor = c - (SINTAB_MAX >> 1);

    s16 Minimum = min(acor, bcor, ccor);
    s16 Maximum = max(acor, bcor, ccor);
    s16 Offset = Minimum + Maximum;

    return Offset >> 1;
}

void tim1_brk_isr(void)
{
    gpio_set(blue_led_port, blue_led_bit); /* BLUE LED on */
    gpio_set(red_led_port, red_led_bit); /* RED LED on */
}

/* Calculate dutycycles */
#ifdef TUMANAKO_KIWIAC
void tim1_up_isr(void)
#else
void tim4_isr(void)
#endif
{
    static   u8  IntCnt = 0;
    static   u16 Arg = 0;
             u16 Amp = 37550; /* Full amplitude for now, 15% extra with SVPWM */
             s16 Ofs;
             u8  Idx;
    volatile u32 *pTimCcr = &TUMANAKO_PWM_TIM_CCR1;

    IntCnt++;

    /* Clear interrupt pending flag - do this at the start of the routine */
    TUMANAKO_TIMX_SR &= ~TIM_SR_UIF;

    /* In center aligned mode, this ISR is called twice:
       - in the middle of the period
       - at the end of the period
       we only want to calculate in the middle */
    if (IntCnt & 1)
    {
        /* 1. Calculate sine */
        DutyCycle[0] = SineLookup(Arg);
        DutyCycle[1] = SineLookup((u16)(((u32)Arg + PHASE_SHIFT120) & 0xFFFF));
        DutyCycle[2] = SineLookup((u16)(((u32)Arg + PHASE_SHIFT240) & 0xFFFF));

        /* 2. Calculate the offset of SVPWM */
        Ofs = CalcSVPWMOffset(DutyCycle[0], DutyCycle[1], DutyCycle[2]);

        for (Idx = 0; Idx < 3; Idx++, pTimCcr++)
        {
            /* 3. subtract it from all 3 phases -> no difference in phase-to-phase voltage */
            DutyCycle[Idx] -= Ofs;
            /* 4. Set desired amplitude and match to PWM resolution */
            DutyCycle[Idx] = MultiplyAmplitude(Amp, DutyCycle[Idx]);
            /* 5. Write to compare registers */
            *pTimCcr = DutyCycle[Idx];
        }

        /* Increase sine arg */
        /* values below 0 makes us run through the sine lookup backwards
           -> motor spins the other direction */
        Arg += frq;
        /* "lifesign" */
        gpio_toggle(blue_led_port, blue_led_bit);	/* LED on/off */
    } /* end if */
} /* end isr */

void clock_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	/* Enable all present GPIOx clocks. (whats with GPIO F and G?)*/
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPDEN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPEEN);

	/* Enable clock for USART1. */
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_USART1EN);

	/* Enable TIM1 clock */
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_TIM1EN);

	/* Enable TIM3 clock */
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM3EN);

	/* Enable PWM TIM clock (TIM1 or TIM4 depending on KiwiAC or not defined above) */
	rcc_peripheral_enable_clock(&TUMANAKO_PWM_RCC_APBXENR, TUMANAKO_PWM_RCC_APBXENR_TIMXEN);
}

void usart_setup(void)
{
	/* Setup GPIO pin GPIO_USART3_TX/GPIO10 on GPIO port B for transmit. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

	/* Setup UART parameters. */
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART1);
}

/* UART1 only write syscall, untimately called by printf()  */
int _write(int fd, const u8 *buf, int len)
{
	int i;

	(void)fd;

	for(i = 0; i < len; i++) 
		usart_send(USART1, buf[i]);

	return len;
}

void gpio_setup(void)
{
#ifdef TUMANAKO_KIWIAC
  AFIO_MAPR |= AFIO_MAPR_TIM1_REMAP_FULL_REMAP;
#endif

	/* Set LEDs (in GPIO port C) to 'output push-pull'. */
	gpio_set_mode(red_led_port, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, red_led_bit);
	gpio_set_mode(blue_led_port, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, blue_led_bit);
    /* Timer Ch GPIO */
    gpio_set_mode(TUMANAKO_PWM_TIM_PORT, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, TUMANAKO_PWM_TIM_CHANNELS);

}

/* Enable timer interrupts */
void nvic_setup(void)
{
    nvic_enable_irq(TUMANAKO_NVIC_PWM_IRQ);
    nvic_set_priority(TUMANAKO_NVIC_PWM_IRQ, 0);
}

void tim_setup(void)
{
    /* Center aligned PWM - control register 1 */
    TUMANAKO_PWM_CR1 = TIM_CR1_CMS_CENTER_1 | TIM_CR1_ARPE;
    /* PWM mode 1 and preload enable - capture compare mode register 1 and 2 */
    TUMANAKO_PWM_CCMR1 = TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE | TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE;
    TUMANAKO_PWM_CCMR2 = TIM_CCMR2_OC3M_PWM1 | TIM_CCMR2_OC3PE;
    /* Output enable */
#ifdef TUMANAKO_KIWIAC

    //clear CC1P (capture compare enable register, active high)
    TIM1_CCER &= (uint16_t)~TIM_CCER_CC1P;

    TUMANAKO_PWM_CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE;
    //Deadtime = 28 ~ 800ns
    TIM1_BDTR = (TIM_BDTR_OSSR | TIM_BDTR_OSSI | 28 ) & ~TIM_BDTR_BKP & ~TIM_BDTR_AOE;

    TIM1_CR2 &= (uint16_t)~(TIM_CR2_OIS1 || TIM_CR2_OIS1N || TIM_CR2_OIS2 || TIM_CR2_OIS2N || TIM_CR2_OIS3 || TIM_CR2_OIS3N);

#else
    TUMANAKO_PWM_CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
#endif
    /* Enable update generation */
    TUMANAKO_PWM_TIM_EGR = TIM_EGR_UG;
    /* Enable update event interrupt */
    TUMANAKO_PWM_TIM_DIER = TIM_DIER_UIE;
    /* Prescaler */
    TUMANAKO_PWM_PSC = 0x1;
    /* PWM frequency */
    TUMANAKO_PWM_ARR = PWM_MAX; //4096 

    TIM1_RCR = 1;  //set rep_rate

    /* start out with 50:50 duty cycle (does NOT handle rolling starts!!!)*/
    TUMANAKO_PWM_TIM_CCR1 = PWM_MAX / 2;
    TUMANAKO_PWM_TIM_CCR2 = PWM_MAX / 2;
    TUMANAKO_PWM_TIM_CCR3 = PWM_MAX / 2;
    /* Enable timer */
    TUMANAKO_PWM_CR1 |= TIM_CR1_CEN;

#ifdef TUMANAKO_KIWIAC
    TIM1_BDTR |= TIM_BDTR_MOE; //enable TIM1 main outputs
#endif


    /* setup capture timer. Strong filtering for EMI "robustness" */
    TUMANAKO_ROT_TIM_SCMR = TIM_SMCR_ECE | TIM_SMCR_ETP | TIM_SMCR_ETF_DTS_DIV_16_N_8;
    /* setup capture timer */
    TUMANAKO_ROT_TIM_CR1  = TIM_CR1_CEN;
}

void adc_setup(void)
{
	int i;
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);

	/* make sure it didnt run during config */
	adc_off(ADC1);

	/* we configure everything for one single conversion */
	adc_disable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);
	adc_enable_discontinous_mode_regular(ADC1);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);
	/* we want read out the temperature sensor so we have to enable it */
	adc_enable_temperature_sensor(ADC1);
	adc_set_conversion_time_on_all_channels(ADC1, ADC_SMPR_SMP_28DOT5CYC);

	adc_on(ADC1);
	/* wait for adc starting up*/
        for (i = 0; i < 80000; i++);    /* Wait (needs -O0 CFLAGS). */

	adc_reset_calibration(ADC1);
	adc_calibration(ADC1);
}

u8 adcchfromport(int command_port, int command_bit)
{
	/*
	 PA0 ADC12_IN0
	 PA1 ADC12_IN1
	 PA2 ADC12_IN2
	 PA3 ADC12_IN3
	 PA4 ADC12_IN4
	 PA5 ADC12_IN5
	 PA6 ADC12_IN6
	 PA7 ADC12_IN7
	 PB0 ADC12_IN8
	 PB1 ADC12_IN9
	 PC0 ADC12_IN10
	 PC1 ADC12_IN11
	 PC2 ADC12_IN12
	 PC3 ADC12_IN13
	 PC4 ADC12_IN14
	 PC5 ADC12_IN15
	 temp ADC12_IN16
	 */
	switch (command_port) {
		case 0: /* port A */
			if (command_bit<8) return command_bit;
			break;
		case 1: /* port B */
			if (command_bit<2) return command_bit+8;
			break;
		case 2: /* port C */
			if (command_bit<6) return command_bit+10;
			break;
	}
	return 16;
}

/* Toggle lifesign LED and read ADC */
void ToggleRedLed(void)
{
    static u16 LastCtr = 0;
#if USE_THROTTLE_POT
    s16 slip_spnt = 12000 - analog_data;
#else
    s16 slip_spnt = 9500;
#endif

	gpio_toggle(red_led_port, red_led_bit);	/* LED on/off */
	CtrVal = TUMANAKO_ROT_TIM_CNT - LastCtr;
	LastCtr = TUMANAKO_ROT_TIM_CNT;

    /** @todo: match task time with constant here */
	slip = POLE_PAIRS * 10000 * CtrVal / ((NUM_IMPULSE_PER_REV * HZ_PER_DIGIT_8 * frq) >> 8);

	if (MOD_SLIP == mode)
	{
	    frq = (POLE_PAIRS * 100000 * CtrVal) / ((NUM_IMPULSE_PER_REV * HZ_PER_DIGIT_8 * slip_spnt) >> 8);
	}
}

void ReadAdc(void)
{
	adc_on(ADC1);	/* If the ADC_CR2_ON bit is already set -> setting it another time starts the conversion */
	while (!(ADC_SR(ADC1) & ADC_SR_EOC));	/* Waiting for end of conversion */
	analog_data = ADC_DR(ADC1) & 0xFFF;		/* read adc data */

#if USE_THROTTLE_POT
	if (MOD_CTRL == mode)
	{
	    frq = (analog_data - 2048) >> 1;
	}
#endif
}

int main(void)
{
    static u8 channel_array[16] = {16,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};
    char c;

	clock_setup();
	gpio_setup();
	usart_setup();
	adc_setup();		/* todo: check setup of analog peripheral*/
	nvic_setup();
	tim_setup();
	init_timsched();

	gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO3);
    channel_array[0] = adcchfromport(2,3);
	adc_set_regular_sequence(ADC1, 1, channel_array);

    mode = MOD_CTRL;
	
    create_task(ReadAdc,   PRIO_GRP1, 0, 250);
	/* The timing is still messed up, this gives me 100 ms */
	create_task(ToggleRedLed, PRIO_GRP1, 1, 200);

	while (1) {
   		while (!(USART1_SR & USART_SR_RXNE));
        c = usart_recv(USART1);

        if ('c' == c) mode = MOD_CTRL;
        if ('s' == c) mode = MOD_SLIP;

        printf("%d %d %d %d %d %d %d - ", DutyCycle[0], DutyCycle[1],
          DutyCycle[2], analog_data, slip, (int)(frq * HZ_PER_DIGIT_8) >> 8, /* inverter frequency */
         (int)(POLE_PAIRS * 10 * CtrVal)/NUM_IMPULSE_PER_REV); /* rotor frequency */

        //output various register data (useful for debug of timer setup and behaviour)
       /* Do we really print bitfields in decimal format? */
        printf("%lu %lu %lu %lu  %lu %lu %lu %lu  %lu %lu %lu %lu  "
           "%lu %lu %lu %lu  %lu %lu %lu %lu\r",
           TIM1_CR1, TIM1_CR2, TIM1_SMCR, TIM1_DIER,
           TIM1_SR, TIM1_EGR, TIM1_CCMR1, TIM1_CCMR2,
           TIM1_CCER, TIM1_CNT, TIM1_PSC, TIM1_ARR,
           TIM1_RCR, TIM1_CCR1, TIM1_CCR2, TIM1_CCR3,
           TIM1_CCR4, TIM1_BDTR, TIM1_DCR, TIM1_DMAR);
        fflush(stdout);

	}
	return 0;
}




