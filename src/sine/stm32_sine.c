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

#include <libopenstm32/rcc.h>
#include <libopenstm32/gpio.h>
#include <libopenstm32/usart.h>
#include <libopenstm32/adc.h>
#include <libopenstm32/timer.h>
#include <libopenstm32/nvic.h>
#include <libopenstm32/scb.h>
#include "stm32_sine.h"
#include "stm32_timsched.h"

static const u16 SinTab[] = { SINTAB };/* sine LUT */
static       u16 analog_data;  			/* used to set inverter frequency */
			 u16 DutyCycle[3]; 			/* global for debugging */

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

#define	red_led		GPIO12	/* PC12 */
#define	blue_led	GPIO6	/* PD6 */

#define max(a,b,c) (a>b && a>c)?a:(b>a && b>c)?b:c

#define min(a,b,c) (a<b && a<c)?a:(b<a && b<c)?b:c

/* TODO: add defines for timers used to make this program more generic */

void output_digit(int num);

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

/* Calculate dutycycles */
void tim4_isr(void)
{
	static   u16 Arg = 0;
             u16 Amp = 37550; /* Full amplitude for now, 15% extra with SVPWM */
             s16 Ofs;
             u8  Idx;
    volatile u32 *pTimCcr = &TIM4_CCR1;

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

    /* Clear interrupt pending flag */
    TIM4_SR &= ~TIM_SR_UIF;
    /* Increase sine arg */
    /* midval means 0 Hz */
    /* values below midval makes us run through the sine lookup backwards
       -> motor spins the other direction */
    Arg += ((s16)analog_data - 2048) >> 2;
    /* "lifesign" */
	gpio_toggle(GPIOD, blue_led);	/* LED on/off */
}

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

	/* Enable TIM3 clock */
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM3EN);

	/* Enable TIM4 clock */
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM4EN);
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

void gpio_setup(void)
{
	/* Set LEDs (in GPIO port C) to 'output push-pull'. */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, red_led);
	gpio_set_mode(GPIOD, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, blue_led);
    /* Timer 4 Ch 1,2,3 GPIO */
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_TIM4_CH1 | GPIO_TIM4_CH2 | GPIO_TIM4_CH3);

}

void output_digit(int num)
{
	if(num > 1000)	usart_send(USART1, num/1000 + '0');
	else usart_send(USART1, ' ');
	num -= ((u8) (num/1000)) * 1000;
	usart_send(USART1, num/100 + '0');
	num -= ((u8) (num/100))  * 100;
	usart_send(USART1, num/10 + '0');
	num -= ((u8) (num/10))   * 10;
	usart_send(USART1, num + '0');
}

/* Enable timer interrupts */
void nvic_setup(void)
{
    nvic_enable_irq(NVIC_TIM4_IRQ);
    nvic_set_priority(NVIC_TIM4_IRQ, 0);
}

void tim_setup(void)
{
    /* Center aligned PWM */
    TIM4_CR1 = TIM_CR1_CMS_CENTER_1 | TIM_CR1_ARPE;
    /* PWM mode 1 and preload enable */
    TIM4_CCMR1 = TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE | TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE;
    TIM4_CCMR2 = TIM_CCMR2_OC3M_PWM1 | TIM_CCMR2_OC3PE;
    /* Output enable */
    TIM4_CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
    /* Enable update generation */
    TIM4_EGR = TIM_EGR_UG;
    /* Enable update event interrupt */
    TIM4_DIER = TIM_DIER_UIE;
    /* Prescaler */
    TIM4_PSC = 0x1;
    /* PWM frequency */
    TIM4_ARR = PWM_MAX;
    /* start out with 50:50 duty cycle */
    TIM4_CCR1 = PWM_MAX / 2;
    TIM4_CCR2 = PWM_MAX / 2;
    TIM4_CCR3 = PWM_MAX / 2;
    /* Enable timer */
    TIM4_CR1 |= TIM_CR1_CEN;
}

void adc_setup(void)
{
	int i;
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);

	/* make shure it didnt run during config */
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
void ToggleLed(void)
{
	gpio_toggle(GPIOC, red_led);	/* LED on/off */
}

void ReadAdc(void)
{
	adc_on(ADC1);	/* If the ADC_CR2_ON bit is already set -> setting it another time starts the conversion */
	while (!(ADC_SR(ADC1) & ADC_SR_EOC));	/* Waiting for end of conversion */
	analog_data = ADC_DR(ADC1) & 0xFFF;		/* read adc data */
}

int main(void)
{
    static u8 channel_array[16] = {16,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};

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

	create_task(ReadAdc,   PRIO_GRP1, 0, 250);
	create_task(ToggleLed, PRIO_GRP2, 0, 500);

	while (1) {
   		while (!(USART1_SR & USART_SR_RXNE));
        usart_recv(USART1);

        output_digit(DutyCycle[0]);
        usart_send(USART1, ' ');
        output_digit(DutyCycle[1]);
        usart_send(USART1, ' ');
        output_digit(DutyCycle[2]);
        usart_send(USART1, ' ');
        output_digit(analog_data);
		usart_send(USART1, '\r');
	}
	return 0;
}




