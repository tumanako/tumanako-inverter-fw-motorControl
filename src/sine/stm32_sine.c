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
#include <STM32MCU.h>
#include <stm32_sine.h>

static const u16 SinTab[] = { SINTAB };/* sine LUT */
static       u16 analog_data;  			/* used to set inverter frequency */
			 u16 DutyCycle[3]; 			/* global for debugging */

#define SINTAB_ARGDIGITS 8
#define SINTAB_ENTRIES  (1 << SINTAB_ARGDIGITS)
#define SINTAB_DIGITS    16
#define PWM_DIGITS       12
#define PWM_MAX         (1 << PWM_DIGITS)
#define PHASE_SHIFT120  ((u32)(65535 / 3))
#define PHASE_SHIFT240  ((u32)(2 * (65535 / 3)))

/* TODO: add defines for timers used to make this program more generic */

void output_digit(int num);

/* Performs a lookup in the sine table */
/* 0 = 0, 2Pi = 65535 */
u16 SineLookup(u16 Arg)
{
    /* No interpolation for now */
    /* We divide arg by 2^(SINTAB_ARGDIGITS) */
    /* No we can directly address the lookup table */
    Arg >>= SINTAB_DIGITS - SINTAB_ARGDIGITS;
    return SinTab[Arg];
}

/* 0 = 0, 1 = 32767 */
u16 MultiplyAmplitude(u16 Amplitude, u16 Baseval)
{
    u32 Temp;
    Temp = (u32)Amplitude * (u32)Baseval;
    /* Divide by 32768 */
    /* -> Allow overmodulation, maybe useful? */
    Temp >>= (SINTAB_DIGITS - 1);
    /* Match to PWM resolution */
    return Temp >> (SINTAB_DIGITS - PWM_DIGITS);
}

/* Toggle lifesign LED and read ADC */
void tim3_isr(void)
{
    /* Clear interrupt pending flag */
    TIM3_SR &= ~TIM_SR_CC1IF;
	gpio_toggle(GPIOC, red_led);	/* LED on/off */

	adc_on(ADC1);	/* If the ADC_CR2_ON bit is already set -> setting it another time starts the conversion */
	while (!(ADC_SR(ADC1) & ADC_SR_EOC));	/* Waiting for end of conversion */
	analog_data = ADC_DR(ADC1) & 0xFFF;		/* read adc data */
}

/* Calculate dutycycles */
void tim4_isr(void)
{
	static u16 Arg = 0;
           u16 Amp = 32767; /* Full amplitude for now */


    DutyCycle[0] = SineLookup(Arg);
    DutyCycle[0] = MultiplyAmplitude(Amp, DutyCycle[0]);
    /* The CCR must never be zero, otherwise no interrupts occur anymore */
	/* TODO: check this for UIF */
    TIM4_CCR1 = DutyCycle[0] + 1;
    DutyCycle[1] = SineLookup((u16)(((u32)Arg + PHASE_SHIFT120) & 0xFFFF));
    DutyCycle[1] = MultiplyAmplitude(Amp, DutyCycle[1]);
    TIM4_CCR2 = DutyCycle[1] + 1;
    DutyCycle[2] = SineLookup((u16)(((u32)Arg + PHASE_SHIFT240) & 0xFFFF));
    DutyCycle[2] = MultiplyAmplitude(Amp, DutyCycle[2]);
    TIM4_CCR3 = DutyCycle[2] + 1;

    /* Clear interrupt pending flag */
    TIM4_SR &= ~TIM_SR_UIF;
    /* Increase sine arg */
    Arg += analog_data >> 4;
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
    nvic_enable_irq(NVIC_TIM3_IRQ);
    nvic_enable_irq(NVIC_TIM4_IRQ);
    nvic_set_priority(NVIC_TIM3_IRQ, 1);
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

    /* Set up blinking Led timer */
    TIM3_CR1 = TIM_CR1_DIR_DOWN | TIM_CR1_ARPE;
    TIM3_CCMR1 = TIM_CCMR1_OC1M_ACTIVE | TIM_CCMR1_OC1PE;
    TIM3_EGR = TIM_EGR_UG;
    TIM3_PSC = 0x4000;
    TIM3_ARR = 500;
    TIM3_CCR1 = 0;
    TIM3_DIER = TIM_DIER_CC1IE;
    TIM3_CR1 |=  TIM_CR1_CEN;
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


int main(void)
{
    static u8 channel_array[16] = {16,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};

	clock_setup();
	gpio_setup();
	usart_setup();
	adc_setup();		/* todo: check setup of analog peripheral*/
	nvic_setup();
	tim_setup();

	gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO3);
    channel_array[0] = adcchfromport(2,3);
	adc_set_regular_sequence(ADC1, 1, channel_array);

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




