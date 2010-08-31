stm32_sine
----------

1. Sine wave lookup
-------------------
stm32_sine.h contains a large define for the sine wave LUT. It has basically been
created using OpenOffice Calc.

The sine is scaled with 32767 and offset with 32767. Thus when a normal sine function is

-1	: LUT is 0
 0	: LUT is 32767
 1	: LUT is 65535

To address the table, we scale the signs argument so that 2xPi maps to 65535.
So, when the sine table has 256 entries, we devide the argument by 256 and
with the result do the lookup.

address = argument / 256

When using a u16 argument, it will wrap at 65535 and thus always stay within
the table boundaries.

We call the function SineLookup(Arg)

2. Space Vector modulation
--------------------------
Sine wave modulation can never use the full DC-bus voltage, because when one
of the three sine waves reaches its maximum, neither of the other two sine
waves reach their minimum. This wastes around 15% of the DC voltage.
To use the full voltage of the DC-bus, we allow amplitudes greater than 1,
namely amplitudes up to 1.15. Now, when one sine wave, say L1, reaches an
amplitude of 1, we start offsetting the virtual neutral potential in a way
that L1 never exceeds 1. Since we apply this offset to all three phases,
the phase-to-phase voltage stays the same.
It all comes down to the formular
Offset = 1/2 * (min{a,b,c} + max{a,b,c})
where a,b,c in [-1,1] represent the three phases.
In the source code it is explained how we apply this calculation to the
[0,65535] value range.

We call the function SVPWMOfs

3. Sine wave scaling
--------------------
to scale the result of our lookup, we use a simple integer mulitplication
We map zero amplitude to 0 and full amplitude to 32767. With values larger
than that we can generate a double humped or overmodulated SVM later on.

Scaling now works in 3 steps:

a) Multiply sine lookup value with amplitude with a u32 multiplication
b) Shift the result 15 (meaning division by 32767)
c) Shift the result to match the PWM resolution, e.g. by 16 if PWM resolution is 12 bits.

We call the Function MultAmp(amp, base)

3. Dutycycle calculation
------------------------
The 3 sine waves look like this (x=0..2Pi, a=0..1)

L1 = a * sin(x         )
L2 = a * sin(x +   Pi/3)
L3 = a * sin(x + 2xPi/3)

With SVPWM:
Ofs = SVPWMOfs(L1, L2, L3)

With the scaling introduced above this comes to (arg=0..65535, amp=0..37550)
Dutycycle L1 = MultAmp(amp, SineLookup( arg                        ) - Ofs)
Dutycycle L2 = MultAmp(amp, SineLookup((arg +     65535/3) & 0xFFFF) - Ofs)
Dutycycle L3 = MultAmp(amp, SineLookup((arg + 2 * 65535/3) & 0xFFFF) - Ofs)

The addition arg + 65535/3 is done in u32 and then masked to u16, because
you never know what the compiler does in case of overflow.

4. Frequency
------------
The frequency is increased by skipping over the lookup table faster. Thus, after
every calculation of the dutycycles we add a certain amount to the argument.
What frequency that comes down to depends on the PWM frequency.

In the program we add the result of the ADC (0..4095) offset by 2048 and 
divided by 8. That way we can skip over the sine wave table backward and
forward and thus spin the motor in both directions.

5. Timer ISR
------------
The timer interrupt handler calculates the new dutycycle as described above,
increases the argument and clears the interrupt pending flag

6. Timer setup
--------------
This is largely hardware dependend, since the timer outputs map to fixed pins.
The best choice would be TIM1 or TIM8 since the allow for complementary
outputs with dead time generation. Unfortunatly on my hardware the outputs
are connected to other peripherals and I couldn't start the timer for
unknown reasons. Thus, I'm using TIM4 with OCC channels 1-3

Timer setup comes down to the following steps

a) Enable the APB clock, TIM4 is on APB1
b) Configure pins. TIM4 Ch1-3 is on GPIOB Pins PA6-PA8
c) Configure interrupt controller (nvic_enable_irq and nvic_set_priority)
d) Enable center aligned mode 1 (CMS_CENTER_1) and ARPE in TIMx_CR1
e) Set PWM mode 1 (OCxM_PWM1) and Preload enable (OCyPE) for channel 1 and 2 in TIMx_CCMR1
f) Set PWM mode 1 (OCxM_PWM1) and Preload enable (OCyPE) for channel 3 TIMx_CCMR2
g) enable the output for channels 1-3 with CCyE in TIMx_CCER
h) enable the update generation flag UG in TIMx_EGR
i) Enable the update interrupt UIE in TIMx_DIER
j) Set the prescaler TIMx_PSC (haven't yet understood, 1 looked allright)
k) set the maximum PWM value and thus the frequency in TIMx_ARR
l) Enable the timer with CEN in TIMx_CR1
