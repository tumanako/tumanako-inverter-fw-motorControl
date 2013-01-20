/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2012 Johannes Huebner <contact@johanneshuebner.com>
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
 /** @addtogroup G_sine Sine wave generation
  * @{
 */
#include "sine_core.h"

#define SINTAB_ARGDIGITS 11
#define SINTAB_ENTRIES  (1 << SINTAB_ARGDIGITS)
/* Value range of sine lookup table */
#define SINTAB_MAX      (1 << SINE_DIGITS)

#define PHASE_SHIFT120  ((uint32_t)(     SINLU_ONEREV / 3))
#define PHASE_SHIFT240  ((uint32_t)(2 * (SINLU_ONEREV / 3)))

#define HZ_PER_DIGIT_8       ((256 * PWM_FREQ) / SINLU_ONEREV)
#define DIGIT_PER_HZ_8       ((256 * SINLU_ONEREV) / PWM_FREQ)

#define max(a,b,c) (a>b && a>c)?a:(b>a && b>c)?b:c
#define min(a,b,c) (a<b && a<c)?a:(b<a && b<c)?b:c

static int32_t SineLookup(uint16_t Arg);
static int32_t MultiplyAmplitude(uint16_t Amplitude, int32_t Baseval);
static int32_t CalcSVPWMOffset(int32_t a, int32_t b, int32_t c);

/** Minimum pulse width in normalized digits */
static uint32_t minPulse = 0;
static int32_t fdir = 0;
static uint32_t ampl = 0;
static uint16_t arg = 0;
static uint32_t jump = 0;
static u32fp frqFac = 0;
static u32fp lastFrq = 0;
static PWM_MODE pwmMode = PWM_SVPWM;
static const int16_t SinTab[] = { SINTAB };/* sine LUT */
static const uint16_t ZERO_OFFSET = SINTAB_MAX / 2;


/** Output the next dutycyles.
  * This function is meant to be called by your timer interrupt handler
  */
void sin_Step(uint32_t newDutyCycles[3] /**< outputs three new normalized dutycycles */)
{
    int32_t Ofs;
    uint32_t Idx;

    int32_t sine[3];

    if (0 != fdir)
    {

       /* 1. Calculate sine */
       sine[0] = SineLookup(arg);
       sine[1] = SineLookup((arg + PHASE_SHIFT120) & 0xFFFF);
       sine[2] = SineLookup((arg + PHASE_SHIFT240) & 0xFFFF);

       for (Idx = 0; Idx < 3; Idx++)
       {
          /* 2. Set desired amplitude  */
          sine[Idx] = MultiplyAmplitude(ampl, sine[Idx]);
       }

       /* 3. Calculate the offset of SVPWM */
       Ofs = CalcSVPWMOffset(sine[0], sine[1], sine[2]);

       for (Idx = 0; Idx < 3; Idx++)
       {
          /* 4. subtract it from all 3 phases -> no difference in phase-to-phase voltage */
          sine[Idx] -= Ofs;
          /* Shift above 0 */
          newDutyCycles[Idx] = sine[Idx] + ZERO_OFFSET;
          /* Short pulse supression */
          if (newDutyCycles[Idx] < minPulse)
          {
             newDutyCycles[Idx] = 0U;
          }
          else if (newDutyCycles[Idx] > (SINTAB_MAX - minPulse))
          {
             newDutyCycles[Idx] = SINTAB_MAX;
          }
       }

       arg += fdir * jump;
    }
    else
    {
       newDutyCycles[0] = ZERO_OFFSET;
       newDutyCycles[1] = ZERO_OFFSET;
       newDutyCycles[2] = ZERO_OFFSET;
    }
}

/** Set the sine frequency to be sythesized.
  * @pre Set the correct PWM frequency with sin_SetPwmFrq first
  */
void sin_SetFrq(u32fp frq /**< frequency */)
{
   lastFrq = frq;
   jump = FP_TOINT(FP_MUL(frqFac, frq));
}

/** Set amplitude of the synthesized sine wave */
void sin_SetAmp(uint32_t amp /**< amplitude in digit. Largest value is 37813 */)
{
   ampl = amp;
}

/** set three phase field direction */
void sin_SetDir(int32_t dir /**< -1=turn left, 1=turn right, 0=stop */)
{
   fdir = dir;
}

/** Set the calling frequency of sin_Step so
  * that sin_SetFrq can determine the correct
  * frequency.
 */
void sin_SetPwmFrq(uint32_t frq /**< PWM frequency in Hz */)
{
   /* Sine frequency is determined by how many lookup table entries are skipped on each step */
   /* The lookup table is adressed with 0..65535 */
   /* It contains one sine period */
   /* Now we determine the factor to calculate the jump width from a frequency in Hz */
   frqFac = FP_DIV(FP_FROMINT(SINLU_ONEREV), FP_FROMINT(frq));
   sin_SetFrq(lastFrq);
}

/** Selects pure sine wave mode or SVPWM mode */
void sin_SetPwmMode(PWM_MODE newMode)
{
   pwmMode = newMode;
}

/** Sets the minimum pulse width in normalized digits.
  * @post duty cylcles shorter than minWidth are supressed, both on the negative and the positive pulse
  */
void sin_SetMinPulseWidth(uint32_t minWidth)
{
   minPulse = minWidth;
}


/* Performs a lookup in the sine table */
/* 0 = 0, 2Pi = 65535 */
static int32_t SineLookup(uint16_t Arg)
{
    /* No interpolation for now */
    /* We divide arg by 2^(SINTAB_ARGDIGITS) */
    /* No we can directly address the lookup table */
    Arg >>= SINLU_ARGDIGITS - SINTAB_ARGDIGITS;
    return (int32_t)SinTab[Arg];
}

/* 0 = 0, 1 = 32767 */
static int32_t MultiplyAmplitude(uint16_t Amplitude, int32_t Baseval)
{
    int32_t Temp;
    Temp = (int32_t)((uint32_t)Amplitude * Baseval);
    /* Divide by 32768 */
    /* -> Allow overmodulation, for SVPWM or FTPWM */
    Temp >>= (SINE_DIGITS - 1);
    return Temp;
}

static int32_t CalcSVPWMOffset(int32_t a, int32_t b, int32_t c)
{
    if (PWM_SVPWM == pwmMode)
    {
        /* Formular for svpwm:
           Offset = 1/2 * (min{a,b,c} + max{a,b,c}) */
        /* this is valid only for a,b,c in [-37813,37813] */

        int32_t Minimum = min(a, b, c);
        int32_t Maximum = max(a, b, c);
        int32_t Offset = Minimum + Maximum;

        return (Offset >> 1);
    }
    return 0;
}

/** @} */
