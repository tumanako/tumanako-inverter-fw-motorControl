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
 /** @addtogroup G_slip Slip controller
  * @{
 */
#define CST_DIGITS 10
#include "my_fp.h"
#include "slip_ctl.h"

static ctr_t impulsePerRev = 1;
static s32fp slipSpnt = 0;
static s32fp curSlip = 0;
static s32fp _kp = FP_FROMINT(5);
static u32fp speed = 0;
static uint8_t polePairs = 1;
static u32fp minSpeed = FP_FROMINT(100);
static u32fp maxFrq = FP_FROMINT(300);

/*
PID_HANDLE pid =
{
   FP_FROMFLT(5.0),
   FP_FROMFLT(0.0),
   FP_FROMFLT(0),
   FP_FROMFLT(-4),
   FP_FROMFLT(4),
   FP_FROMFLT(0.01),
   0, 0
};*/

/** Calculates new frequency to match slip setpoint
 * @param period Time in µs between two adjacent encoder teeth
 * @param invFrq Current inverter frequency
 * @return frequency to match slip setpoint in Hz
*/
u32fp ControlSlip(ctr_t period, s32fp invFrq)
{
   invFrq = CST_CONVERT(invFrq);
   u32fp n = FP_FROMINT(1000000000 / (period * impulsePerRev)) / 1000;
   speed = 60 * n;

   if (speed < minSpeed)
   {
      if (slipSpnt > 0)
         n = minSpeed / 60;
      else
         return 0;
   }

   u32fp n1 = invFrq / polePairs;
   s32fp slip = FP_FROMINT(1) - FP_DIV(n, n1);
   s32fp slipErr = slipSpnt - slip;
   u32fp newFrq = invFrq + FP_MUL(_kp, slipErr);
   curSlip = FP_FROMINT(1) - FP_DIV(n, newFrq / polePairs);

   if (newFrq > maxFrq)
      newFrq = maxFrq;

   return CST_ICONVERT(newFrq);
}

/** returns the slip calculated by ControlSlip
 * @pre call ControlSlip
 * @return slip in %
*/
s32fp GetSlip()
{
   return CST_ICONVERT(100 * curSlip);
}

/** Get the rotors current speed
 * @return rotor speed in rpm
 */
u32fp GetSpeed()
{
   return CST_ICONVERT(speed);
}

/** Set the number of impulses per 1 revelation of the motor shaft */
void SetImpulsePerRev(ctr_t imp)
{
   impulsePerRev = imp;
}

/** Set slip setpoint.
 * - 0% = synchronous speed
 * - 100% = standstill
 * - 5%   = field spins 5% faster than rotor (accelerate)
 * - -5%  = field spins 5% slower than rotor (brake)
 *
 * @param slip slip setpoint in percent
 */
void SetSlipSpnt(s32fp slip)
{
   slipSpnt = CST_CONVERT(slip) / 100;
}

/** Set the number of pole pairs of the motor.
 *  - 1 = 2 pole motor (1 rev per period)
 *  - 2 = 4 pole motor (0.5 rev per period)
 *  - 3 = 6 pole motor etc.
 *
 * @param pairs
 */
void SetMotorPolePairs(uint8_t pairs)
{
   polePairs = pairs;
}

/** Set the minimum regulating rotor speed.
 *  No control takes place below that speed
 * @param rpm minimum rotor speed, speeds lower than this are mapped to this
 */
void SetMinSpeed(u32fp rpm)
{
   minSpeed = CST_CONVERT(rpm);
}

/** Set the maximum allowed output frequency */
void SetMaxFrq(u32fp frq)
{
   maxFrq = CST_CONVERT(frq);
}

/** Set the slip controllers error amplification */
void SetControlKp(s32fp kp)
{
   _kp = CST_CONVERT(kp);
}

/** } */
