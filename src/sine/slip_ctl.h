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
#ifndef SLIP_CTL_H_INCLUDED
#define SLIP_CTL_H_INCLUDED
#include "my_fp.h"


typedef uint16_t ctr_t;

#ifdef __cplusplus
extern "C"
{
#endif

u32fp ControlSlip(ctr_t ctrVal, s32fp invFrq);
s32fp GetSlip();
u32fp GetSpeed();
void SetSlipSpnt(s32fp slip);
void SetImpulsePerRev(ctr_t imp);
void SetMotorPolePairs(uint8_t pairs);
void SetMinSpeed(u32fp rpm);
void SetMaxFrq(u32fp frq);
void SetControlKp(s32fp kp);

#ifdef __cplusplus
}
#endif

#endif // SLIP_CTL_H_INCLUDED
