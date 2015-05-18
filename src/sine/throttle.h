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

#ifndef THROTTLE_H
#define THROTTLE_H

#include "my_fp.h"

class Throttle
{
   public:
      static bool CheckAndLimitRange(int* potval);
      static int CalcThrottle(int potval, bool brkpedal);
      static int CalcIdleSpeed(int speed);
      static int CalcCruiseSpeed(int speed);
      static int potmin;
      static int potmax;
      static int brknom;
      static int brknompedal;
      static int brkmax;
      static int idleSpeed;
      static int cruiseSpeed;
      static s32fp speedkp;
};

#endif // THROTTLE_H
