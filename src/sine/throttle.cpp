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

#include "throttle.h"
#include "my_math.h"

#define POT_SLACK 200

int Throttle::potmin;
int Throttle::potmax;
int Throttle::brknom;
int Throttle::brknompedal;
int Throttle::brkmax;
int Throttle::idleSpeed;
int Throttle::cruiseSpeed;
s32fp Throttle::speedkp;
int Throttle::speedflt;
int Throttle::speedFiltered;

bool Throttle::CheckAndLimitRange(int* potval)
{
   if (((*potval + POT_SLACK) < potmin) || (*potval > (potmax + POT_SLACK)))
   {
      *potval = potmin;
      return false;
   }
   else if (*potval < potmin)
   {
      *potval = potmin;
   }
   else if (*potval > potmax)
   {
      *potval = potmax;
   }

   return true;
}

int Throttle::CalcThrottle(int potval, bool brkpedal)
{
   int potnom = 0;

   if (brkpedal)
   {
      potnom = brknompedal;
   }
   else
   {
      potnom = potval - potmin;
      potnom = ((100 + brknom) * potnom) / (potmax-potmin);
      potnom -= brknom;
      if (potnom < 0)
         potnom = (potnom * brkmax) / brknom;
   }

   return potnom;
}

int Throttle::CalcIdleSpeed(int speed)
{
   int speederr = idleSpeed - speed;
   return FP_TOINT(MIN(FP_FROMINT(50), speedkp * speederr));
}

int Throttle::CalcCruiseSpeed(int speed)
{
   speedFiltered = IIRFILTER(speedFiltered, speed, speedflt);
   int speederr = cruiseSpeed - speedFiltered;
   return FP_TOINT(MAX(FP_FROMINT(-100), MIN(FP_FROMINT(100), speedkp * speederr)));
}

int Throttle::TemperatureDerate(s32fp tmphs)
{
   if (tmphs < TMPHS_MAX)
      return 100;
   if (tmphs < (TMPHS_MAX + 2))
      return 50;
   return 0;
}
