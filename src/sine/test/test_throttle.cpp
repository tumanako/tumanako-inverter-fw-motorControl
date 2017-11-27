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

#include "my_fp.h"
#include "my_math.h"
#include "test_list.h"
#include "throttle.h"

using namespace std;

static void TestSetup()
{
      Throttle::potmin[0] = 1000;
      Throttle::potmax[0] = 2000;
      Throttle::potmin[1] = 3000;
      Throttle::potmax[1] = 4000;
      Throttle::brknom = 30;
      Throttle::brknompedal = -60;
      Throttle::brkPedalRamp = 25;
      Throttle::brkmax = 50;
      Throttle::idleSpeed = 100;
      Throttle::speedkp = FP_FROMFLT(0.25);
      Throttle::speedflt = 5;
      Throttle::idleThrotLim = FP_FROMFLT(30);
}

static void TestBrkPedal()
{
   int percent = Throttle::CalcThrottle(1500, 3000, true);
   ASSERT(percent == -25)
   percent = Throttle::CalcThrottle(1500, 3000, true);
   ASSERT(percent == -50)
   percent = Throttle::CalcThrottle(1500, 3000, true);
   ASSERT(percent == -60)
   percent = Throttle::CalcThrottle(1500, 3000, true);
   ASSERT(percent == -60)
}

static void TestRegen()
{
   Throttle::CalcThrottle(2000, 3000, false);
   int percent = Throttle::CalcThrottle(1000, 3000, false);
   ASSERT(percent == -25)
}

void ThrottleTest::RunTest()
{
   TestSetup();
   TestBrkPedal();
   TestRegen();
}
