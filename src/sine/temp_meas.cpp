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
#define __TEMP_LU_TABLES
#include "temp_meas.h"
#include <stdint.h>

enum coeff { PTC, NTC };

typedef struct TempSensor
{
   int tempMin;
   int tempMax;
   uint8_t step;
   uint8_t tabSize;
   enum coeff coeff;
   const uint16_t *lookup;
} TEMP_SENSOR;

/* Temp sensor with JCurve */
static const uint16_t JCurve[] = { JCURVE };

/* Temp sensor in Semikron Skiip82 module */
static const uint16_t Semikron[] = { SEMIKRON };

/* Temp sensor KTY83-110 */
static const uint16_t Kty83[] = { KTY83 };

/* Temp sensor KTY84-130 */
static const uint16_t Kty84[] = { KTY84 };

static const TEMP_SENSOR sensors[] =
{
   { -25, 106, 5,  sizeof(JCurve) / sizeof(JCurve[0]),     NTC, JCurve },
   { 0,   100, 5,  sizeof(Semikron) / sizeof(Semikron[0]), PTC, Semikron },
   { -50, 170, 10, sizeof(Kty83) / sizeof(Kty83[0]),       PTC, Kty83  },
   { -40, 300, 10, sizeof(Kty84) / sizeof(Kty84[0]),       PTC, Kty84  },
};

s32fp TempMeas::Lookup(int digit, Sensors sensorId)
{
   if (sensorId >= TEMP_LAST) return 0;

   const TEMP_SENSOR * sensor = &sensors[sensorId];
   unsigned int Idx;
   uint16_t last = sensor->lookup[0] + (sensor->coeff == NTC?-1:+1);

   for (Idx = 0; Idx < sensor->tabSize; Idx++)
   {
      uint16_t cur = sensor->lookup[Idx];
      if ((sensor->coeff == NTC && cur >= digit) || (sensor->coeff == PTC && cur <= digit))
      {
         s32fp a = FP_FROMINT(sensor->coeff == NTC?cur - digit:digit - cur);
         s32fp b = FP_FROMINT(sensor->coeff == NTC?cur - last:last - cur);
         return FP_FROMINT(sensor->step * Idx + sensor->tempMin) - sensor->step * FP_DIV(a, b);
      }
      last = cur;
   }
   return FP_FROMINT(sensor->tempMax);
}
/*
s32fp temp_JCurve(int digit)
{
   unsigned int Idx;
   uint16_t last = JCurve[0] - 1;

   for (Idx = 0; Idx < JCURVE_TAB_ENTRIES; Idx++)
   {
      uint16_t cur = JCurve[Idx];
      if (cur >= digit)
      {
         s32fp a = FP_FROMINT(cur - digit);
         s32fp b = FP_FROMINT(cur - last);
         return FP_FROMINT(JCURVE_STEP * Idx + JCURVE_MIN) - JCURVE_STEP * FP_DIV(a, b);
      }
      last = cur;
   }
   return FP_FROMINT(JCURVE_MAX);
}

s32fp temp_KTY83(int digit)
{
   unsigned int Idx;
   uint16_t last = Kty83[0] + 1;

   for (Idx = 0; Idx < KTY83_TAB_ENTRIES; Idx++)
   {
      uint16_t cur = Kty83[Idx];
      if (cur <= digit)
      {
         s32fp a = FP_FROMINT(digit - cur);
         s32fp b = FP_FROMINT(last - cur);
         return FP_FROMINT((KTY83_STEP * Idx) + KTY83_MIN) - KTY83_STEP * FP_DIV(a, b);
      }
      last = cur;
   }
   return FP_FROMINT(KTY83_MAX);
}*/
