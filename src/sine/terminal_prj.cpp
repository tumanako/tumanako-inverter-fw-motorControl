/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2011 Johannes Huebner <dev@johanneshuebner.com>
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

#include <libopencm3/cm3/scb.h>
#include "terminal.h"
#include "params.h"
#include "my_string.h"
#include "my_fp.h"
#include "printf.h"
#include "param_save.h"
#include "errormessage.h"
#include "pwmgeneration.h"
#include "stm32_can.h"

#define NUM_BUF_LEN 15

static void ParamGet(char *arg);
static void ParamSet(char *arg);
static void LoadDefaults(char *arg);
static void GetAll(char *arg);
static void PrintList(char *arg);
static void PrintAtr(char *arg);
static void StopInverter(char *arg);
static void StartInverter(char *arg);
static void SaveParameters(char *arg);
static void LoadParameters(char *arg);
static void Help(char *arg);
static void PrintParamsJson(char *arg);
static void MapCanTx(char *arg);
static void PrintErrors(char *arg);
static void Reset(char *arg);

extern "C" const TERM_CMD TermCmds[] =
{
  { "set", ParamSet },
  { "get", ParamGet },
  { "defaults", LoadDefaults },
  { "all", GetAll },
  { "list", PrintList },
  { "atr",  PrintAtr },
  { "stop", StopInverter },
  { "start", StartInverter },
  { "save", SaveParameters },
  { "load", LoadParameters },
  { "help", Help },
  { "json", PrintParamsJson },
  { "cantx", MapCanTx },
  { "errors", PrintErrors },
  { "reset", Reset },
  { NULL, NULL }
};
//cantx param id offset len
static void MapCanTx(char *arg)
{
   Param::PARAM_NUM paramIdx;
   int values[4];

   arg = my_trim(arg);

   for (int i = -1; i < 4; i++)
   {
      char *ending = (char *)my_strchr(arg, ' ');

      if (0 == *ending && i < 3)
      {
         printf("Missing argument\r\n");
         return;
      }

      *ending = 0;

      if (i < 0)
         paramIdx = Param::NumFromString(arg);
      else
         values[i] = my_atoi(arg);
      arg = ending + 1;
   }

   printf("%d,%d,%d,%d\r\n", paramIdx, values[0], values[1], values[2]);

   if (Param::PARAM_INVALID != paramIdx)
   {
       if (can_addsend(paramIdx, values[0], values[1], values[2], values[3]) < 0)
       {
          printf("Value out of range\r\n");
       }
       else
       {
          printf("Set OK\r\n");
       }
   }
   else
   {
       printf("Unknown parameter %s\r\n", arg);
   }
}

static void PrintParamsJson(char *arg)
{
   const Param::Attributes *pAtr;
   char comma = ' ';

   arg = arg;
   printf("{");
   for (uint32_t idx = 0; idx < Param::PARAM_LAST; idx++)
   {
      pAtr = Param::GetAttrib((Param::PARAM_NUM)idx);

      printf("%c\r\n   \"%s\": {\"unit\":\"%s\",\"value\":%f,\"isparam\":",comma, pAtr->name, pAtr->unit, Param::Get((Param::PARAM_NUM)idx));

      if (Param::IsParam((Param::PARAM_NUM)idx))
      {
         printf("true,\"minimum\":%f,\"maximum\":%f,\"default\":%f,\"category\":\"%s\"}", pAtr->min, pAtr->max, pAtr->def, pAtr->category);
      }
      else
      {
         printf("false}");
      }
      comma = ',';
   }
   printf("\r\n}\r\n");
}

static void PrintList(char *arg)
{
   const Param::Attributes *pAtr;

   arg = arg;

   printf("Available parameters and values\r\n");

   for (uint32_t idx = 0; idx < Param::PARAM_LAST; idx++)
   {
      pAtr = Param::GetAttrib((Param::PARAM_NUM)idx);
      printf("%s [%s]\r\n", pAtr->name, pAtr->unit);
   }
}

static void PrintAtr(char *arg)
{
   const Param::Attributes *pAtr;

   arg = arg;

   printf("Parameter attributes\r\n");
   printf("Name\t\tmin - max [default]\r\n");

   for (uint32_t idx = 0; idx < Param::PARAM_LAST; idx++)
   {
      pAtr = Param::GetAttrib((Param::PARAM_NUM)idx);
      /* Only display for params */
      if (Param::IsParam((Param::PARAM_NUM)idx))
      {
         printf("%s\t\t%f - %f [%f]\r\n", pAtr->name,pAtr->min,pAtr->max,pAtr->def);
      }
   }
}

static void ParamGet(char *arg)
{
   Param::PARAM_NUM idx;
   s32fp val;
   char* comma;
   char orig;

   arg = my_trim(arg);

   do
   {
      comma = (char*)my_strchr(arg, ',');
      orig = *comma;
      *comma = 0;

      idx = Param::NumFromString(arg);

      *comma = orig;
      arg = comma + 1;

      if (Param::PARAM_INVALID != idx)
      {
         val = Param::Get(idx);
         printf("%f\r\n", val);
      }
      else
      {
         printf("Unknown parameter\r\n");
      }
   } while (',' == *comma);
}

static void LoadDefaults(char *arg)
{
   arg = arg;
   Param::LoadDefaults();
   printf("Defaults loaded\r\n");
}

static void GetAll(char *arg)
{
   const Param::Attributes *pAtr;

   arg = arg;

   for (uint32_t  idx = 0; idx < Param::PARAM_LAST; idx++)
   {
      pAtr = Param::GetAttrib((Param::PARAM_NUM)idx);
      printf("%s\t\t%f\r\n", pAtr->name, Param::Get((Param::PARAM_NUM)idx));
   }
}

static void ParamSet(char *arg)
{
   char *pParamVal;
   s32fp val;
   Param::PARAM_NUM idx;

   arg = my_trim(arg);
   pParamVal = (char *)my_strchr(arg, ' ');

   if (*pParamVal == 0)
   {
      printf("No parameter value given\r\n");
      return;
   }

   *pParamVal = 0;
   pParamVal++;

   val = fp_atoi(pParamVal);
   idx = Param::NumFromString(arg);

   if (Param::PARAM_INVALID != idx)
   {
       if (0 == Param::Set(idx, val))
       {
          printf("Set OK\r\n");
       }
       else
       {
          printf("Value out of range\r\n");
       }
   }
   else
   {
       printf("Unknown parameter %s\r\n", arg);
   }
}

static void StopInverter(char *arg)
{
    arg = arg;
    Param::SetDig(Param::opmode, 0);
    printf("Inverter halted.\r\n");
}

static void StartInverter(char *arg)
{
   arg = my_trim(arg);
   s32fp val = fp_atoi(arg);
   if (val < FP_FROMINT(MOD_LAST))
   {
      Param::SetFlt(Param::opmode, val);
      PwmGeneration::SetOpmode(FP_TOINT(val));
      printf("Inverter started\r\n");
   }
   else
   {
      printf("Invalid inverter mode");
   }
}

static void SaveParameters(char *arg)
{
   arg = arg;
   uint32_t crc = parm_save();
   printf("Parameters stored, CRC=%x\r\n", crc);
}

static void LoadParameters(char *arg)
{
   arg = arg;
   if (0 == parm_load())
   {
      parm_Change((Param::PARAM_NUM)0);
      printf("Parameters loaded\r\n");
   }
   else
   {
      printf("Parameter CRC error\r\n");
   }
}

static void PrintErrors(char *arg)
{
   arg = arg;
   ErrorMessage::PrintAllErrors();
}

static void Help(char *arg)
{
   arg = arg;
}

static void Reset(char *arg)
{
   arg = arg;
   scb_reset_system();
}
