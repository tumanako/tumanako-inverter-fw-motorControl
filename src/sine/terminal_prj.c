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

#define STM32F1
#include <libopencm3/cm3/scb.h>
#include "terminal.h"
#include "params.h"
#include "my_string.h"
#include "my_fp.h"
#include "printf.h"
#include "param_save.h"

#define NUM_BUF_LEN 15

static void ParamGet(char *arg);
static void ParamSet(char *arg);
static void GetAll(char *arg);
static void PrintList(char *arg);
static void PrintAtr(char *arg);
static void StopInverter(char *arg);
static void StartInverter(char *arg);
static void SaveParameters(char *arg);
static void LoadParameters(char *arg);
static void Help(char *arg);
static void Reset(char *arg);

const TERM_CMD TermCmds[] =
{
  { "set", ParamSet },
  { "get", ParamGet },
  { "all", GetAll },
  { "list", PrintList },
  { "atr",  PrintAtr },
  { "stop", StopInverter },
  { "start", StartInverter },
  { "save", SaveParameters },
  { "load", LoadParameters },
  { "help", Help },
  { "reset", Reset },
  { NULL, NULL }
};

static void PrintList(char *arg)
{
   PARAM_NUM idx;
   const PARAM_ATTRIB *pAtr;

   arg = arg;

   printf("Available parameters and values\n");

   for (idx = 0; idx < PARAM_LAST; idx++)
   {
      pAtr = parm_GetAttrib(idx);
      printf("%s [%s]\n", pAtr->name, pAtr->unit);
   }
}

static void PrintAtr(char *arg)
{
   PARAM_NUM idx;
   const PARAM_ATTRIB *pAtr;

   arg = arg;

   printf("Parameter attributes\n");
   printf("Name\t\tmin - max [default]\n");

   for (idx = 0; idx < PARAM_LAST; idx++)
   {
      pAtr = parm_GetAttrib(idx);
      /* Only display for params */
      if (parm_IsParam(idx))
      {
         printf("%s\t\t%f - %f [%f]\n", pAtr->name,pAtr->min,pAtr->max,pAtr->def);
      }
   }
}

static void ParamGet(char *arg)
{
   PARAM_NUM idx;
   s32fp val;
   char* comma;
   char orig;

   arg = my_trim(arg);

   do
   {
      comma = (char*)my_strchr(arg, ',');
      orig = *comma;
      *comma = 0;

      idx = parm_NumFromString(arg);

      *comma = orig;
      arg = comma + 1;

      if (PARAM_INVALID != idx)
      {
         val = parm_GetScl(idx);
         printf("%f\n", val);
      }
      else
      {
         printf("Unknown parameter\n");
      }
   } while (',' == *comma);
}

static void GetAll(char *arg)
{
   const PARAM_ATTRIB *pAtr;

   arg = arg;

   for (int idx = 0; idx < PARAM_LAST; idx++)
   {
      pAtr = parm_GetAttrib(idx);
      printf("%s\t\t%f\n", pAtr->name, parm_GetScl(idx));
   }
}

static void ParamSet(char *arg)
{
   char *pParamVal;
   s32fp val;
   PARAM_NUM idx;

   arg = my_trim(arg);
   pParamVal = (char *)my_strchr(arg, ' ');

   if (*pParamVal == 0)
   {
      printf("No parameter value given\n");
      return;
   }

   *pParamVal = 0;
   pParamVal++;

   val = fp_atoi(pParamVal);
   idx = parm_NumFromString(arg);

   if (PARAM_INVALID != idx)
   {
       if (0 == parm_Set(idx, val))
       {
          printf("Set OK\n");
       }
       else
       {
          printf("Value out of range\n");
       }
   }
   else
   {
       printf("Unknown parameter %s\n", arg);
   }
}

static void StopInverter(char *arg)
{
    arg = arg;
    parm_SetDig(VALUE_opmode, 0);
    printf("Inverter halted.\n");
}

static void StartInverter(char *arg)
{
   arg = my_trim(arg);
   s32fp val = fp_atoi(arg);
   if (val <= FP_FROMINT(2))
   {
      parm_SetFlt(VALUE_opmode, val);
      printf("Inverter started\n");
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
   printf("Parameters stored, CRC=%x\n", crc);
}

static void LoadParameters(char *arg)
{
   arg = arg;
   if (0 == parm_load())
   {
      parm_Change(0);
      printf("Parameters loaded\n");
   }
   else
   {
      printf("Parameter CRC error\n");
   }
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
