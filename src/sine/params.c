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

#include "params.h"
#include "my_string.h"

#define PARAM_ENTRY(category, name, unit, min, max, def, id) { category, #name, unit, FP_FROMFLT(min), FP_FROMFLT(max), FP_FROMFLT(def), id },
#define VALUE_ENTRY(name, unit) { 0, #name, unit, 0, 0, 0, 0 },
static const PARAM_ATTRIB attribs[] =
{
    PARAM_LIST
};
#undef PARAM_ENTRY
#undef VALUE_ENTRY

#define PARAM_ENTRY(category, name, unit, min, max, def, id) FP_FROMFLT(def),
#define VALUE_ENTRY(name, unit) 0,
static s32fp values[] =
{
    PARAM_LIST
};
#undef PARAM_ENTRY
#undef VALUE_ENTRY

/**
* Set a parameter
*
* @param[in] ParamNum Parameter index
* @param[in] ParamVal New value of parameter
* @return 0 if set ok, -1 if ParamVal outside of allowed range
*/
int parm_Set(PARAM_NUM ParamNum, s32fp ParamVal)
{
    char res = -1;

    if (ParamVal >= attribs[ParamNum].min && ParamVal <= attribs[ParamNum].max)
    {
        values[ParamNum] = ParamVal;
        parm_Change(ParamNum);
        res = 0;
    }
    return res;
}

/**
* Get a parameters fixed point value
*
* @param[in] ParamNum Parameter index
* @return Parameters value
*/
s32fp parm_Get(PARAM_NUM ParamNum)
{
    return values[ParamNum];
}

/**
* Get a parameters unscaled digit value
*
* @param[in] ParamNum Parameter index
* @return Parameters value
*/
int parm_GetInt(PARAM_NUM ParamNum)
{
    return FP_TOINT(values[ParamNum]);
}

/**
* Set a parameters digit value
*
* @param[in] ParamNum Parameter index
* @param[in] ParamVal New value of parameter
*/
void parm_SetDig(PARAM_NUM ParamNum, int ParamVal)
{
   values[ParamNum] = FP_FROMINT(ParamVal);
}

/**
* Set a parameters fixed point value
*
* @param[in] ParamNum Parameter index
* @param[in] ParamVal New value of parameter
*/
void parm_SetFlt(PARAM_NUM ParamNum, s32fp ParamVal)
{
   values[ParamNum] = ParamVal;
}

/**
* Get the paramater index from a parameter name
*
* @param[in] name Parameters name
* @return Parameter index if found, PARAM_INVALID otherwise
*/
PARAM_NUM parm_NumFromString(const char *name)
{
    PARAM_NUM ParamNum = PARAM_INVALID;
    PARAM_NUM CurNum = 0;
    const PARAM_ATTRIB *pCurAtr = attribs;

    for (; CurNum < PARAM_LAST; CurNum++, pCurAtr++)
    {
         if (0 == my_strcmp(pCurAtr->name, name))
         {
             ParamNum = CurNum;
             break;
         }
    }
    return ParamNum;
}

/**
* Get the parameter attributes
*
* @param[in] ParamNum Parameter index
* @return Parameter attributes
*/
const PARAM_ATTRIB *parm_GetAttrib(PARAM_NUM ParamNum)
{
    return &attribs[ParamNum];
}

/** Find out if ParamNum is a parameter or display value
 * @retval 1 it is a parameter
 * @retval 0 otherwise
 */
int parm_IsParam(PARAM_NUM ParamNum)
{
   return attribs[ParamNum].min != attribs[ParamNum].max;
}

/** Load default values for all parameters */
void parm_LoadDefaults()
{
   const PARAM_ATTRIB *curAtr = attribs;

   for (PARAM_NUM idx = 0; idx < PARAM_LAST; idx++, curAtr++)
   {
      if (curAtr->id > 0)
         parm_SetFlt(idx, curAtr->def);
   }
}
