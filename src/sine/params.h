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

#include "param_prj.h"
#include "my_fp.h"

#define PARAM_ENTRY(name, unit, min, max, def, ofs, fac, id) PARAM_##name,
#define VALUE_ENTRY(name, unit, ofs, fac) VALUE_##name,
typedef enum
{
    PARAM_LIST
    PARAM_LAST,
    PARAM_INVALID
} PARAM_NUM;
#undef PARAM_ENTRY
#undef VALUE_ENTRY

typedef enum
{
    TYPE_PARAM,
    TYPE_VALUE,
    TYPE_LAST
} PARAM_TYPE;

typedef struct
{
    char *name;
    char *unit;
    s32fp min;
    s32fp max;
    s32fp def;
    s32fp ofs;
    s32fp fac;
    uint32_t id;
} PARAM_ATTRIB;

#ifdef __cplusplus
extern "C"
{
#endif

char parm_Set(PARAM_NUM ParamNum, s32fp ParamVal);
s32fp  parm_Get(PARAM_NUM ParamNum);
int    parm_GetInt(PARAM_NUM ParamNum);
s32fp  parm_GetScl(PARAM_NUM ParamNum);
char parm_SetDig(PARAM_NUM ParamNum, int ParamVal);
char parm_SetFlt(PARAM_NUM ParamNum, s32fp ParamVal);
PARAM_NUM parm_NumFromString(const char *name);
const PARAM_ATTRIB *parm_GetAttrib(PARAM_NUM ParamNum);
char parm_IsParam(PARAM_NUM ParamNum);

//User defined callback
extern void parm_Change(PARAM_NUM ParamNum);

#ifdef __cplusplus
}
#endif
