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

#define OPMODES "0=OFF, 1=RUN, 2=MANUAL_RUN, 3=BOOST"
#define PWMFRQS  "0=17.6kHz, 1=8.8kHz, 2=4.4KHz, 3=2.2kHz, 4=1.1kHz"
#define PWMPOLS  "0=ACTHIGH, 1=ACTLOW"
#define DIRS     "-1=REV, 0=NEUTRAL, 1=FWD"
#define SNS_HS   "0=JCurve, 1=Semikron"
#define SNS_M    "2=KTY83-110, 3=KTY84-130"
#define PWMFUNCS "0=tmpm, 1=tmphs, 2=speed"
#define CRUISEMODS "0=Button, 1=Switch"
#define IDLEMODS "0=always, 1=nobrake"
#define VER 3.08

#define BUTTON 0
#define MOD_OFF    0
#define MOD_RUN    1
#define MOD_MANUAL 2
#define MOD_BOOST  3

#define PWM_FUNC_TMPM  0
#define PWM_FUNC_TMPHS 1
#define PWM_FUNC_SPEED 2

#define IDLE_MODE_ALWAYS 0
#define IDLE_MODE_NOBRAKE 1


/* Entries must be ordered as follows:
   1. Saveable parameters (id != 0)
   2. Temporary parameters (id = 0)
   3. Display values
 */
//Next param id (increase when adding new parameter!): 68
/*              name         unit       min     max     default id */
#define PARAM_LIST \
    PARAM_ENTRY(boost,       "dig",     0,      37813,  1700,   1   ) \
    PARAM_ENTRY(fweak,       "Hz",      0,      400,    67,     2   ) \
    PARAM_ENTRY(fpconst,     "Hz",      0,      400,    400,    60  ) \
    PARAM_ENTRY(fslipmin,    "Hz",      0,      100,    1,      37  ) \
    PARAM_ENTRY(fslipmax,    "Hz",      0,      100,    3,      33  ) \
    PARAM_ENTRY(polepairs,   "",        1,      16,     2,      32  ) \
    PARAM_ENTRY(ampmin,      "%",       0,      100,    50,     4   ) \
    PARAM_ENTRY(encflt,      "",        0,      16,     4,      6   ) \
    PARAM_ENTRY(fmin,        "Hz",      0,      400,    1,      34  ) \
    PARAM_ENTRY(fmax,        "Hz",      0,      400,    200,    9   ) \
    PARAM_ENTRY(pwmfrq,      PWMFRQS,   0,      4,      2,      13  ) \
    PARAM_ENTRY(pwmpol,      PWMPOLS,   0,      1,      0,      52  ) \
    PARAM_ENTRY(deadtime,    "dig",     0,      255,    28,     14  ) \
    PARAM_ENTRY(numimp,      "Imp/rev", 8,      8192,   60,     15  ) \
    PARAM_ENTRY(potmin,      "dig",     0,      4095,   0,      17  ) \
    PARAM_ENTRY(potmax,      "dig",     0,      4095,   4095,   18  ) \
    PARAM_ENTRY(pot2min,     "dig",     0,      4095,   4095,   63  ) \
    PARAM_ENTRY(pot2max,     "dig",     0,      4095,   4095,   64  ) \
    PARAM_ENTRY(idlespeed,   "rpm",     -100,   1000,   -100,   54  ) \
    PARAM_ENTRY(idlethrotlim,"%",       0,      100,    50,     65  ) \
    PARAM_ENTRY(idlemode,    IDLEMODS,  0,      1,      0,      66  ) \
    PARAM_ENTRY(speedkp,     "",        0,      100,    0.25,   53  ) \
    PARAM_ENTRY(speedflt,    "",        0,      16,     1,      57  ) \
    PARAM_ENTRY(cruisemode,  CRUISEMODS,0,      1,      0,      62  ) \
    PARAM_ENTRY(bmslimhigh,  "%",       0,      100,    50,     55  ) \
    PARAM_ENTRY(bmslimlow,   "%",       -100,   0,      -1,     56  ) \
    PARAM_ENTRY(brknompedal, "%",       -100,   0,      -50,    38  ) \
    PARAM_ENTRY(brknom,      "%",       0,      100,    30,     19  ) \
    PARAM_ENTRY(brkmax,      "%",       0,      100,    30,     49  ) \
    PARAM_ENTRY(brkrampstr,  "Hz",      0,      400,    10,     39  ) \
    PARAM_ENTRY(brkout,      "%",       -100,   -1,      -50,    67  ) \
    PARAM_ENTRY(udcsw,       "V",       0,      1000,   330,    20  ) \
    PARAM_ENTRY(udcmin,      "V",       0,      1000,   450,    42  ) \
    PARAM_ENTRY(udcmax,      "V",       0,      1000,   520,    43  ) \
    PARAM_ENTRY(udclim,      "V",       0,      1000,   540,    48  ) \
    PARAM_ENTRY(ocurlim,     "A",       -1000,  1000,   -100,   22  ) \
    PARAM_ENTRY(minpulse,    "dig",     0,      4095,   1000,   24  ) \
    PARAM_ENTRY(il1gain,     "dig/A",   -100,   100,    -4.7,   27  ) \
    PARAM_ENTRY(il2gain,     "dig/A",   -100,   100,    -4.7,   28  ) \
    PARAM_ENTRY(udcgain,     "dig/V",   0,      4095,   6.175,  29  ) \
    PARAM_ENTRY(pwmfunc,     PWMFUNCS,  0,      2,      0,      58  ) \
    PARAM_ENTRY(pwmgain,     "dig/C",   -65535, 65535,  100,    40  ) \
    PARAM_ENTRY(pwmofs,      "dig",     -65535, 65535,  0,      41  ) \
    PARAM_ENTRY(speedgain,   "rpm/kHz", 0,      65535,  6000,   59  ) \
    PARAM_ENTRY(snshs,       SNS_HS,    0,      1,      0,      45  ) \
    PARAM_ENTRY(snsm,        SNS_M,     2,      3,      2,      46  ) \
    PARAM_ENTRY(fslipspnt,   "Hz",      -100,   100,    0,      0   ) \
    PARAM_ENTRY(version,     "",        0,      0,      VER,    0   ) \
    PARAM_ENTRY(ampnom,      "%",       0,      100,    0,      0   ) \
    VALUE_ENTRY(opmode,      OPMODES ) \
    VALUE_ENTRY(udc,         "V"     ) \
    VALUE_ENTRY(idc,         "A"     ) \
    VALUE_ENTRY(il1,         "A"     ) \
    VALUE_ENTRY(il2,         "A"     ) \
    VALUE_ENTRY(uac,         "V"     ) \
    VALUE_ENTRY(il1rms,      "A"     ) \
    VALUE_ENTRY(il2rms,      "A"     ) \
    VALUE_ENTRY(id,          "A"     ) \
    VALUE_ENTRY(iq,          "A"     ) \
    VALUE_ENTRY(p,           "kW"    ) \
    VALUE_ENTRY(q,           "kVA"   ) \
    VALUE_ENTRY(s,           "kVA"   ) \
    VALUE_ENTRY(pf,          ""      ) \
    VALUE_ENTRY(t,           "Nm"    ) \
    VALUE_ENTRY(fstat,       "Hz"    ) \
    VALUE_ENTRY(speed,       "rpm"   ) \
    VALUE_ENTRY(amp,         "dig"   ) \
    VALUE_ENTRY(pot,         "dig"   ) \
    VALUE_ENTRY(pot2,        "dig"   ) \
    VALUE_ENTRY(potnom,      "%"     ) \
    VALUE_ENTRY(dir,         DIRS    ) \
    VALUE_ENTRY(tmphs,       "°C"    ) \
    VALUE_ENTRY(tmpm,        "°C"    ) \
    VALUE_ENTRY(din_cruise,  ""      ) \
    VALUE_ENTRY(din_start,   ""      ) \
    VALUE_ENTRY(din_brake,   ""      ) \
    VALUE_ENTRY(din_mprot,   ""      ) \
    VALUE_ENTRY(din_forward, ""      ) \
    VALUE_ENTRY(din_reverse, ""      ) \
    VALUE_ENTRY(din_emcystop,""      ) \
    VALUE_ENTRY(din_ocur,    ""      ) \
    VALUE_ENTRY(din_bms,     ""      ) \
    VALUE_ENTRY(tm_meas,     "us"    ) \

