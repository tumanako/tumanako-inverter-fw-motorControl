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

#define OPMODES "0=OFF, 1=RUN, 2=MANUAL_RUN"
#define PWMFRQS  "0=17.6kHz, 1=8.8kHz, 2=4.4KHz, 3=2.2kHz"
#define DIRS     "-1=FWD, 0=NEUTRAL, 1=REV"
#define SNS_HS   "0=JCurve, 1=Semikron"
#define SNS_M    "2=KTY83-110, 3=KTY84-130"
#define VER 2.60

/* Entries must be ordered as follows:
   1. Saveable parameters (id != 0)
   2. Temporary parameters (id = 0)
   3. Display values
 */

/*              name         unit       min     max     default ofs gain id */
#define PARAM_LIST \
    PARAM_ENTRY(boost,       "dig",     0,      37813,  1700,   0, 1.000, 1   ) \
    PARAM_ENTRY(fweak,       "Hz",      0,      400,    67,     0, 1.000, 2   ) \
    PARAM_ENTRY(fslipmin,    "Hz",      0,      100,    1,      0, 1.000, 37  ) \
    PARAM_ENTRY(fslipmax,    "Hz",      0,      100,    3,      0, 1.000, 33  ) \
    PARAM_ENTRY(polepairs,   "",        1,      16,     2,      0, 1.000, 32  ) \
    PARAM_ENTRY(ampmin,      "%",       0,      100,    50,     0, 1.000, 4   ) \
    PARAM_ENTRY(speedflt,    "",        0,      16,     4,      0, 1.000, 6   ) \
    PARAM_ENTRY(fmin,        "Hz",      0,      400,    1,      0, 1.000, 34  ) \
    PARAM_ENTRY(fmax,        "Hz",      0,      400,    200,    0, 1.000, 9   ) \
    PARAM_ENTRY(pwmfrq,      PWMFRQS,   0,      3,      2,      0, 1.000, 13  ) \
    PARAM_ENTRY(deadtime,    "dig",     0,      255,    28,     0, 1.000, 14  ) \
    PARAM_ENTRY(numimp,      "Imp/rev", 8,      8192,   60,     0, 1.000, 15  ) \
    PARAM_ENTRY(potmin,      "dig",     0,      4095,   0,      0, 1.000, 17  ) \
    PARAM_ENTRY(potmax,      "dig",     0,      4095,   4095,   0, 1.000, 18  ) \
    PARAM_ENTRY(brknompedal, "%",       -100,   0,      -50,    0, 1.000, 38  ) \
    PARAM_ENTRY(brknom,      "%",       0,      100,    30,     0, 1.000, 19  ) \
    PARAM_ENTRY(brkrampstr,  "Hz",      0,      400,    10,     0, 1.000, 39  ) \
    PARAM_ENTRY(udcsw,       "V",       0,      1000,   330,    0, 1.000, 20  ) \
    PARAM_ENTRY(udcmin,      "V",       0,      1000,   450,    0, 1.000, 42  ) \
    PARAM_ENTRY(udcmax,      "V",       0,      1000,   520,    0, 1.000, 43  ) \
    PARAM_ENTRY(ocurlim,     "A",       -500,   500,    100,    0, 1.000, 22 ) \
    PARAM_ENTRY(minpulse,    "dig",     0,      4095,   1000,   0, 1.000, 24 ) \
    PARAM_ENTRY(il1ofs,      "dig",     0,      4095,   1988,   0, 1.000, 25  ) \
    PARAM_ENTRY(il2ofs,      "dig",     0,      4095,   1988,   0, 1.000, 26  ) \
    PARAM_ENTRY(il1gain,     "dig/A",   -100,   100,    -4.7,   0, 1.000, 27  ) \
    PARAM_ENTRY(il2gain,     "dig/A",   -100,   100,    -4.7,   0, 1.000, 28  ) \
    PARAM_ENTRY(udcgain,     "dig/V",   0,      4095,   6.175,  0, 1.000, 29  ) \
    PARAM_ENTRY(tmpgain,     "dig/C",   0,      65535,  100,    0, 1.000, 40  ) \
    PARAM_ENTRY(tmpofs,      "dig",     -65535, 65535,  0,      0, 1.000, 41  ) \
    PARAM_ENTRY(snshs,       SNS_HS,    0,      1,      0,      0, 1.000, 45  ) \
    PARAM_ENTRY(snsm,        SNS_M,     2,      3,      2,      0, 1.000, 46  ) \
    PARAM_ENTRY(fslipspnt,   "Hz",      -100,   100,    0,      0, 1.000, 0   ) \
    PARAM_ENTRY(version,     "",        0,      0,      VER,    0, 1.000,  0  ) \
    PARAM_ENTRY(ampnom,      "%",       0,      100,    0,      0, 1.000, 0 ) \
    VALUE_ENTRY(opmode,      OPMODES,                           0, 1.000 ) \
    VALUE_ENTRY(udc,         "V",                               0, 1.000 ) \
    VALUE_ENTRY(idc,         "A",                               0, 1.000 ) \
    VALUE_ENTRY(il1,         "A",                               0, 1.000 ) \
    VALUE_ENTRY(il2,         "A",                               0, 1.000 ) \
    VALUE_ENTRY(uac,         "V",                               0, 1.000 ) \
    VALUE_ENTRY(il1rms,      "A",                               0, 1.000 ) \
    VALUE_ENTRY(il2rms,      "A",                               0, 1.000 ) \
    VALUE_ENTRY(id,          "A",                               0, 1.000 ) \
    VALUE_ENTRY(iq,          "A",                               0, 1.000 ) \
    VALUE_ENTRY(p,           "kW",                              0, 1.000 ) \
    VALUE_ENTRY(q,           "kVA",                             0, 1.000 ) \
    VALUE_ENTRY(s,           "kVA",                             0, 1.000 ) \
    VALUE_ENTRY(pf,          "",                                0, 1.000 ) \
    VALUE_ENTRY(fstat,       "Hz",                              0, 1.000 ) \
    VALUE_ENTRY(speed,       "rpm",                             0, 1.000 ) \
    VALUE_ENTRY(amp,         "dig",                             0, 1.000 ) \
    VALUE_ENTRY(pot,         "dig",                             0, 1.000 ) \
    VALUE_ENTRY(potnom,      "%",                               0, 1.000 ) \
    VALUE_ENTRY(dir,         DIRS,                              0, 1.000 ) \
    VALUE_ENTRY(tmphs,       "°C",                              0, 1.000 ) \
    VALUE_ENTRY(tmpm,        "°C",                              0, 1.000 ) \
    VALUE_ENTRY(din_on,      "",                                0, 1.000 ) \
    VALUE_ENTRY(din_start,   "",                                0, 1.000 ) \
    VALUE_ENTRY(din_brake,   "",                                0, 1.000 ) \
    VALUE_ENTRY(din_mprot,   "",                                0, 1.000 ) \
    VALUE_ENTRY(din_forward, "",                                0, 1.000 ) \
    VALUE_ENTRY(din_reverse, "",                                0, 1.000 ) \
    VALUE_ENTRY(din_emcystop,"",                                0, 1.000 ) \
    VALUE_ENTRY(din_ocur,    "",                                0, 1.000 ) \
    VALUE_ENTRY(din_bms,     "",                                0, 1.000 ) \
    VALUE_ENTRY(tm_meas,     "us",                              0, 1.000 ) \

