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

#define OPMODES "0=OFF 1=SLIP 2=F_POT 3=F_SPNT"
#define PWMMODES "0=SINE 1=SVPWM"
#define PWMFRQS  "0=17.6kHz, 1=8.8kHz, 2=4.4KHz, 3=2.2kHz"
#define DIRS     "-1=FWD, 1=REV"
#define VERSION32(a,b,c,d) (a * 100000 + b * 1000 + c + d * 0.01)
#define VER VERSION32(0,0,0,32)

/*              name         unit       min     max     default ofs gain id */
#define PARAM_LIST \
    PARAM_ENTRY(boost,       "dig",     0,      37813,  1700,   0, 1.000, 1 ) \
    PARAM_ENTRY(fweak,       "Hz",      0,      200,    67,     0, 1.000, 2 ) \
    PARAM_ENTRY(slipmax,     "%",       0,      100,    5,      0, 1.000, 3 ) \
    PARAM_ENTRY(ampmin,      "%",       0,      100,    50,     0, 1.000, 4 ) \
    PARAM_ENTRY(slipkp,      "",        0,      100,    5,      0, 1.000, 5 ) \
    PARAM_ENTRY(speedflt,    "",        2,      100,    4,      0, 1.000, 6 ) \
    PARAM_ENTRY(minspeed,    "rpm",     0,      900,    100,    0, 1.000, 7 ) \
    PARAM_ENTRY(fspnt,       "Hz",      0,      300,    10,     0, 1.000, 8 ) \
    PARAM_ENTRY(fmax,        "Hz",      0,      400,    200,    0, 1.000, 9 ) \
    PARAM_ENTRY(ampmax,      "dig",     0,      37813,  37813,  0, 1.000, 11 ) \
    PARAM_ENTRY(pwmmode,     PWMMODES,  0,      1,      1,      0, 1.000, 12 ) \
    PARAM_ENTRY(pwmfrq,      PWMFRQS,   0,      3,      2,      0, 1.000, 13 ) \
    PARAM_ENTRY(deadtime,    "dig",     0,      255,    28,     0, 1.000, 14 ) \
    PARAM_ENTRY(num_imp,     "Imp/rev", 8,      8192,   60,     0, 1.000, 15 ) \
    PARAM_ENTRY(potmin,      "dig",     0,      4095,   0,      0, 1.000, 17 ) \
    PARAM_ENTRY(potmax,      "dig",     0,      4095,   4095,   0, 1.000, 18 ) \
    PARAM_ENTRY(brknom,      "%",       0,      100,    30,     0, 1.000, 19 ) \
    PARAM_ENTRY(udcsw,       "V",       0,      1000,   330,    1, 6.175, 20 ) \
    PARAM_ENTRY(ocurlim,     "A",       -400,   400,    100,    455, 4.500, 22 ) \
    PARAM_ENTRY(ucurlim,     "A",       -400,   400,   -100,    455, 4.500, 23 ) \
    PARAM_ENTRY(minpulse,    "dig",     0,      4095,   1000,   0, 1.000,  24 ) \
    PARAM_ENTRY(il1ofs,      "dig",     0,      4095,   2047,   0, 1.000, 25 ) \
    PARAM_ENTRY(il2ofs,      "dig",     0,      4095,   2047,   0, 1.000, 26 ) \
    PARAM_ENTRY(il1gain,     "dig/A",   0,      4095,   3.1,    0, 1.000, 27 ) \
    PARAM_ENTRY(il2gain,     "dig/A",   0,      4095,   3.1,    0, 1.000, 28 ) \
    PARAM_ENTRY(udcgain,     "dig/V",   0,      4095,   6.175,  0, 1.000, 29 ) \
    PARAM_ENTRY(version,     "",        VER,    VER,    VER,    0, 1.000,  0 ) \
    VALUE_ENTRY(opmode,      OPMODES,                           0, 1.000 ) \
    VALUE_ENTRY(udc,         "V",                               0, 1.000) \
    VALUE_ENTRY(il1,         "A",                               0, 1.000) \
    VALUE_ENTRY(il2,         "A",                               0, 1.000) \
    VALUE_ENTRY(il1rms,      "A",                               0, 1.000) \
    VALUE_ENTRY(il2rms,      "A",                               0, 1.000) \
    VALUE_ENTRY(slip,        "",                                0, 1.000 ) \
    VALUE_ENTRY(fstat,       "Hz",                              0, 1.000 ) \
    VALUE_ENTRY(slipspnt,    "%",                               0, 1.000 ) \
    VALUE_ENTRY(speed,       "rpm",                             0, 1.000 ) \
    VALUE_ENTRY(ctr,         "",                                0, 1.000 ) \
    VALUE_ENTRY(amp,         "dig",                             0, 1.000 ) \
    VALUE_ENTRY(ampnom,      "%",                               0, 1.000 ) \
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
    VALUE_ENTRY(din_bms,     "",                                0, 1.000 ) \
    VALUE_ENTRY(tm_meas,     "us",                              0, 1.000 ) \

