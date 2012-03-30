#include "motor_controller.hpp"
#include "hal.hpp"
#include "lookup_tables.h"

/* Value range of sine lookup table */
#define SINTAB_DIGITS    16
#define SINTAB_MAX      (1 << SINTAB_DIGITS)
/* Domain of lookup function */
#define SINLU_ARGDIGITS  16
#define SINLU_ONEREV    (1 << SINLU_ARGDIGITS)
#define PWM_DIGITS      11
#define PWM_MAX         (1 << PWM_DIGITS)
#define PHASE_SHIFT120  ((unsigned int)(     SINLU_ONEREV / 3))
#define PHASE_SHIFT240  ((unsigned int)(2 * (SINLU_ONEREV / 3)))

#define max(a,b,c) (a>b && a>c)?a:(b>a && b>c)?b:c

#define min(a,b,c) (a<b && a<c)?a:(b<a && b<c)?b:c

static const unsigned short SinTab[] = { SINTAB };/* sine LUT */
static const char *paramNames[10] = { "frqspnt", "amp", "pwmmod", "frq", "boostamp", "boostfrq", "maxamp", "weakfrq", "slewrate", 0 };
enum { P_FRQSPNT, V_AMP, P_PWMMOD, V_FRQ, P_BOOSTAMP, P_BOOSTFRQ, P_MAXAMP, P_WEAKFRQ, P_SLEWRATE };

SineMotorController::SineMotorController(MotorControlHal *hal, Parameters *params) :
MotorController(params)
{
   hal->SetTimerHandler(this);
   hw = hal;
   slewCtr = 0;
   paramValues[P_FRQSPNT] = 0;
   paramValues[P_PWMMOD] = MOD_SVPWM;
   paramValues[P_BOOSTAMP] = 2000;
   paramValues[P_BOOSTFRQ] = 1;
   paramValues[P_MAXAMP] = 37810;
   paramValues[P_WEAKFRQ] = 120;
   paramValues[P_SLEWRATE] = 100;
   _params->SetInfo(paramNames, paramValues);
   frq = 0;
}

void SineMotorController::SetFrqSpnt(int frqspnt)
{
   paramValues[P_FRQSPNT] = frqspnt;
}

int SineMotorController::GetCurFrq()
{
   return frq;
}

void SineMotorController::RampFrq()
{
   if (frq < paramValues[P_FRQSPNT])
      frq++;
   else if (frq > paramValues[P_FRQSPNT])
      frq--;
}

void SineMotorController::CalcUf()
{
   if (paramValues[V_FRQ] == 0)
   {
      paramValues[V_AMP] = 0;
   }
   else if (paramValues[V_FRQ] < paramValues[P_BOOSTFRQ])
   {
      paramValues[V_AMP] = paramValues[P_BOOSTAMP];
   }
   else if (paramValues[V_FRQ] < paramValues[P_WEAKFRQ])
   {
      int m = (paramValues[P_MAXAMP] - paramValues[P_BOOSTAMP])/(paramValues[P_WEAKFRQ] - paramValues[P_BOOSTFRQ]);
      int b = paramValues[P_MAXAMP] - (m * paramValues[P_WEAKFRQ]);
      paramValues[V_AMP] = m*paramValues[V_FRQ] + b;
   }
   else
   {
      paramValues[V_AMP] = paramValues[P_MAXAMP];
   }
}

void SineMotorController::TimerInterrupt(void)
{
   int Sine[3], Ofs;
   unsigned short DutyCycle[3];
   static unsigned short arg;

   if (slewCtr++ >= paramValues[P_SLEWRATE])
   {
      RampFrq();
      CalcUf();
      paramValues[V_FRQ] = frq;
      slewCtr= 0;
   }

   /* 1. Calculate sine */
   Sine[0] = SineLookup(arg);
   Sine[1] = SineLookup((unsigned short)(((unsigned int)arg + PHASE_SHIFT120) & 0xFFFF));
   Sine[2] = SineLookup((unsigned short)(((unsigned int)arg + PHASE_SHIFT240) & 0xFFFF));

   for (int Idx = 0; Idx < 3; Idx++)
   {
      /* 4. Set desired amplitude and match to PWM resolution */
      Sine[Idx] = MultiplyAmplitude(paramValues[V_AMP], Sine[Idx]);
   }
   /* 2. Calculate the offset of SVPWM */
   Ofs = CalcSVPWMOffset(Sine[0], Sine[1], Sine[2]);

   for (int Idx = 0; Idx < 3; Idx++)
   {
      /* 3. subtract it from all 3 phases -> no difference in phase-to-phase voltage */
      Sine[Idx] -= Ofs;
      DutyCycle[Idx] = Sine[Idx] + 32768;
      /* Match to PWM resolution */
      DutyCycle[Idx] >>= (SINTAB_DIGITS - PWM_DIGITS);
      if (DutyCycle[Idx] > PWM_MAX) DutyCycle[Idx] = PWM_MAX;
   }
   hw->SetDutyCycles(DutyCycle);

   if ((16384 >= (arg - frq + 1)) && (16384 <= (arg + frq - 1)))
   {
      hw->SetTrigger(TRIGGER_RISE);
   }
   else
   {
      hw->SetTrigger(TRIGGER_FALL);
   }
   arg+=frq;
}

/* Performs a lookup in the sine table */
/* 0 = 0, 2Pi = 65535 */
short SineMotorController::SineLookup(unsigned short  Arg)
{
    /* No interpolation for now */
    /* We divide arg by 2^(SINTAB_ARGDIGITS) */
    /* No we can directly address the lookup table */
    Arg >>= SINLU_ARGDIGITS - SINTAB_ARGDIGITS;
    return SinTab[Arg] - 32768;
}

/* 0 = 0, 1 = 32767 */
int SineMotorController::MultiplyAmplitude(unsigned short  Amplitude, int  Baseval)
{
    int Temp;
    Temp = (int)(unsigned int)Amplitude * Baseval;
    /* Divide by 32768 */
    /* -> Allow overmodulation, for SVPWM or FTPWM */
    Temp >>= (SINTAB_DIGITS - 1);
    return Temp;
}

int SineMotorController::CalcSVPWMOffset(int a, int b, int c)
{
    if (MOD_SVPWM == paramValues[P_PWMMOD])
    {
        /* Formular for svpwm:
           Offset = 1/2 * (min{a,b,c} + max{a,b,c}) */
        /* this is valid only for a,b,c in [-37813,37813] */

        int Minimum = min(a, b, c);
        int Maximum = max(a, b, c);
        int Offset = Minimum + Maximum;

        return (Offset >> 1);
    }
    return 0;
}
