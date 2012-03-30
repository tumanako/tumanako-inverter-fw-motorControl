#include "slip_controller.hpp"
#include "mediator.hpp"

#define HZ_TO_RPM(f) (f * 60)

static const char *paramNames[10] = { "maxslip", "revticks", "polepairs", "rotorfrq", "slip", 0 };
enum { P_MAXSLIP, P_REVTICKS, P_POLEPAIRS, V_ROTORFRQ, V_SLIP };

SlipController::SlipController(MotorControlHal *hal, Parameters *params, SineMotorController *controller, Mediator<int, 10> *m) :
   hw(hal), _params(params), _controller(controller), _medianFilter(m), scalDigits(16)
{
   paramValues[P_MAXSLIP] = 1 << scalDigits;
   paramValues[P_REVTICKS] = 60;
   paramValues[P_POLEPAIRS] = 2;
   _params->SetInfo(paramNames, paramValues);
}

Parameters *SlipController::GetParameters()
{
   return _params;
}


float PIRegler(int e)
{
   const int windup = 500;
   const int Kp = 0;
   const int Ki = 5;
   const int T = 1;
   static int esum = 0;
   esum += e;
   if (esum < -windup) esum = -windup;
   if (esum > windup) esum = windup;
   float result = Kp * e + Ki * T * esum;
   return result / 1000;
}

void SlipController::Tick()
{
   float slip_spnt = hw->GetThrottle();
   slip_spnt /= 1000;
   float e;
   //im Buch n_rotor = n
   _medianFilter->Insert(hw->GetRevTicks());
   float n_rotor = _medianFilter->Median();
   //im Buch n_stator = f1
   float f_stator = _controller->GetCurFrq(); //TODO scale with pwm frq
   return;
#if 1
   if (0.0 == slip_spnt)
   {
      _controller->SetFrqSpnt(5);
      return;
   }
   if (f_stator < 1.0)
   {
      _controller->SetFrqSpnt(1);
      return;
   }
#endif
   //Im Buch f_rotor = f2
   float f_rotor;
   float slip;
   //n_rotor /= paramValues[P_REVTICKS];
   //paramValues[V_ROTORFRQ] = HZ_TO_RPM(n_rotor);

   //f_stator /= paramValues[P_POLEPAIRS];

   f_rotor = f_stator - n_rotor;

   slip = 1 - n_rotor / f_stator;//f_rotor / f_stator;
   paramValues[V_SLIP] = slip * 1000;

   e = slip_spnt - slip;
   float newfrq = PIRegler(e*1000);

   _controller->SetFrqSpnt((int)newfrq);

   //hw->GetThrottle();

}
