#include "slip_controller.hpp"

#define HZ_TO_RPM(f) (f * 60)

static const char *paramNames[10] = { "maxslip", "revticks", "polepairs", "rotorfrq", "slip", 0 };
enum { P_MAXSLIP, P_REVTICKS, P_POLEPAIRS, V_ROTORFRQ, V_SLIP };

SlipController::SlipController(MotorControlHal *hal, Parameters *params, SineMotorController *controller) :
   hw(hal), _params(params), _controller(controller), scalDigits(16)
{
   paramValues[P_MAXSLIP] = 1 << scalDigits;
   paramValues[P_REVTICKS] = 60;
   paramValues[P_POLEPAIRS] = 2;
}

float PIRegler(float e)
{
   float Kp = 10;
   float Ki = 0.1;
   float T = 0.05;
   static float esum = 0;
   //regaus=kp*regdiffp + ki*ta*esum; //Reglergleichung
   esum += e;
   return Kp * e + Ki * T * esum;
}

void SlipController::Tick()
{
   float slip_spnt = 0.02; //2% Schlupf
   float e;
   //im Buch n_rotor = n
   float n_rotor = hw->GetRevTicks(); //TODO: filter
   //im Buch n_stator = f1
   float f_stator = _controller->GetCurFrq(); //TODO scale with pwm frq
   //Im Buch f_rotor = f2
   float f_rotor;
   float slip;
   n_rotor /= paramValues[P_REVTICKS];
   paramValues[V_ROTORFRQ] = HZ_TO_RPM(n_rotor);

   f_stator /= paramValues[P_POLEPAIRS];

   f_rotor = f_stator - n_rotor;

   slip = 1 - n_rotor / f_stator;//f_rotor / f_stator;
   paramValues[V_SLIP] = slip * 1000;

   e = slip_spnt - slip;
   float newfrq = PIRegler(e);

   _controller->SetFrqSpnt((int)newfrq);

   //hw->GetThrottle();

}
