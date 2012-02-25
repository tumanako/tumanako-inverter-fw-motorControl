#ifndef MOTOR_CONTROLLER_HPP_INCLUDED
#define MOTOR_CONTROLLER_HPP_INCLUDED
#include "hal.hpp"
#include "params.hpp"

enum ModSchemes
{
   MOD_SVPWM,
   MOD_SINE
};

class MotorController : public TimerHandler, public Configurable
{
   public:
      MotorController(Parameters *params) : _params(params) {}

   private:
      //Implement "Configurable" interface
      Parameters *GetParameters()
      {
         return _params;
      }

   protected:
      MotorControlHal *hw;
      Parameters *_params;
};

class SineMotorController : public MotorController
{
   public:
      SineMotorController(MotorControlHal *hal, Parameters *params);
      void SetFrqSpnt(int frqspnt);
      int GetCurFrq();

   private:
      int frq;
      int slewCtr;
      //unsigned short arg;
      void RampFrq();
      void CalcUf();
      void TimerInterrupt();
      int CalcSVPWMOffset(int  a, int  b, int  c);
      int MultiplyAmplitude(unsigned short  Amplitude, int  Baseval);
      short SineLookup(unsigned short  Arg);


      int paramValues[10];
};

#endif // MOTOR_CONTROLLER_HPP_INCLUDED
