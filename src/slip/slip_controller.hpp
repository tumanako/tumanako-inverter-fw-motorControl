#ifndef SLIP_CONTROLLER_HPP_INCLUDED
#define SLIP_CONTROLLER_HPP_INCLUDED
#include "params.hpp"
#include "hal.hpp"
#include "motor_controller.hpp"

class SlipController : public Configurable
{
   public:
      SlipController(MotorControlHal *hal, Parameters *params, SineMotorController *controller);
      void Tick();

   private:
      const int scalDigits;
      Parameters *_params;
      SineMotorController *_controller;
      MotorControlHal *hw;
      int paramValues[10];

      Parameters *GetParameters();
};

#endif // SLIP_CONTROLLER_HPP_INCLUDED
