#ifndef SLIP_CONTROLLER_HPP_INCLUDED
#define SLIP_CONTROLLER_HPP_INCLUDED
#include "params.hpp"
#include "hal.hpp"
#include "motor_controller.hpp"
#include "mediator.hpp"

class SlipController : public Configurable
{
   public:
      SlipController(MotorControlHal *hal, Parameters *params, SineMotorController *controller, Mediator<int, 10> *m);
      void Tick();

   private:
      MotorControlHal *hw;
      Parameters *_params;
      SineMotorController *_controller;
      Mediator<int, 10> *_medianFilter;
      const int scalDigits;
      int paramValues[10];

      Parameters *GetParameters();
};

#endif // SLIP_CONTROLLER_HPP_INCLUDED
