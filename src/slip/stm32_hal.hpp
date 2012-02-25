#ifndef STM32_HAL_HPP_INCLUDED
#define STM32_HAL_HPP_INCLUDED

#include "libopencm3/stm32/timer.h"
#include "hal.hpp"
#include "stm32_irq.hpp"

class Stm32MotorControlHW : public MotorControlHal, private Stm32Irq
{
    public:
        Stm32MotorControlHW();
    /* Implement HAL interface */
    public:
      void SetDutyCycles(u16 DutyCycles[3]);
      u16 GetRevTicks();
      u16 GetB6Temp();
      u16 GetThrottle();
      u16 GetBusVoltage();
      bool IsReverseDrivingSelected();
      bool IsBrakePedalPressed();
      void SetMainBreaker(bool Close);
      void SetPrechargeRelay(bool Close);
      void ToggleLed();
      void SetTrigger(enum TrigState);

    private:
      void clock_setup();
      void gpio_setup();
      void tim_setup();
      void Isr();
      int pwmdigits;
      u16 lastRevTicks;
};

#endif // STM32_HAL_HPP_INCLUDED
