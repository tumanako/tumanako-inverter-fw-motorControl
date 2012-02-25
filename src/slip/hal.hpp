#ifndef HAL_HPP_INCLUDED
#define HAL_HPP_INCLUDED

class TimerHandler
{
   public:
      virtual void TimerInterrupt() = 0;
};

enum TrigState
{
   TRIGGER_RISE,
   TRIGGER_FALL
};

//template <typename RegType>
#define RegType unsigned short
class MotorControlHal
{
    public:
      virtual void SetDutyCycles(RegType DutyCycles[3]) = 0;
      virtual RegType GetRevTicks() = 0;
      virtual RegType GetB6Temp() = 0;
      virtual RegType GetThrottle() = 0;
      virtual RegType GetBusVoltage() = 0;
      virtual bool IsReverseDrivingSelected() = 0;
      virtual bool IsBrakePedalPressed() = 0;
      virtual void SetMainBreaker(bool Close) = 0;
      virtual void SetPrechargeRelay(bool Close) = 0;
      virtual void ToggleLed() = 0;
      virtual void SetTrigger(enum TrigState) = 0;

      void SetTimerHandler(TimerHandler *handler)
      {
         this->timerHandler = handler;
      }

   protected:
      TimerHandler *timerHandler;
};


#endif // HAL_HPP_INCLUDED
