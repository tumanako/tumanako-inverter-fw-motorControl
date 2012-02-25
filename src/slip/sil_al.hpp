#ifndef SIL_AL_HPP_INCLUDED
#define SIL_AL_HPP_INCLUDED

#include "hal.hpp"
#include "sil2Main.h"

class SilMotorControlHW : public MotorControlHal
{
    public:
        SilMotorControlHW(sil2Frame *sil);
    /* Implement HAL interface */
    public:
        virtual void SetDutyCycles(RegType DutyCycles[3]);
        virtual RegType GetRevTicks();
        virtual RegType GetB6Temp();
        virtual RegType GetThrottle();
        virtual RegType GetBusVoltage();
        virtual bool IsReverseDrivingSelected();
        virtual bool IsBrakePedalPressed();
        virtual void SetMainBreaker(bool Close);
        virtual void SetPrechargeRelay(bool Close);
        virtual void ToggleLed();
        virtual void SetTrigger(enum TrigState state);
        void Tick();
        void SetRevTicks(int ticks);

    private:
      int samples;
      int _revTicks;
      sil2Frame *_sil;
};


#endif // SIL_AL_HPP_INCLUDED
