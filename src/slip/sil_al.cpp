#include "sil_al.hpp"
#include <plplot/plplotP.h>
#include <plplot/wxPLplotwindow.h>
#include "sil2Main.h"
#include "params.hpp"

static PLFLT x[600], y[3][600], yold[3][600];

SilMotorControlHW::SilMotorControlHW(sil2Frame *sil) :
   _sil(sil)
{
   timerHandler = 0;
   for (int i = 0; i < 600; i++)
   {
      x[i] = i;
   }
   samples = 0;
   _throttle = 0;
}


void SilMotorControlHW::SetDutyCycles(RegType DutyCycles[3])
{
   wxPLplotstream* pls = _sil->pls;

   if (samples < 600)
   {
      yold[0][samples] = y[0][samples];
      yold[1][samples] = y[1][samples];
      yold[2][samples] = y[2][samples];
      y[0][samples] = DutyCycles[0];
      y[1][samples] = DutyCycles[1];
      y[2][samples] = DutyCycles[2];
      samples++;
   }

   if (599 == samples)
   {
      for (int i = 0; i < 3; i++)
      {
         pls->col0(0);
         pls->line(600, x, yold[i]);
      }
      for (int i = 0; i < 3; i++)
      {
         pls->col0(i+1);
         pls->line(600, x, y[i]);
      }
      _sil->Refresh(false, 0);
   }
}

RegType SilMotorControlHW::GetRevTicks()
{
   return _revTicks;
}

RegType SilMotorControlHW::GetB6Temp()
{
}

RegType SilMotorControlHW::GetThrottle()
{
   return _throttle;
}

RegType SilMotorControlHW::GetBusVoltage()
{
}

bool SilMotorControlHW::IsReverseDrivingSelected()
{
}

bool SilMotorControlHW::IsBrakePedalPressed()
{
}

void SilMotorControlHW::SetMainBreaker(bool Close)
{
}

void SilMotorControlHW::SetPrechargeRelay(bool Close)
{
}

void SilMotorControlHW::ToggleLed()
{
}

void SilMotorControlHW::SetTrigger(enum TrigState state)
{
   extern int freq;
   if (TRIGGER_RISE == state)
   {
      if (samples == 600)
      {
         samples = 0;
         freq = _sil->controller->GetParameter("frq");
         _sil->settingsDlg->frqSpin->SetValue(freq);
      }
   }
   else
   {
   }
}

void SilMotorControlHW::Tick()
{
   if (0 != timerHandler)
      timerHandler->TimerInterrupt();
}

void SilMotorControlHW::SetRevTicks(int ticks)
{
   _revTicks = ticks;
}

void SilMotorControlHW::SetThrottle(int val)
{
   _throttle = val;
}
