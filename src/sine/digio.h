#ifndef DIGIO_H_INCLUDED
#define DIGIO_H_INCLUDED

#define STM32F1
#include <libopencm3/stm32/gpio.h>
#include "digio_prj.h"

namespace PinMode {
   enum PinMode
   {
       INPUT_PD,
       INPUT_PU,
       INPUT_FLT,
       INPUT_AIN,
       OUTPUT,
       LAST
   };
}

#define DIG_IO_ENTRY(name, port, pin, mode) name,
namespace Pin {
   enum DigPin
   {
       DIG_IO_LIST
       DIG_IO_LAST
   };
}
#undef DIG_IO_ENTRY

class DigIo
{
public:
   static void Init(void);
   static bool Get(Pin::DigPin DigIos);
   static void Set(Pin::DigPin DigIos);
   static void Clear(Pin::DigPin DigIos);
   static void Toggle(Pin::DigPin DigIos);
};

#endif // DIGIO_H_INCLUDED
