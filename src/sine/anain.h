#ifndef ANAIO_H_INCLUDED
#define ANAIO_H_INCLUDED

#include <stdint.h>
#include "anain_prj.h"

#define ANA_IN_ENTRY(name, port, pin) name,
namespace Pin {
   enum AnaIns
   {
       ANA_IN_LIST
       LAST
   };
}
#undef ANA_IN_ENTRY

class AnaIn
{
public:
   static void Init(void);
   static uint16_t Get(Pin::AnaIns);

private:
   enum ports
   {
      PORTA = 0,
      PORTB,
      PORTC,
      PORT_LAST
   };

   enum pins
   {
      PIN0, PIN1, PIN2, PIN3, PIN4, PIN5, PIN6, PIN7,
      PIN8, PIN9,PIN10,PIN11,PIN12,PIN13,PIN14,PIN15,
      PIN_LAST
   };

   struct AnaInfo
   {
      uint8_t port;
      uint8_t pin;
   };

   static const AnaInfo ins[];
   static uint16_t values[];

   static uint8_t AdcChFromPort(int command_port, int command_bit);
};

#endif // ANAIO_H_INCLUDED
