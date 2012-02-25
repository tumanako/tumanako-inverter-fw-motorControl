#include "stm32_hal.hpp"
#include "stm32_serialio.hpp"
#include "terminal.hpp"
#include "motor_controller.hpp"
#include "hwdefs.h"
#include "terminal_prj.hpp"

static const TerminalCommand *commands[5];

int main(void)
{
   Stm32MotorControlHW hw;
   Parameters p;
   SineMotorController controller(&hw, &p);
   Stm32SerialIO<TERM_USART, USART_BAUDRATE, IRQ_USART1, GPIOA, TERM_USART_TXPIN> serIO;

   const TerminalCommandSet set(&controller);
   const TerminalCommandGet get(&controller);
   const TerminalCommandTest test;

   commands[0] = &set;
   commands[1] = &get;
   commands[2] = 0;

   Terminal Terminal1(commands, &serIO);

   while (1)
   {
      int i = 5000000;
      while (i > 0)
         i--;
      hw.ToggleLed();
   }
   return 0;
}
