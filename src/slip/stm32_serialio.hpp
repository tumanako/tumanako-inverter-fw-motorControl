#ifndef STM32_SERIALIO_HPP_INCLUDED
#define STM32_SERIALIO_HPP_INCLUDED

extern "C" {
#include <libopencm3/stm32/usart.h>
#include "libopencm3/stm32/nvic.h"
#include "libopencm3/stm32/f1/gpio.h"
}

#include "textio.hpp"
#include "stm32_irq.hpp"

template<u32 usart, u32 baudrate, u32 irq, u32 gpio, u32 pin>
class Stm32SerialIO : public TextIO, private Stm32Irq
{
   public:
      Stm32SerialIO()
      {
         Stm32Irq::Register(irq, this);

          /* Setup GPIO pin GPIO_USART3_TX/GPIO10 on GPIO port B for transmit. */
          gpio_set_mode(gpio, GPIO_MODE_OUTPUT_50_MHZ,
                        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, pin);

          /* Setup UART parameters. */
          usart_set_baudrate(usart, baudrate);
          usart_set_databits(usart, 8);
          usart_set_stopbits(usart, USART_STOPBITS_1);
          usart_set_mode(usart, USART_MODE_TX_RX);
          usart_set_parity(usart, USART_PARITY_NONE);
          usart_set_flow_control(usart, USART_FLOWCONTROL_NONE);
          USART_CR1(usart) |= USART_CR1_RXNEIE;
          nvic_enable_irq(Stm32Irq::IrqToNvicIrq(irq));
          nvic_set_priority(Stm32Irq::IrqToNvicIrq(irq), 0xF << 4);

          /* Finally enable the USART. */
          usart_enable(usart);
      }


   private:
      //Implement TextIO interface
      void SetChar(char c)
      {
         usart_send_blocking(usart, c);
      }

      bool WantEcho() const
      {
         return true;
      }

      char CommitChar() const
      {
         return '\r';
      }

      //Implement Stm32Irq interface
      void Isr()
      {
         if (USART_SR(usart) & USART_SR_RXNE)
         {
            char c=usart_recv(usart);
            this->handler->NewChar(c);
         }
      }
};


#endif // STM32_SERIALIO_HPP_INCLUDED
