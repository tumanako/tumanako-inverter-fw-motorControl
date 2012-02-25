#define STM32F1
extern "C" {
#include <libopencm3/stm32/usart.h>
#include "libopencm3/stm32/nvic.h"
#include "libopencm3/stm32/f1/gpio.h"
}
#include "stm32_serialio.hpp"
#include "hwdefs.h"

void Stm32SerialIO<u32 irq>::Isr(void)
{
   if (USART_SR(usart) & USART_SR_RXNE)
   {
      char c=usart_recv(usart);
      this->handler->NewChar(c);
   }
}

#if 0
template<u32 irq>
Stm32SerialIO::Stm32SerialIO(u32 usart, u32 baudrate) :
   usart(usart)
{
   Stm32Irq::Register(irq, this);

    /* Setup GPIO pin GPIO_USART3_TX/GPIO10 on GPIO port B for transmit. */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, TERM_USART_TXPIN);

    /* Setup UART parameters. */
    usart_set_baudrate(usart, baudrate);
    usart_set_databits(usart, 8);
    usart_set_stopbits(usart, USART_STOPBITS_1);
    usart_set_mode(usart, USART_MODE_TX_RX);
    usart_set_parity(usart, USART_PARITY_NONE);
    usart_set_flow_control(usart, USART_FLOWCONTROL_NONE);
    USART_CR1(TERM_USART) |= USART_CR1_RXNEIE;
    nvic_enable_irq(irq);
    nvic_set_priority(irq, 0xF << 4);

    /* Finally enable the USART. */
    usart_enable(usart);
}
#endif

void Stm32SerialIO<u32 irq>::SetChar(char c)
{
   usart_send_blocking(usart, c);
}
