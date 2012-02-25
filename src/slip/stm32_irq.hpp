#ifndef STM32_IRQ_HPP_INCLUDED
#define STM32_IRQ_HPP_INCLUDED

#define HANDLED_IRQS    \
   IRQ_ENTRY(USART1)    \
   IRQ_ENTRY(USART2)    \
   IRQ_ENTRY(TIM1_UP)   \
   IRQ_ENTRY(TIM8_UP)   \

#define IRQ_ENTRY(irq) IRQ_##irq,
enum
{
   HANDLED_IRQS
   IRQ_LAST
};
#undef IRQ_ENTRY

#define MAX_INTERRUPTS NVIC_

class Stm32Irq
{
   public:
      Stm32Irq()
      {
      }

      static void Register(int IrqIdx, Stm32Irq *DerivedThis);
      static void usart1_isr();
      static void usart2_isr();
      static void tim1_up_isr();
      static void tim8_up_isr();
      static unsigned int IrqToNvicIrq(int IrqIdx);

      virtual void Isr() = 0;
};


#endif // STM32_IRQ_HPP_INCLUDED
