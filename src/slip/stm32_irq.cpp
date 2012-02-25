#include "stm32_irq.hpp"
#include <libopencm3/stm32/nvic.h>
#include <libopencm3/stm32/timer.h>

static Stm32Irq *jumpTable[IRQ_LAST];

#define IRQ_ENTRY(irq) NVIC_##irq##_IRQ,
static const unsigned int nvicLookup[] =
{
   HANDLED_IRQS
};
#undef IRQ_ENTRY

void Stm32Irq::Register(int irqIdx, Stm32Irq *handler)
{
   if (irqIdx < IRQ_LAST)
   {
      jumpTable[irqIdx] = handler;
   }
}

unsigned int Stm32Irq::IrqToNvicIrq(int irqIdx)
{
   if (irqIdx < IRQ_LAST)
   {
      return nvicLookup[irqIdx];
   }
   return 0xFFFFFFFF;
}

void Stm32Irq::usart1_isr()
{
   jumpTable[IRQ_USART1]->Isr();
}

void Stm32Irq::usart2_isr()
{
   jumpTable[IRQ_USART2]->Isr();
}

void Stm32Irq::tim1_up_isr()
{
   jumpTable[IRQ_TIM1_UP]->Isr();
}

void Stm32Irq::tim8_up_isr()
{
   jumpTable[IRQ_TIM8_UP]->Isr();
}

extern "C" void usart1_isr(void)
{
   Stm32Irq::usart1_isr();
}

extern "C" void tim1_up_isr(void)
{
   Stm32Irq::tim1_up_isr();
}
