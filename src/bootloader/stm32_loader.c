/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2010 Johannes Huebner <contact@johanneshuebner.com>
 * Copyright (C) 2010 Edward Cheeseman <cheesemanedward@gmail.com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define STM32F1

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/crc.h>
#include <libopencm3/cm3/scb.h>
#include "hwdefs.h"

#define PAGE_SIZE 1024
#define PAGE_WORDS (PAGE_SIZE / 4)
#define FLASH_START 0x08000000
#define APP_FLASH_START 0x08001000

static void clock_setup(void)
{
   RCC_CLOCK_SETUP();

   rcc_set_ppre1(RCC_CFGR_PPRE1_HCLK_DIV2);

   /* Enable all present GPIOx clocks. (whats with GPIO F and G?)*/
   rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
   rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
   rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN);
   rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPDEN);
   rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPEEN);

   #ifdef HWCONFIG_OLIMEX
   rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USART3EN);
   #endif
   #ifdef HWCONFIG_OLIMEX_H107
   rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USART2EN);
   rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_AFIOEN);
   gpio_primary_remap(AFIO_MAPR_SWJ_CFG_FULL_SWJ, AFIO_MAPR_USART2_REMAP);
   #endif
   #ifdef HWCONFIG_TUMANAKO_KIWIAC
   rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_USART1EN);
   #endif

   /* Enable DMA1 clock */
   rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA1EN);

   /* Enable CRC clock */
   rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_CRCEN);
}

static void usart_setup(void)
{
    gpio_set_mode(TERM_USART_TXPORT, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, TERM_USART_TXPIN);

    /* Setup UART parameters. */
    usart_set_baudrate(TERM_USART, USART_BAUDRATE);
    usart_set_databits(TERM_USART, 8);
    usart_set_stopbits(TERM_USART, USART_STOPBITS_2);
    usart_set_mode(TERM_USART, USART_MODE_TX_RX);
    usart_set_parity(TERM_USART, USART_PARITY_NONE);
    usart_set_flow_control(TERM_USART, USART_FLOWCONTROL_NONE);

    /* Finally enable the USART. */
    usart_enable(TERM_USART);
}

static void dma_setup(void *data)
{
   DMA1_CPAR3 = (uint32_t)&USART3_DR;
   DMA1_CMAR3 = (uint32_t)data;
   DMA1_CCR3 |= DMA_CCR_MSIZE_8BIT;
   DMA1_CCR3 |= DMA_CCR_PSIZE_8BIT;
   DMA1_CCR3 |= DMA_CCR_MINC;
   DMA1_CCR3 |= DMA_CCR_TCIE;
}

static uint32_t RecvCrc()
{
   uint32_t recvCrc = 0;

   recvCrc = usart_recv_blocking(TERM_USART);
   recvCrc |= usart_recv_blocking(TERM_USART) << 8;
   recvCrc |= usart_recv_blocking(TERM_USART) << 16;
   recvCrc |= usart_recv_blocking(TERM_USART) << 24;

   return recvCrc;
}

static void WriteFlash(uint32_t addr, uint32_t *pageBuffer)
{
   flash_erase_page(addr);

   for (int idx = 0; idx < PAGE_WORDS; idx++)
   {
      flash_program_word(addr + idx * 4, pageBuffer[idx]);
   }
}

void wait(void)
{
   for (volatile uint32_t i = 1 << 20; i > 0; i--);
}

int main(void)
{
   char numPages;
   uint32_t page_buffer[PAGE_WORDS];
   uint32_t addr = APP_FLASH_START;

   clock_setup();
   usart_setup();
   dma_setup(page_buffer);

   wait();
   usart_send_blocking(TERM_USART, 'S');
   wait();
   numPages = usart_recv(TERM_USART);

   flash_unlock();
   flash_set_ws(2);

   while (numPages > 0)
   {
      uint32_t recvCrc = 0;
      uint32_t timeOut = 1 << 21;
      CRC_CR |= CRC_CR_RESET;

      usart_send_blocking(TERM_USART, 'P');

      USART_CR3(TERM_USART) |= USART_CR3_DMAR;
      DMA1_CNDTR3 = PAGE_SIZE;
      DMA1_CCR3 |= DMA_CCR_EN;
      DMA1_IFCR = DMA_IFCR_CTCIF3;

      while ((DMA1_ISR & DMA_ISR_TCIF3) == 0)
      {
         timeOut--;

         //When the buffer is not full after about 200ms
         //Request the entire page again
         if (0 == timeOut)
         {
            usart_send_blocking(TERM_USART, 'T');
            timeOut = 1 << 21;
            DMA1_CCR3 &= ~DMA_CCR_EN;
            DMA1_CNDTR3 = PAGE_SIZE;
            DMA1_CCR3 |= DMA_CCR_EN;
         }
      }

      DMA1_CCR3 &= ~DMA_CCR_EN;
      USART_CR3(TERM_USART) &= ~USART_CR3_DMAR;

      for (uint32_t idx = 0; idx < PAGE_WORDS; idx++)
      {
         CRC_DR = page_buffer[idx];
      }

      usart_send_blocking(TERM_USART, 'C');
      recvCrc = RecvCrc();

      if (CRC_DR == recvCrc)
      {
         WriteFlash(addr, page_buffer);
         numPages--;
         addr += PAGE_SIZE;
      }
      else
      {
         usart_send_blocking(TERM_USART, 'E');
      }
   }

   usart_send_blocking(TERM_USART, 'D');
   wait();
   usart_disable(TERM_USART);

   flash_lock();

   void (*app_main)(void) = (void (*)(void)) *(volatile uint32_t*)(APP_FLASH_START + 4);
   SCB_VTOR = APP_FLASH_START;
   app_main();

   return 0;

}
