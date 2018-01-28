/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2011 Johannes Huebner <dev@johanneshuebner.com>
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

#include "my_string.h"
#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include "terminal.h"
#include <stdarg.h>

#define BUFSIZE 128
#define SET_RTS() /*gpio_set(GPIOB, GPIO11)*/
#define CLR_RTS() /*gpio_clear(GPIOB, GPIO11)*/

static const TERM_CMD *CmdLookup(char *buf);
static void term_send(uint32_t usart, const char *str);

extern const TERM_CMD TermCmds[];
static char inBuf[BUFSIZE];
static uint32_t _usart = 0xffffffff;

void term_Init(uint32_t usart)
{
   _usart = usart;
}
/** Run the terminal */
void term_Run()
{
   int idx = 0;
   char *argStart = NULL;
   char *argLast = NULL;
   const TERM_CMD *pCurCmd = NULL;
   const TERM_CMD *pLastCmd = NULL;
   char c;

   while (1)
   {
      c = usart_recv_blocking(_usart);
      usart_send(_usart, c); //No need to block here

      if ('\n' == c || '\r' == c || idx > (BUFSIZE - 2))
      {
         if (NULL != pCurCmd)
         {
            pCurCmd->CmdFunc(argStart);
            argLast = argStart;
            pLastCmd = pCurCmd;
            argStart = NULL;
            pCurCmd = NULL;
         }
         else if (idx > 0)
         {
            term_send(_usart, "Unknown command sequence\r\n");
         }
         idx = 0;
      }
      else if ('!' == c)
      {
         if (NULL != pLastCmd)
         {
            pLastCmd->CmdFunc(argLast);
         }
         idx = 0;
      }
      else
      {
         inBuf[idx] = c;
         idx++;
         inBuf[idx] = 0;
         if (NULL == argStart)
         {
            pCurCmd = CmdLookup(inBuf);
            if (NULL != pCurCmd)
            {
               argStart = &inBuf[idx];
            }
         }
         if (idx >= BUFSIZE)
         {
            idx--;
         }
      }
   } /* while(1) */
} /* term_Run */

int putchar(int c)
{
   if (_usart == 0xffffffff) return -1;
   usart_send_blocking(_usart, c);
   return 0;
}

static const TERM_CMD *CmdLookup(char *buf)
{
   const TERM_CMD *pCmd = TermCmds;

   for (; NULL != pCmd->cmd; pCmd++)
   {
      if (0 == my_strcmp(buf, pCmd->cmd))
      {
         break;
      }
   }
   if (NULL == pCmd->cmd)
   {
      pCmd = NULL;
   }
   return pCmd;
}

static void term_send(uint32_t usart, const char *str)
{
   SET_RTS();
   for (;*str > 0; str++)
       usart_send_blocking(usart, *str);
   CLR_RTS();
}


