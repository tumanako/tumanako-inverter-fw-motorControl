#include "my_string.h"
#include "my_fp.h"

#define FRAC_MASK ((1 << FRAC_DIGITS) - 1)


char* fp_itoa(char * buf, s32fp a)
{
   int sign = a < 0?-1:1;
   int32_t nat = (sign * a) >> FRAC_DIGITS;
   uint32_t frac = ((UTOA_FRACDEC * ((sign * a) & FRAC_MASK))) >> FRAC_DIGITS;
   char *p = buf;
   if (sign < 0)
   {
      *p = '-';
      p++;
   }
   p += my_ltoa(p, nat, 10);
   *p = '.';
   p++;
   for (uint32_t dec = UTOA_FRACDEC / 10; dec > 1; dec /= 10)
   {
      if ((frac / dec) == 0)
      {
         *p = '0';
         p++;
      }
   }
   my_ltoa(p, frac, 10);
   return buf;
}

s32fp fp_atoi(char *str)
{
   int32_t nat = my_atoi(str);
   const char *decPoint = my_strchr(str, '.');
   int32_t frac = 0;

   if (0 != *decPoint)
   {
      decPoint++;

      frac = my_atoi(decPoint);
      frac <<= FRAC_DIGITS;
      frac /= UTOA_FRACDEC;
   }
   return FP_FROMINT(nat) + frac;
}
