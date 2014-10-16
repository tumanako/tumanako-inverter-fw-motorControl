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
   int nat = 0;
   int frac = 0;
   int div = 10;
   int sign = 1;
   if ('-' == *str)
   {
      sign = -1;
      str++;
   }
   for (; *str >= '0' && *str <= '9'; str++)
   {
      nat *= 10;
      nat += *str - '0';
   }
   if (*str != 0)
   {
      for (str++; *str >= '0' && *str <= '9'; str++)
      {
         frac += FP_FROMINT(*str - '0') / div;
         div *= 10;
      }
   }

   return sign * (FP_FROMINT(nat) + frac);
}

u32fp fp_sqrt(u32fp rad)
{
   u32fp sqrt = rad >> (rad<1000?4:8); //Starting value for newton iteration
   u32fp sqrtl;
   sqrt = sqrt>FP_FROMINT(1)?sqrt:FP_FROMINT(1); //Must be > 0

   do {
      sqrtl = sqrt;
      sqrt = (sqrt + FP_DIV(rad, sqrt)) >> 1;
   } while ((sqrtl - sqrt) > 1);

   return sqrt;
}
