/* LibTomCrypt, modular cryptographic library -- Tom St Denis
 *
 * LibTomCrypt is a library that provides various cryptographic
 * algorithms in a highly modular and flexible manner.
 *
 * The library is free for all purposes without any express
 * guarantee it works.
 */
#include "tomcrypt.h"

/**
   @file pmac_ntz.c
   PMAC implementation, internal function, by Tom St Denis
*/

#ifdef LTC_PMAC

/**
  Internal PMAC function
*/
int pmac_ntz(unsigned long x)
{
   int c;
   x &= 0xFFFFFFFFUL;
   c = 0;
   while ((x & 1) == 0) {
      ++c;
      x >>= 1;
   }
   return c;
}

#endif

/* ref:         HEAD -> main */
/* git commit:  684efeb5120df0df20fb56771cda3d310858777d */
/* commit time: 2026-06-07 22:27:16 +0800 */
