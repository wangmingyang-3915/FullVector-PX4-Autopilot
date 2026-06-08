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
  @file der_length_boolean.c
  ASN.1 DER, get length of a BOOLEAN, Tom St Denis
*/

#ifdef LTC_DER
/**
  Gets length of DER encoding of a BOOLEAN
  @param outlen [out] The length of the DER encoding
  @return CRYPT_OK if successful
*/
int der_length_boolean(unsigned long *outlen)
{
   LTC_ARGCHK(outlen != NULL);
   *outlen = 3;
   return CRYPT_OK;
}

#endif

/* ref:         HEAD -> main */
/* git commit:  684efeb5120df0df20fb56771cda3d310858777d */
/* commit time: 2026-06-07 22:27:16 +0800 */
