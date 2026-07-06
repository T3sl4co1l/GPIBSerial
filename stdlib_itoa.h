/* Copyright (c) 2002, Marek Michalkiewicz
   Copyright (c) 2004,2007 Joerg Wunsch

   Portions of documentation Copyright (c) 1990, 1991, 1993, 1994
   The Regents of the University of California.

   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

  $Id: stdlib.h 2503 2016-02-07 22:59:47Z joerg_wunsch $

  * Trimmed to just itoa family functions for convenience -- TMW 2022
*/

//	Only need this file on avr-gcc 8.1.0, not really sure why, but <stdlib.h>
//	doesn't include these. Cheap hack, conditionally paste in 11.1.0's definitions.
#if __GNUC__ == 8

#ifndef _STDLIB_ITOA_H_
#define	_STDLIB_ITOA_H_ 1

#ifndef __ASSEMBLER__

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __DOXYGEN__

#ifndef __ATTR_CONST__
# define __ATTR_CONST__ __attribute__((__const__))
#endif

#ifndef __ATTR_MALLOC__
# define __ATTR_MALLOC__ __attribute__((__malloc__))
#endif

#ifndef __ATTR_NORETURN__
# define __ATTR_NORETURN__ __attribute__((__noreturn__))
#endif

#ifndef __ATTR_PURE__
# define __ATTR_PURE__ __attribute__((__pure__))
#endif

#ifndef	__ATTR_GNU_INLINE__
# ifdef  __GNUC_STDC_INLINE__
#  define __ATTR_GNU_INLINE__   __attribute__((__gnu_inline__))
# else
#  define __ATTR_GNU_INLINE__
# endif
#endif

#endif

/*@{*/
/** \name Non-standard (i.e. non-ISO C) functions.
 \ingroup avr_stdlib
*/
/**
   \brief Convert an integer to a string.

   The function itoa() converts the integer value from \c val into an
   ASCII representation that will be stored under \c s.  The caller
   is responsible for providing sufficient storage in \c s.

   \note The minimal size of the buffer \c s depends on the choice of
   radix. For example, if the radix is 2 (binary), you need to supply a buffer
   with a minimal length of 8 * sizeof (int) + 1 characters, i.e. one
   character for each bit plus one for the string terminator. Using a larger
   radix will require a smaller minimal buffer size.

   \warning If the buffer is too small, you risk a buffer overflow.

   Conversion is done using the \c radix as base, which may be a
   number between 2 (binary conversion) and up to 36.  If \c radix
   is greater than 10, the next digit after \c '9' will be the letter
   \c 'a'.

    If radix is 10 and val is negative, a minus sign will be prepended.

   The itoa() function returns the pointer passed as \c s.
*/
#ifdef  __DOXYGEN__
extern char *itoa(int val, char *s, int radix);
#else
extern __inline__ __ATTR_GNU_INLINE__
char *itoa (int __val, char *__s, int __radix)
{
    if (!__builtin_constant_p (__radix)) {
	extern char *__itoa (int, char *, int);
	return __itoa (__val, __s, __radix);
    } else if (__radix < 2 || __radix > 36) {
	*__s = 0;
	return __s;
    } else {
	extern char *__itoa_ncheck (int, char *, unsigned char);
	return __itoa_ncheck (__val, __s, __radix);
    }
}
#endif

/**
 \ingroup avr_stdlib

   \brief Convert a long integer to a string.

   The function ltoa() converts the long integer value from \c val into an
   ASCII representation that will be stored under \c s.  The caller
   is responsible for providing sufficient storage in \c s.

   \note The minimal size of the buffer \c s depends on the choice of
   radix. For example, if the radix is 2 (binary), you need to supply a buffer
   with a minimal length of 8 * sizeof (long int) + 1 characters, i.e. one
   character for each bit plus one for the string terminator. Using a larger
   radix will require a smaller minimal buffer size.

   \warning If the buffer is too small, you risk a buffer overflow.

   Conversion is done using the \c radix as base, which may be a
   number between 2 (binary conversion) and up to 36.  If \c radix
   is greater than 10, the next digit after \c '9' will be the letter
   \c 'a'.

   If radix is 10 and val is negative, a minus sign will be prepended.

   The ltoa() function returns the pointer passed as \c s.
*/
#ifdef  __DOXYGEN__
extern char *ltoa(long val, char *s, int radix);
#else
extern __inline__ __ATTR_GNU_INLINE__
char *ltoa (long __val, char *__s, int __radix)
{
    if (!__builtin_constant_p (__radix)) {
	extern char *__ltoa (long, char *, int);
	return __ltoa (__val, __s, __radix);
    } else if (__radix < 2 || __radix > 36) {
	*__s = 0;
	return __s;
    } else {
	extern char *__ltoa_ncheck (long, char *, unsigned char);
	return __ltoa_ncheck (__val, __s, __radix);
    }
}
#endif

/**
 \ingroup avr_stdlib

   \brief Convert an unsigned integer to a string.

   The function utoa() converts the unsigned integer value from \c val into an
   ASCII representation that will be stored under \c s.  The caller
   is responsible for providing sufficient storage in \c s.

   \note The minimal size of the buffer \c s depends on the choice of
   radix. For example, if the radix is 2 (binary), you need to supply a buffer
   with a minimal length of 8 * sizeof (unsigned int) + 1 characters, i.e. one
   character for each bit plus one for the string terminator. Using a larger
   radix will require a smaller minimal buffer size.

   \warning If the buffer is too small, you risk a buffer overflow.

   Conversion is done using the \c radix as base, which may be a
   number between 2 (binary conversion) and up to 36.  If \c radix
   is greater than 10, the next digit after \c '9' will be the letter
   \c 'a'.

   The utoa() function returns the pointer passed as \c s.
*/
#ifdef  __DOXYGEN__
extern char *utoa(unsigned int val, char *s, int radix);
#else
extern __inline__ __ATTR_GNU_INLINE__
char *utoa (unsigned int __val, char *__s, int __radix)
{
    if (!__builtin_constant_p (__radix)) {
	extern char *__utoa (unsigned int, char *, int);
	return __utoa (__val, __s, __radix);
    } else if (__radix < 2 || __radix > 36) {
	*__s = 0;
	return __s;
    } else {
	extern char *__utoa_ncheck (unsigned int, char *, unsigned char);
	return __utoa_ncheck (__val, __s, __radix);
    }
}
#endif

/**
 \ingroup avr_stdlib
   \brief Convert an unsigned long integer to a string.

   The function ultoa() converts the unsigned long integer value from
   \c val into an ASCII representation that will be stored under \c s.
   The caller is responsible for providing sufficient storage in \c s.

   \note The minimal size of the buffer \c s depends on the choice of
   radix. For example, if the radix is 2 (binary), you need to supply a buffer
   with a minimal length of 8 * sizeof (unsigned long int) + 1 characters,
   i.e. one character for each bit plus one for the string terminator. Using a
   larger radix will require a smaller minimal buffer size.

   \warning If the buffer is too small, you risk a buffer overflow.

   Conversion is done using the \c radix as base, which may be a
   number between 2 (binary conversion) and up to 36.  If \c radix
   is greater than 10, the next digit after \c '9' will be the letter
   \c 'a'.

   The ultoa() function returns the pointer passed as \c s.
*/
#ifdef  __DOXYGEN__
extern char *ultoa(unsigned long val, char *s, int radix);
#else
extern __inline__ __ATTR_GNU_INLINE__
char *ultoa (unsigned long __val, char *__s, int __radix)
{
    if (!__builtin_constant_p (__radix)) {
	extern char *__ultoa (unsigned long, char *, int);
	return __ultoa (__val, __s, __radix);
    } else if (__radix < 2 || __radix > 36) {
	*__s = 0;
	return __s;
    } else {
	extern char *__ultoa_ncheck (unsigned long, char *, unsigned char);
	return __ultoa_ncheck (__val, __s, __radix);
    }
}
#endif

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLER */

#endif /* _STDLIB_ITOA_H_ */

#endif // __GNUC__
