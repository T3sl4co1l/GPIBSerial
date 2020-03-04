//#include "main.h"
#include "ccpwrite.h"
#include <inttypes.h>
#include <avr/interrupt.h>

#ifdef __cplusplus
extern "C" {
#endif


void CCPWrite (volatile uint8_t *address, uint8_t value)
{
    ENTER_CRITICAL_REGION( );
#ifdef __ICCAVR__

    asm("movw r30, r16");
#ifdef RAMPZ
    RAMPZ = 0;
#endif
    asm("ldi  r16,  0xD8 \n"
        "out  0x34, r16  \n"
#if (__MEMORY_MODEL__ == 1)
        "st     Z,  r17  \n");
#elif (__MEMORY_MODEL__ == 2)
        "st     Z,  r18  \n");
#else /* (__MEMORY_MODEL__ == 3) || (__MEMORY_MODEL__ == 5) */
        "st     Z,  r19  \n");
#endif /* __MEMORY_MODEL__ */

#elif defined __GNUC__
    volatile uint8_t * tmpAddr = address;
#ifdef RAMPZ
    RAMPZ = 0;
#endif
    asm volatile(
        "movw r30,  %0"       "\n\t"
        "ldi  r16,  %2"       "\n\t"
        "out   %3, r16"       "\n\t"
        "st     Z,  %1"       "\n\t"
        :
        : "r" (tmpAddr), "r" (value), "M" (CCP_IOREG_gc), "i" (&CCP)
        : "r16", "r30", "r31"
        );

#endif
    LEAVE_CRITICAL_REGION( );
}


#ifdef __cplusplus
}
#endif
