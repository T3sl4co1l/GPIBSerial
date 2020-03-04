// This file has been prepared for Doxygen automatic documentation generation.
//#include "avr_compiler.h"

#ifndef CCPWRITE_H
#define CCPWRITE_H

#include <stdint.h>

void CCPWrite (volatile uint8_t *address, uint8_t value);

#define ENTER_CRITICAL_REGION() uint8_t volatile saved_sreg = SREG; \
                                     cli();

#define LEAVE_CRITICAL_REGION() SREG = saved_sreg;

#endif // CCPWRITE_H
