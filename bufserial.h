#ifndef BUFSERIAL_H_INCLUDED
#define BUFSERIAL_H_INCLUDED

#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#define STRING_AND_LENGTH(x)	sizeof(x) - 1, (uint8_t*)x
#define BUFFER_AND_LENGTH(x)	strlen((char*)x), (uint8_t*)x

#define SER_BUF_SIZE		64
#define SER_PORT			USARTC0
#define SER_PORT_RXC_v		USARTC0_RXC_vect
#define SER_PORT_DRE_v		USARTC0_DRE_vect
#define SER_PORT_TXD_EN()	PORTC.DIRSET = PIN3_bm, PORTC.DIRCLR = PIN2_bm;
#define SER_ENABLE_DRE()	SER_PORT.CTRLA = (SER_PORT.CTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_LO_gc
#define SER_DISABLE_DRE()	SER_PORT.CTRLA &= ~USART_DREINTLVL_gm
#define SER_ENABLE_RXC()	SER_PORT.CTRLA = (SER_PORT.CTRLA & ~USART_RXCINTLVL_gm) | USART_RXCINTLVL_LO_gc
#define SER_DISABLE_RXC()	SER_PORT.CTRLA &= ~USART_RXCINTLVL_gm
#define SER_TX_READY		( (SER_PORT.STATUS & USART_DREIF_bm) == USART_DREIF_bm )
#define SER_RX_READY		( (SER_PORT.STATUS & USART_RXCIF_bm) == USART_RXCIF_bm )
#define SER_RX_OVF			( (SER_PORT.STATUS & USART_BUFOVF_bm) == USART_BUFOVF_bm )

//#define serWaitTransmit()	while (!serTxBufferEmpty()) {}


void    serInit(void);
void    serWaitTransmit(void);
uint8_t serGetByte(void);
bool    serPutByte(uint8_t b);
uint8_t serPutNumHex(uint16_t n);
uint8_t serPutNumHexFixed_w(uint16_t n);
uint8_t serPutNumHexFixed_b(uint8_t n);
uint8_t serPutNumDec(uint16_t n);
uint8_t serPutNumDecL(uint32_t n);
uint8_t serPutString(uint8_t len, uint8_t* s);
uint8_t serPutStringPgm(const uint8_t* s);
uint8_t serPutStringPgmL(uint8_t len, const uint8_t* s);
bool    serRxBufferFull(void);
bool    serRxBufferEmpty(void);
bool    serTxBufferFull(void);
bool    serTxBufferEmpty(void);
uint8_t nibbleToAscii(uint8_t c);

#endif // BUFSERIAL_H_INCLUDED
