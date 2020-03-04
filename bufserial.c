#include "bufserial.h"

/** Serial buffers */
uint8_t serRxBuffer[SER_BUF_SIZE];
uint8_t serTxBuffer[SER_BUF_SIZE];
/** Index into rx/txBuffer.  Start = where to enqueue new bytes; End = byte currently being sent. */
volatile uint8_t serRxBufferStart = 0, serRxBufferEnd = 0;
volatile uint8_t serTxBufferStart = 0, serTxBufferEnd = 0;

/**
 *	Set up all the registers needed for operation, flush buffers
 */
void serInit(void) {

	serTxBufferStart = 0;
	serTxBufferEnd = 0;
	serRxBufferStart = 0;
	serRxBufferEnd = 0;

	//	Initialize async serial port
	SER_PORT_TXD_EN();
	SER_PORT.CTRLA = 0;
	SER_PORT.CTRLB = USART_RXEN_bm | USART_TXEN_bm;
	SER_PORT.CTRLC = USART_CHSIZE_8BIT_gc;
	//	115.2k baud
//	SER_PORT.BAUDCTRLA = (2094 & 0xff) << USART_BSEL_gp;
//	SER_PORT.BAUDCTRLB = ((-7) << USART_BSCALE_gp) | ((2094 >> 8) << USART_BSEL_gp);
	//	230.4k baud
	SER_PORT.BAUDCTRLA = (983 & 0xff) << USART_BSEL_gp;
	SER_PORT.BAUDCTRLB = ((-7) << USART_BSCALE_gp) | ((983 >> 8) << USART_BSEL_gp);
	SER_ENABLE_RXC();

	//	Enable interrupt sources
	PMIC.CTRL |= PMIC_LOLVLEN_bm;
	sei();

	while (!serRxBufferEmpty()) {
		serGetByte();
	}

}

/**
 *	Purge the transmit buffer.
 */
void serWaitTransmit(void) {
	while (!serTxBufferEmpty()) {}
}

/**
 *	Retrieves a byte from the receive queue.
 *	@return	byte dequeued, or zero if empty.
 *	(Check rxBufferEmpty() before executing to ensure no ambiguity.)
 */
uint8_t serGetByte(void) {

	uint8_t r = 0;

	//	Disable interrupt for atomic access
	SER_DISABLE_RXC();
	if (!serRxBufferEmpty()) {
		//	More bytes to go, pop off the next one
		r = serRxBuffer[serRxBufferEnd++];
		if (serRxBufferEnd >= SER_BUF_SIZE) {
			serRxBufferEnd = 0;
		}
	}

	SER_ENABLE_RXC();
	return r;
}

/**
 *	Places a byte into the transmit queue, and starts
 *	the transmitter if it wasn't already running.
 *	@param	b	byte to enqueue
 *	@return	number of bytes not queued:
 *	false if successful, true if buffer full.
 */
bool serPutByte(uint8_t b) {

	bool r = false;

	//	Disable interrupt for atomic access
	SER_DISABLE_DRE();

	if (serTxBufferFull()) {
		//	no room, just leave
		r = true;
	} else {
		//	enqueue the byte
		serTxBuffer[serTxBufferStart++] = b;
		if (serTxBufferStart >= SER_BUF_SIZE) {
			serTxBufferStart = 0;
		}
	}
	SER_ENABLE_DRE();
	return r;
}

/**
 *	Places an ASCII formatted hexadecimal number into the transmit
 *	queue, and starts the transmitter if it wasn't already running.
 *	@param	n	number to transmit
 *	@return	number of bytes enqueued
 */
uint8_t serPutNumHex(uint16_t n) {

	uint8_t l, tmpBuf[8];

	utoa(n, (char*)tmpBuf, 16);
	l = strlen((char*)tmpBuf);
	return l - serPutString(l, tmpBuf);
}

/**
 *	Places an ASCII formatted hexadecimal number into the transmit
 *	queue, and starts the transmitter if it wasn't already running.
 *	Fixed length (padded with leading zeroes if applicable).
 *	@param	n	number to transmit
 *	@return	number of bytes enqueued
 */
uint8_t serPutNumHexFixed_w(uint16_t n) {

	uint8_t b, i;

	b = n >> 8;
	for (i = 0; i < 4; i++) {
		if (i == 2) {
			b = n;
		}
		b = b >> 4 | b << 4;
		serPutByte(nibbleToAscii(b));
	}
	return i;
}

/**
 *	Places an ASCII formatted hexadecimal number into the transmit
 *	queue, and starts the transmitter if it wasn't already running.
 *	Fixed length (padded with leading zeroes if applicable).
 *	@param	n	number to transmit
 *	@return	number of bytes enqueued
 */
uint8_t serPutNumHexFixed_b(uint8_t n) {

	return serPutByte(nibbleToAscii(n >> 4 | n << 4))
			+ serPutByte(nibbleToAscii(n));
}

/**
 *	Places an ASCII formatted decimal number into the transmit
 *	queue, and starts the transmitter if it wasn't already running.
 *	@param	n	number to transmit
 *	@return	number of bytes enqueued
 */
uint8_t serPutNumDec(uint16_t n) {

	uint8_t l, tmpBuf[8];

	utoa(n, (char*)tmpBuf, 10);
	l = strlen((char*)tmpBuf);
	return l - serPutString(l, tmpBuf);
}

/**
 *	Places an ASCII formatted decimal number into the transmit
 *	queue, and starts the transmitter if it wasn't already running.
 *	@param	n	number to transmit
 *	@return	number of bytes enqueued
 */
uint8_t serPutNumDecL(uint32_t n) {

	uint8_t l, tmpBuf[12];

	ultoa(n, (char*)tmpBuf, 10);
	l = strlen((char*)tmpBuf);
	return l - serPutString(l, tmpBuf);
}

/**
 *	Copies a buffer into the transmit queue, and starts
 *	the transmitter if it wasn't already running.
 *	Queuing starts from the beginning of buf and ends at
 *	one less than buf + len.
 *	@param	len		number of bytes
 *	@param	buf		buffer to enqueue
 *	@return	number of bytes remaining in buf.  Zero when successful.
 */
uint8_t serPutString(uint8_t len, uint8_t* buf) {

	uint8_t i;

	//	Disable interrupt for atomic access
	SER_DISABLE_DRE();
	for (i = 0; len; i++, len--) {
		if (serPutByte(buf[i])) {
			break;
		}
//		if (serTxBufferFull()) {
//			//	no room, just leave
//			break;
//		} else {
//			//	enqueue the byte
//			serTxBuffer[serTxBufferStart++] = buf[i];
//			if (serTxBufferStart >= SER_BUF_SIZE) {
//				serTxBufferStart = 0;
//			}
//		}
	}
	SER_ENABLE_DRE();
	return len;
}

/**
 *	Copies a PROGMEM string into the transmit queue, and starts
 *	the transmitter if it wasn't already running.
 *	@param	s	PROGMEM string to transmit
 *	@return	number of bytes written (compare with strlen_P(s)),
 *			up to a maximum of 255 bytes.
 */
uint8_t serPutStringPgm(const uint8_t* s) {

	uint8_t l = strlen_P((char*)s);
	return l - serPutStringPgmL(l, s);
}

/**
 *	Copies a PROGMEM string into the transmit queue, and starts
 *	the transmitter if it wasn't already running.
 *	Stops if the transmit buffer is full; in that case, the
 *	return is nonzero.
 *	@param	len	Length of the string to transmit
 *	@param	s	PROGMEM string to transmit
 *	@return	number of bytes remaining.
 */
uint8_t serPutStringPgmL(uint8_t len, const uint8_t* s) {

	//	Disable interrupt for atomic access
	SER_DISABLE_DRE();
	while (len--) {
		if (serTxBufferFull()) {
			//	no room, just leave
			break;
		}
		//	enqueue the byte
		serTxBuffer[serTxBufferStart++] = pgm_read_byte(s++);
		if (serTxBufferStart >= SER_BUF_SIZE) {
			serTxBufferStart = 0;
		}
	}
	SER_ENABLE_DRE();
	return len;
}

/**
 *	Checks if the serial receive buffer is full.
 */
inline bool serRxBufferFull() {

	uint8_t s = serRxBufferStart + 1;

	if (s > SER_BUF_SIZE) {
		s = 0;
	}
	return s == serRxBufferEnd;
}

/**
 *	Checks if the serial receive buffer is empty.
 */
inline bool serRxBufferEmpty() {
	return serRxBufferStart == serRxBufferEnd;
}

/**
 *	Checks if the serial transmit buffer is full.
 */
inline bool serTxBufferFull(void) {

	uint8_t s = serTxBufferStart + 1;

	if (s > SER_BUF_SIZE) {
		s = 0;
	}
	return s == serTxBufferEnd;
}

/**
 *	Checks if the serial transmit buffer is empty.
 */
inline bool serTxBufferEmpty(void) {
	return serTxBufferStart == serTxBufferEnd;
}

/**
 *	Converts the low 4 bits of c to ASCII coded hexadecimal.
 */
uint8_t nibbleToAscii(uint8_t c) {

	c &= 0x0f;
	if (c >= 10) {
		c += 'A' - 10;
	} else {
		c += '0';
	}
	return c;
}


/* * *  Interrupt Handlers  * * */

/**
 *	Asynchronous serial port buffered receive handler
 */
ISR(SER_PORT_RXC_v) {

	if (serRxBufferFull()) {
		//	Can't do much but refuse additional inputs
		SER_DISABLE_RXC();
		//if (serAsciiMode) {
		//	serPutByte('\b');	//	Ding on buffer overflow
		//}

	} else {
		//	enqueue the byte
		serRxBuffer[serRxBufferStart++] = SER_PORT.DATA;
		if (serRxBufferStart >= SER_BUF_SIZE) {
			serRxBufferStart = 0;
		}
	}

}

/**
 *	Asynchronous serial port buffered transmit handler
 */
ISR(SER_PORT_DRE_v) {

	if (!serTxBufferEmpty()) {
		//	More bytes to go, pop off the next one
		SER_PORT.DATA = serTxBuffer[serTxBufferEnd++];
		if (serTxBufferEnd >= SER_BUF_SIZE) {
			serTxBufferEnd = 0;
		}
	} else {
		//	Done, turn off interrupt
		SER_DISABLE_DRE();
	}

}
