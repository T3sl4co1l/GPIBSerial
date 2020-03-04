/*
 *	AVR GPIB-Serial Adapter
 *	Tim Williams, 1-10-2020
 *
 *	Adapted from GPIBUSB Adapter © 2014 Steven Casagrande
 *	(AGPL version 3 license).
 *
 *	Primary changes:
 *	- Formatting, comments updated
 *	- Porting PIC stuff to AVR and in-house stuff
 *
 *	Compile for MCU=atxmega64d3, CPU_CORE=x64d3
 *
 *	ATXMEGA64D3-U -- Pin Assignments
 *
 *	Pin     Func.	GPIB Pin	Description
 *	---------------------------------------
 *	PD0		GPIO	1			DIO1
 *	PD1		GPIO	2			DIO2
 *	PD2		GPIO	3			DIO3
 *	PD3		GPIO	4			DIO4
 *	PD4		GPIO	13			DIO5
 *	PD5		GPIO	14			DIO6
 *	PD6		GPIO	15			DIO7
 *	PD7		GPIO	16			DIO8
 *	PE0		GPIO	17			REN
 *	PE1		GPIO	5			EOI
 *	PE2		GPIO	6			DAV
 *	PE3		GPIO	7			NRFD
 *	PE4		GPIO	8			NDAC
 *	PE5		GPIO	11			ATN
 *	PE6		GPIO	9			IFC
 *	PE7		GPIO	10			SRQ
 *	GND		GND		12			SHIELD
 *	PR0		XTAL1				4MHz crystal, 27pF loading capacitors
 *	PR1		XTAL2				4MHz crystal, 27pF loading capacitors
 *	NRST	NRST				Programming port
 *	PDI		PDI					Programming port
 *	PB3		GPIO				Heartbeat LED
 *	PB5		GPIO				Test point
 *	PC2		RXD0				Serial port
 *	PC3		TXD0				Serial port
 *	PC4		SC					System control*
 *	PC5		TE					Talk enable*
 *	PC6		PE					Pull-up enable*
 *	PC7		DC					Direction control*
 *
 *	*Connect to control signals for GPIB interface ICs, if used.
 *
 *	Most pins are assigned in main.h, except the serial port which
 *	is assigned in bufserial.h.
 */

#include "main.h"
//	Redundant hack to get Code::Blocks to pick up header references
#ifndef _AVR_ATxmega64D3_H_
#include <avr/iox64d3.h>
#endif // _AVR_ATxmega64D3_H_


/* * *  Fuse Settings  * * */

FUSES = {
	FUSE0_DEFAULT,
	FUSE1_DEFAULT,
	FUSE2_DEFAULT,
	0,	//	reserved
	FUSE4_DEFAULT,
	FUSE5_DEFAULT
};


/* * *  Global Variables  * * */

/**	By default, we are using EOI to signal end
 *	of msg from instrument  */
bool eoiUse = true;
/**	Enable read/write status, error messages  */
bool debug = false;
bool autoRead = true;
bool eot_enable = true;
bool listen_only = false;
bool mode = true;
bool save_cfg = true;

const uint8_t version = 6;	//	Incremented to 6 -- TMW
uint8_t partnerAddress = 1;
uint8_t myAddress;
uint8_t strip = 0;
uint8_t eos_code = 3;
/**	Default end of string character  */
uint8_t eos = 10;
/**	Default CR  */
uint8_t eot_char = 13;
uint8_t status_byte = 0;
uint32_t timeoutMillis = 1000;

volatile uint32_t timerMillis = 0;
volatile bool timerExpired = true;

uint8_t eos_string[3];
uint8_t cmd_buf[10];
uint8_t buf[256];
uint8_t bufLength = 0;

// Variables for device mode
bool device_talk = false;
bool device_listen = false;
bool device_srq = false;

//	EEPROM variables
const eeprom_settings_t ConfigData EEMEM = {
	.valEepCode		= VALID_EEPROM_CODE,
	.mode			= 1,
	.partnerAddr	= 1,
	.eotChar		= 13,
	.eotEn			= 1,
	.eosCode		= 3,
	.eoiUse			= 1,
	.aRead			= 1,
	.lstnOnl		= 0,
	.saveCfg		= 1
};

#define WITH_TIMEOUT
#define WITH_WDT
#define VERBOSE_DEBUG


/* * *  Functions  * * */

/**
 *	Main entry point.
 */
int main(void) {

	initialize();
	serInit();

	// Turn on the error LED
	PORT_TP.OUTSET = BIT_TP;

	PORT_TP.OUTCLR = BIT_TP;

//	setCtrlMode(0);
//	startTimer(500);
//	while (1) {
//		if (timerExpired) {
//			startTimer(500);
//			PORT_HB.OUTTGL = BIT_HB;
//			while (!serRxBufferEmpty()) {
//				serGetByte();
//			}
//			serPutString(STRING_AND_LENGTH("foo\n\r"));
//		}
//	}

	pic_main();

	return 0;
}

/**
 *	Main initialization.  Sets port and clock configurations.
 */
void initialize(void) {

	/*	XMEGA-specific stuff is formatted natively; adopted code is
	 *	left more or less verbatim, using macros for compatibility.  */

	//	Set debug pin states
	PORT_HB.DIRSET = BIT_HB;
	PORT_TP.DIRSET = BIT_TP;

	//	Set up external oscillator
	//	XOSCSEL to external, PLLSRC to XOSC, SCLKSEL to PLL,
	//	PLL multiplier 4x, system prescalers to 1x (default)
	OSC.XOSCCTRL = OSC_FRQRANGE_2TO9_gc | OSC_XOSCSEL_XTAL_16KCLK_gc;
	OSC.CTRL |= OSC_XOSCEN_bm;
	//	Set PLL to XOSC, no divider, 8x (= 32MHz); wait for ref startup
	OSC.PLLCTRL = OSC_PLLSRC_XOSC_gc | (8 << OSC_PLLFAC_gp);
	//	Set then clear HB to confirm clock start
	PORT_HB.OUTSET = BIT_HB;
	while(!(OSC.STATUS & OSC_XOSCRDY_bm));
	//	Turn on PLL; wait for PLL startup
	OSC.CTRL |= OSC_PLLEN_bm;
	while(!(OSC.STATUS & OSC_PLLRDY_bm));
	PORT_HB.OUTCLR = BIT_HB;
	//	Select PLL source
	CCPWrite(&(CLK.CTRL), CLK_SCLKSEL_PLL_gc);

	//	Initialize TCC0, 16 bits, 1ms interrupt
	TCC0.PER = (F_CPU * 0.001f);
	TCC0.CTRLB = TC0_CCAEN_bm | TC_WGMODE_SS_gc;
	TCC0.CTRLA = TC_CLKSEL_DIV1_gc;
	TCC0.CCA = 0;
	stopTimer();

	//	Init WDT

	// Flow port pins will always be outputs
	PORT_TE.OUTCLR = BIT_TE; PORT_TE.DIRSET = BIT_TE;
	PORT_PE.OUTCLR = BIT_PE; PORT_PE.DIRSET = BIT_PE;
	setSCDCMode(mode);
	PORT_SC.DIRSET = BIT_SC;
	PORT_DC.DIRSET = BIT_DC;

	//	Set pullups on all bus pins
	//	Note: assumes PINnCTRL registers are in order
	for (uint8_t i = 0; i < 8; i++) {
		((uint8_t*)&PORT_DIO.PIN0CTRL)[i] = PORT_OPC_PULLUP_gc;
	}
#ifdef PORT_CTRL
	for (uint8_t i = 0; i < 8; i++) {
		((uint8_t*)&PORT_CTRL.PIN0CTRL)[i] = PORT_OPC_PULLUP_gc;
	}
#else
	((uint8_t*)&PORT_ATN.PIN0CTRL)[PIN_ATN] = PORT_OPC_PULLUP_gc;
	((uint8_t*)&PORT_EOI.PIN0CTRL)[PIN_EOI] = PORT_OPC_PULLUP_gc;
	((uint8_t*)&PORT_DAV.PIN0CTRL)[PIN_DAV] = PORT_OPC_PULLUP_gc;
	((uint8_t*)&PORT_NRFD.PIN0CTRL)[PIN_NRFD] = PORT_OPC_PULLUP_gc;
	((uint8_t*)&PORT_NDAC.PIN0CTRL)[PIN_NDAC] = PORT_OPC_PULLUP_gc;
	((uint8_t*)&PORT_IFC.PIN0CTRL)[PIN_IFC] = PORT_OPC_PULLUP_gc;
	((uint8_t*)&PORT_SRQ.PIN0CTRL)[PIN_SRQ] = PORT_OPC_PULLUP_gc;
	((uint8_t*)&PORT_REN.PIN0CTRL)[PIN_REN] = PORT_OPC_PULLUP_gc;
#endif // PORT_CTRL

	// Float all DIO lines
	DeassertDio();
	WriteDio(0xff);

	// Set mode and pin state for all GPIB control lines
	setCtrlMode(mode);

	//	Enable interrupt sources
	PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	sei();

}

/**
 *	Starts the countdown timer.
 *	Poll timerExpired to check when timer has finished.
 *	@param time		milliseconds to count.
 */
void startTimer(uint32_t time) {
	disableTimerInt();
	timerMillis = time;
	timerExpired = false;
	TCC0.CNT = 0;
	enableTimerInt();
}

/**
 *	Timer interrupt handler.
 */
ISR(TCC0_OVF_vect) {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		timerMillis--;
		if (timerMillis == 0) {
			disableTimerInt();
			timerExpired = true;
		}
	}
}

/**
 *	Set GPIB interface direction and talk signals.
 */
void setSCDCMode(bool mode) {

	if (mode) {
	    PORT_SC.OUTSET = BIT_SC;	//	TX on REN and IFC
	    PORT_DC.OUTCLR = BIT_DC;	//	TX on ATN and RX on SRQ
	} else {
	    PORT_SC.OUTCLR = BIT_SC;
	    PORT_DC.OUTSET = BIT_DC;
	}

}

/**
 *	Set GPIB control mode.
 */
void setCtrlMode(bool mode) {

	if (mode) {
		//	Simple optimization assuming all CTRL signals are on the same port
#ifdef PORT_CTRL
		PORT_CTRL.OUT = BIT_ATN | BIT_EOI | BIT_DAV /* | BIT_NRFD */
				/* | BIT_NDAC */ | BIT_IFC | BIT_SRQ /* | BIT_REN */;
		PORT_CTRL.DIR = BIT_ATN /* | BIT_EOI */ /* | BIT_DAV */ | BIT_NRFD
				| BIT_NDAC | BIT_IFC /* | BIT_SRQ */ | BIT_REN;
#else
	    PORT_ATN.OUTSET = BIT_ATN;
	    PORT_ATN.DIRSET = BIT_ATN;
	    PORT_EOI.DIRCLR = BIT_EOI;
	    //PORT_EOI.OUTSET = BIT_EOI;
	    PORT_DAV.DIRCLR = BIT_DAV;
	    //PORT_DAV.OUTSET = BIT_DAV;
	    PORT_NRFD.OUTCLR = BIT_NRFD;
	    PORT_NRFD.DIRSET = BIT_NRFD;
	    PORT_NDAC.OUTCLR = BIT_NDAC;
	    PORT_NDAC.DIRSET = BIT_NDAC;
	    PORT_IFC.OUTSET = BIT_IFC;
	    PORT_IFC.DIRSET = BIT_IFC;
	    PORT_SRQ.DIRCLR = BIT_SRQ;
	    //PORT_SRQ.OUTSET = BIT_SRQ;
	    PORT_REN.OUTCLR = BIT_REN;
	    PORT_REN.DIRSET = BIT_REN;
#endif
	} else {
#ifdef PORT_CTRL
		PORT_CTRL.DIR = 0;
		PORT_CTRL.OUT = BIT_ATN | BIT_EOI | BIT_DAV | BIT_NRFD
				| BIT_NDAC | BIT_IFC | BIT_SRQ | BIT_REN;
#else
	    PORT_ATN.DIRCLR = BIT_ATN;
	    //PORT_ATN.OUTSET = BIT_ATN;
	    PORT_EOI.DIRCLR = BIT_EOI;
	    //PORT_EOI.OUTSET = BIT_EOI;
	    PORT_DAV.DIRCLR = BIT_DAV;
	    //PORT_DAV.OUTSET = BIT_DAV;
	    PORT_NRFD.DIRCLR = BIT_NRFD;
	    //PORT_NRFD.OUTSET = BIT_NRFD;
	    PORT_NDAC.DIRCLR = BIT_NDAC;
	    //PORT_NDAC.OUTSET = BIT_NDAC;
	    PORT_IFC.DIRCLR = BIT_IFC;
	    //PORT_IFC.OUTSET = BIT_IFC;
	    PORT_SRQ.DIRCLR = BIT_SRQ;
	    //PORT_SRQ.OUTSET = BIT_SRQ;
	    PORT_REN.DIRCLR = BIT_REN;
	    //PORT_REN.OUTSET = BIT_REN;
#endif
	}
}

/**
 *	Puts all the GPIB pins into their correct initial states.
 */
void initGpibPins() {

	//	Disable talking on data and handshake lines
	TalkDisable();
	PullupDisable();

	setSCDCMode(mode);
	DeassertDio();
	setCtrlMode(mode);

}

/**
 *	Assign GPIB controller address.
 */
bool assignGpibController(uint8_t address) {
	myAddress = address;

	AssertIfc();	//	Assert interface clear.  Resets bus and makes it
					//	controller in charge.
	startTimer(200);
	while (!timerExpired) {
		restart_wdt();
	};
	DeassertIfc();	// Finishing clearing interface

	AssertRen();	// Put all connected devices into "remote" mode
	cmd_buf[0] = CMD_DCL;
	return gpib_cmd(cmd_buf, 1);	//	Send GPIB DCL cmd, clear all devices on bus
}

/**
 *	Write GPIB CMD bytes to the bus.
 */
bool gpib_cmd(const uint8_t* bytes, uint8_t length) {
	return _gpib_write(bytes, length, true, false);
}

/**
 *	Write a GPIB CMD byte to the bus.
 */
uint8_t gpib_cmd_b(uint8_t b) {
	uint8_t buf[2];
	buf[0] = b; buf[1] = 0;
	return _gpib_write(buf, 1, true, false);
}

/**
 *	Write a GPIB data string to the bus.
 */
bool gpib_write(const uint8_t* bytes, uint8_t length, bool useEOI) {
	return _gpib_write(bytes, length, false, useEOI);
}

/**
 *	Temporary stub
 */
void restart_wdt(void) {
	return;
}

/**
 *	Write a string of bytes to the bus.
 *	@param
 *	bytes: array containing characters to be written
 *	length: number of bytes to write, 0 if unknown
 *	attention: 1 if this is a GPIB command, 0 if data
 *	@return
 *	true on error
 */
bool _gpib_write(const uint8_t* bytes, uint8_t length, bool attention, bool useEOI) {
	uint8_t i;

	PullupEnable();
	//	If byte is a GPIB bus command
	if (attention) {
		// Assert the ATN line, informing all this is a cmd byte
		AssertAtn();
	}
	//	If the length was unknown
	if (length == 0) {
		//	Calculate the number of bytes to be sent
		length = strlen((char*)bytes);
	}

	//	Enable talking
	TalkEnable();
	//DeassertEoi();
	RaiseEoi();
	//DeassertDav();
	RaiseDav();
	DeassertNrfd();
	DeassertNdac();

	//	Before we start transferring, make sure NRFD is high and NDAC is low
#ifdef WITH_TIMEOUT
	startTimer(timeoutMillis);
	while (ReadNdac() || !ReadNrfd()) {
		restart_wdt();
		if (timerExpired) {
			if (debug) {
				serPutString(STRING_AND_LENGTH("Timeout: Before writing "));
				serPutNumDec(bytes[0]);
				serPutString(STRING_AND_LENGTH(" 0x"));
				serPutNumHexFixed_b(bytes[0]);
				serPutByte(eot_char);
			}
			device_talk = false; device_srq = false;
			initGpibPins();
			return true;
		}
	}
#else
	while (ReadNdac() || !ReadNrfd();
#endif
	stopTimer();

	//	Loop through each character, write to bus
	for (i = 0; i < length; i++) {

#ifdef VERBOSE_DEBUG
		serPutString(STRING_AND_LENGTH("Writing byte: "));
		serPutNumDec(bytes[i]);
		serPutString(STRING_AND_LENGTH(" 0x"));
		serPutNumHexFixed_b(bytes[i]);
		serPutByte(eot_char);
#endif

		//	Wait for NDAC to go low, indicating previous bit is now done with
#ifdef WITH_TIMEOUT
		startTimer(timeoutMillis);
		while(ReadNdac()) {
			restart_wdt();
			if (timerExpired) {
				if (debug) {
					serPutString(STRING_AND_LENGTH("Timeout: Waiting for NDAC to go low while writing"));
					serPutByte(eot_char);
				}
				device_talk = false; device_srq = false;
				initGpibPins();
				return true;
			}
		}
#else
		while (ReadNdac());
#endif
		stopTimer();

		// Put the byte on the data lines
		WriteDio(~bytes[i]);
		AssertDio();
		DeassertNrfd();

		// Wait for listeners to be ready for data (NRFD should be high)
#ifdef WITH_TIMEOUT
		startTimer(timeoutMillis);
		while(!ReadNrfd()) {
			restart_wdt();
			if (timerExpired) {
				if (debug) {
					serPutString(STRING_AND_LENGTH("Timeout: Waiting for NRFD to go high while writing"));
					serPutByte(eot_char);
				}
				device_talk = false; device_srq = false;
				initGpibPins();
				return true;
			}
		}
#else
		while (ReadNrfd());
#endif
		stopTimer();

		//	If last byte in string, assert EOI
		if (i == length - 1 && useEOI) {
			AssertEoi();
		}
		//	Inform listeners that the data is ready to be read
		AssertDav();

		// Wait for NDAC to go high, all listeners have accepted the byte
#ifdef WITH_TIMEOUT
		startTimer(timeoutMillis);
		while(!ReadNdac()) {
			restart_wdt();
			if (timerExpired) {
				if (debug) {
					serPutString(STRING_AND_LENGTH("Timeout: Waiting for NDAC to go high while writing"));
					serPutByte(eot_char);
				}
				device_talk = false; device_srq = false;
				initGpibPins();
				return true;
			}
		}
#else
		while (!ReadNdac());
#endif
		stopTimer();
		//	Byte has been accepted by all, indicate byte is no longer valid
		RaiseDav();

	}
	//	Finished outputting all bytes to listeners, stop talking
	TalkDisable();
	// Float all data lines
	DeassertDio();
	DeassertAtn();
	DeassertDav();
	DeassertEoi();
	DeassertNdac();
	//RaiseNdac();
	DeassertNrfd();
	//RaiseNrfd();
	PullupDisable();

	return false;

}

/**
 *	Receives a data byte from the GPIB bus.
 *	If a timeout occurs, the byte may not be valid.
 *	@param byt	Pointer to buffer receiving into.
 *	@return 0 when successful, 1 when EOI, 0xff when timeout.
 */
uint8_t gpib_receive(uint8_t* byt) {
	uint8_t eoiStatus;	//	Returns 0 or 1 depending on status of EOI line

	//	Raise NRFD, telling the talker we are ready for the byte
	//DeassertNrfd();
	RaiseNrfd();

	//	Assert NDAC informing the talker we have not accepted the byte yet
	AssertNdac();
	DeassertDav();

	//	Wait for DAV to go low (talker informing us the byte is ready)
#ifdef WITH_TIMEOUT
	startTimer(timeoutMillis);
	while(ReadDav()) {
		restart_wdt();
		if (timerExpired) {
			if (debug) {
				serPutString(STRING_AND_LENGTH("Timeout: Waiting for DAV to go low while reading"));
				serPutByte(eot_char);
			}
			device_listen = false;
			initGpibPins();
			return 0xff;
		}
	}
#else
	while (ReadDav());
#endif
	stopTimer();

	//	Assert NRFD, informing talker to not change the data lines
	AssertNrfd();

	//	Read port B, where the data lines are connected
	*byt = ~ReadDio();
	eoiStatus = ReadEoi();

#ifdef VERBOSE_DEBUG
	serPutString(STRING_AND_LENGTH("Got byte: "));
	serPutNumDec(*byt);
	serPutString(STRING_AND_LENGTH(" 0x"));
	serPutNumHexFixed_b(*byt);
	serPutByte(eot_char);
#endif

	//	Un-assert NDAC, informing talker that we have accepted the byte
	DeassertNdac();

	//	Wait for DAV to go high (talker knows that we have read the byte)
#ifdef WITH_TIMEOUT
	startTimer(timeoutMillis);
	while (!ReadDav()) {
		restart_wdt();
		if (timerExpired) {
			if (debug) {
				serPutString(STRING_AND_LENGTH("Timeout: Waiting for DAV to go high while reading"));
				serPutByte(eot_char);
			}
			device_listen = false;
			initGpibPins();
			return 0xff;
		}
	}
#else
	while (!ReadDav());
#endif
	stopTimer();

	//	Prep for next byte, we have not accepted anything
	AssertNdac();
#ifdef VERBOSE_DEBUG
	serPutString(STRING_AND_LENGTH("EOI: "));
	serPutNumDec(eoiStatus);
	serPutByte(eot_char);
#endif
	return eoiStatus;
}

#define READ_BUFFER_SIZE	64

bool gpib_read(bool read_until_eoi) {
	uint8_t readCharacter;
	uint8_t readChars = 0;
	uint8_t eoiStatus;
	uint8_t readBuf[READ_BUFFER_SIZE];
	uint8_t* bufPnt = readBuf;
	bool errorFound = false;
	bool reading_done = false;

#ifdef VERBOSE_DEBUG
	serPutString(STRING_AND_LENGTH("gpib_read start"));
	serPutByte(eot_char);
#endif

	if (mode) {
		//	Command all talkers and listeners to stop
		cmd_buf[0] = CMD_UNT;
		errorFound = gpib_cmd(cmd_buf, 1);
		cmd_buf[0] = CMD_UNL;
		errorFound = errorFound || gpib_cmd(cmd_buf, 1);
		if (errorFound) {
			return true;
		}

		//	Set the controller into listener mode
		cmd_buf[0] = myAddress + 0x20;
		errorFound = gpib_cmd(cmd_buf, 1);
		if (errorFound) {
			return true;
		}

		//	Set target device into talker mode
		cmd_buf[0] = partnerAddress + 0x40;
		errorFound = gpib_cmd(cmd_buf, 1);
		if (errorFound) {
			return true;
		}
	}

#ifdef VERBOSE_DEBUG
	serPutString(STRING_AND_LENGTH("gpib_read loop start"));
	serPutByte(eot_char);
#endif
	if (read_until_eoi) {

		do {
			//	eoiStatus is line level
			eoiStatus = gpib_receive(&readCharacter);
			if (eoiStatus == 0xff) {
				return true;
			}
			if (eos_code != 0) {
				//	Check for EOM char
				if ((readCharacter != eos_string[0]) || eoiStatus) {
					//	Copy the read char into the buffer
					readBuf[readChars++] = readCharacter;
				}
			} else {
				if (readCharacter == eos_string[1] && !eoiStatus) {
					if (readBuf[readChars - 1] == eos_string[0]) {
						readChars--;
					}
				} else {
					readBuf[readChars++] = readCharacter;
				}
			}
			if (readChars/* == READ_BUFFER_SIZE - 4*/) {
				while (!serTxBufferEmpty());	//	Purge serial buffer before proceeding
				//serPutString(READ_BUFFER_SIZE - 4, bufPnt);
				serPutString(1, bufPnt);
				readChars = 0; bufPnt = readBuf;
#ifdef WITH_WDT
				restart_wdt();
#endif
			}

		} while (eoiStatus);

		while (!serTxBufferEmpty());
		if (readChars < strip) {
			serPutString(readChars - strip, bufPnt);
		}

	} else {	//	NOT read_until_eoi

		do {
			eoiStatus = gpib_receive(&readCharacter);
			if (eoiStatus == 0xff) {
				return true;
			}
			if (eos_code != 0) {
				//	Check for EOM char
				if (readCharacter != eos_string[0]) {
					//	Copy the read char into the buffer
					readBuf[readChars++] = readCharacter;
				} else {
					reading_done = true;
				}
			} else {
				if (readCharacter == eos_string[1]) {
					if (readBuf[readChars - 1] == eos_string[0]) {
						readChars--;
						reading_done = true;
					}
				} else {
					readBuf[readChars++] = readCharacter;
				}
			}
			if (readChars/* == READ_BUFFER_SIZE - 4*/) {
				while (!serTxBufferEmpty());
				//serPutString(READ_BUFFER_SIZE - 4, bufPnt);
				serPutString(1, bufPnt);
				readChars = 0; bufPnt = readBuf;
#ifdef WITH_WDT
				restart_wdt();
#endif
			}

		} while (reading_done == false);
		reading_done = false;

		while (!serTxBufferEmpty());
		if (readChars < strip) {
			serPutString(readChars - strip, bufPnt);
		}
	}

	if (eot_enable == 1) {
		serPutByte(eot_char);
	}

#ifdef VERBOSE_DEBUG
	serPutString(STRING_AND_LENGTH("gpib_read loop end"));
	serPutByte(eot_char);
#endif

	if (mode) {
		//	Command all talkers and listeners to stop
		cmd_buf[0] = CMD_UNT;
		errorFound = gpib_cmd(cmd_buf, 1);
		cmd_buf[0] = CMD_UNL;
		errorFound = errorFound || gpib_cmd(cmd_buf, 1);
	}

#ifdef VERBOSE_DEBUG
	serPutString(STRING_AND_LENGTH("gpib_read end"));
	serPutByte(eot_char);
#endif

	return errorFound;
}

/**
 *	Address the currently specified GPIB address
 *	(as set by the ++addr cmd) to listen
 */
bool addressTarget(uint8_t address) {
	bool writeError = false;
	cmd_buf[0] = CMD_UNT;
	writeError = gpib_cmd(cmd_buf, 1);
	cmd_buf[0] = CMD_UNL; // Everyone stop listening
	writeError = writeError || gpib_cmd(cmd_buf, 1);
	cmd_buf[0] = address + 0x20;
	writeError = writeError || gpib_cmd(cmd_buf, 1);
	return writeError;
}

/**
 *	Set serial poll mode
 */
void serial_poll(uint8_t address) {
	uint8_t error;
	uint8_t status_byte;

	//	enable serial poll
	cmd_buf[0] = CMD_SPE;
	error = gpib_cmd(cmd_buf, 1);
	cmd_buf[0] = address + 0x40;
	error = error || gpib_cmd(cmd_buf, 1);
	if (error) return;

	error = gpib_receive(&status_byte);
	if (error == 1)
		error = 0;	//	gpib_receive returns EOI lvl and 0xFF on errors
	if (error == 0xff)
		error = 1;

	//	disable serial poll
	cmd_buf[0] = CMD_SPD;
	gpib_cmd(cmd_buf, 1);
	if (!error) {
		serPutNumDec(status_byte);
		serPutByte(eot_char);
	}

}

/**
 *	Original main function
 */
void pic_main(void) {
	bool writeError = false;
	bool inputLine = false;
	uint8_t* buf_pnt = buf;

	// Original Command Set
	const uint8_t addressBuf[4] = "+a:";
	const uint8_t timeoutBuf[4] = "+t:";
	const uint8_t eosBuf[6] = "+eos:";
	const uint8_t eoiBuf[6] = "+eoi:";
	const uint8_t testBuf[6] = "+test";
	const uint8_t readCmdBuf[6] = "+read";
	const uint8_t getCmdBuf[5] = "+get";
	const uint8_t stripBuf[8] = "+strip:";
	const uint8_t versionBuf[5] = "+ver";
	const uint8_t autoReadBuf[11] = "+autoread:";
	const uint8_t resetBuf[7] = "+reset";
	const uint8_t debugBuf[8] = "+debug:";

	// Prologix Compatible Command Set
	const uint8_t addrBuf[7] = "++addr";
	const uint8_t autoBuf[7] = "++auto";
	const uint8_t clrBuf[6] = "++clr";
	const uint8_t eotEnableBuf[13] = "++eot_enable";
	const uint8_t eotCharBuf[11] = "++eot_char";
	const uint8_t ifcBuf[6] = "++ifc";
	const uint8_t lloBuf[6] = "++llo";
	const uint8_t locBuf[6] = "++loc";
	const uint8_t lonBuf[6] = "++lon";	//	TODO: Listen mode
	const uint8_t modeBuf[7] = "++mode";
	const uint8_t readTimeoutBuf[14] = "++read_tmo_ms";
	const uint8_t rstBuf[6] = "++rst";
	const uint8_t savecfgBuf[10] = "++savecfg";
	const uint8_t spollBuf[8] = "++spoll";
	const uint8_t srqBuf[6] = "++srq";
	const uint8_t statusBuf[9] = "++status";
	const uint8_t trgBuf[6] = "++trg";
	const uint8_t verBuf[6] = "++ver";
	//const uint8_t helpBuf[7] = "++help"; //TODO

	PORT_TP.OUTSET = BIT_TP;	//	Turn on the error LED

	// Setup the Watchdog Timer
#ifdef WITH_WDT
	setup_wdt(WDT_ON);
#endif
#ifdef WITH_TIMEOUT
	// Setup the timer
	//set_rtcc(0);
	//setup_timer_2(T2_DIV_BY_16,144,2); // 1ms interupt
	//enable_interrupts(GLOBAL);
#endif

	// Handle the EEPROM stuff
	if (eeprom_read_byte((uint8_t*)&ConfigData.valEepCode) == VALID_EEPROM_CODE) {
		mode =				eeprom_read_byte((uint8_t*)&ConfigData.mode);
		partnerAddress =	eeprom_read_byte((uint8_t*)&ConfigData.partnerAddr);
		eot_char =			eeprom_read_byte((uint8_t*)&ConfigData.eotChar);
		eot_enable =		eeprom_read_byte((uint8_t*)&ConfigData.eotEn);
		eos_code =			eeprom_read_byte((uint8_t*)&ConfigData.eosCode);
		eoiUse =			eeprom_read_byte((uint8_t*)&ConfigData.eoiUse);
		autoRead =			eeprom_read_byte((uint8_t*)&ConfigData.aRead);
		listen_only =		eeprom_read_byte((uint8_t*)&ConfigData.lstnOnl);
		save_cfg =			eeprom_read_byte((uint8_t*)&ConfigData.saveCfg);

		switch (eos_code) {
		case 0:
			eos_code = 0;
			eos_string[0] = 13;
			eos_string[1] = 10;
			eos_string[2] = 0;
			eos = 10;
			break;
		case 1:
			eos_code = 1;
			eos_string[0] = 13;
			eos_string[1] = 0;
			eos = 13;
			break;
		case 2:
			eos_code = 2;
			eos_string[0] = 10;
			eos_string[1] = 0;
			eos = 10;
			break;
		default:
			eos_code = 3;
			eos_string[0] = 0;
			eos = 0;
			break;
		}
	} else {
		eeprom_write_byte((uint8_t*)&ConfigData.valEepCode, VALID_EEPROM_CODE);
		eeprom_write_byte((uint8_t*)&ConfigData.mode,			1);
		eeprom_write_byte((uint8_t*)&ConfigData.partnerAddr,	1);
		eeprom_write_byte((uint8_t*)&ConfigData.eotChar,		13);
		eeprom_write_byte((uint8_t*)&ConfigData.eotEn,			1);
		eeprom_write_byte((uint8_t*)&ConfigData.eosCode,		3);
		eeprom_write_byte((uint8_t*)&ConfigData.eoiUse,			1);
		eeprom_write_byte((uint8_t*)&ConfigData.aRead,			1);
		eeprom_write_byte((uint8_t*)&ConfigData.lstnOnl,		0);
		eeprom_write_byte((uint8_t*)&ConfigData.saveCfg,		1);
	}

	//	Start all the GPIB related stuff
	//	Initialize the GPIB bus
	initGpibPins();
	//AssertNrfd();	//	Needed?
	RaiseNdac();	//	Needed?

	if (mode) {
		assignGpibController(0);
	}

#ifdef VERBOSE_DEBUG
	switch (restart_cause()) {
	case WDT_TIMEOUT:
		serPutString(STRING_AND_LENGTH("WDT restart\r\n"));
		break;
	case NORMAL_POWER_UP:
		serPutString(STRING_AND_LENGTH("Normal power up\r\n"));
		break;
	}
#endif

	//	Main execution loop
	while (1) {
#ifdef WITH_WDT
		restart_wdt();
#endif
		//	Receive lines from serial input
		if (!serRxBufferEmpty()) {
			*buf_pnt = serGetByte();
#ifdef SER_ECHO
			serPutByte(*buf_pnt);	//	Echo
#endif // SER_ECHO
			if (*buf_pnt >= 32 && *buf_pnt < 127
					&& (bufLength < numelem(buf) - 1)) {
				//	Accept printable ASCII
				buf_pnt++; bufLength++;
			} else if (*buf_pnt == 10 || *buf_pnt == 13) {
				*buf_pnt = 0;
				buf_pnt = buf;
				inputLine = true;
			}
		}

		if (inputLine) {
			inputLine = false;

			if (*buf_pnt == '+') {	//	Controller commands start with a +
				// +a:N
				if (strncmp((char*)buf_pnt, (char*)addressBuf, 3) == 0 ) {
					partnerAddress = atoi((char*)(buf_pnt + 3));	//	Parse out the GPIB address
				//	++addr N
				} else if (strncmp((char*)buf_pnt, (char*)addrBuf, 6) == 0) {
					if (*(buf_pnt + 6) == 0) {
						serPutNumDec(partnerAddress);
						serPutByte(eot_char);
					} else if (*(buf_pnt + 6) == 32) {
						partnerAddress = atoi((char*)(buf_pnt + 7));
					}
				//	+t:N
				} else if (strncmp((char*)buf_pnt, (char*)timeoutBuf, 3) == 0) {
					timeoutMillis = atol((char*)(buf_pnt + 3));	//	Parse out the timeout period
				//	++read_tmo_ms N
				} else if (strncmp((char*)buf_pnt, (char*)readTimeoutBuf, 13) == 0) {
					if (*(buf_pnt + 13) == 0) {
						serPutNumDecL(timeoutMillis);
						serPutByte(eot_char);
					} else if (*(buf_pnt + 13) == 32) {
						timeoutMillis = atol((char*)(buf_pnt + 14));
					}
				//	+read
				} else if ((strncmp((char*)buf_pnt, (char*)readCmdBuf, 5) == 0) && mode) {
					if (gpib_read(eoiUse)) {
						if (debug) {
							serPutString(STRING_AND_LENGTH("Read error occurred."));
							serPutByte(eot_char);
						}
						//_delay_ms(1);
						//reset_cpu();
					}
				// ++read
				} else if ((strncmp((char*)buf_pnt + 1, (char*)readCmdBuf, 5) == 0) && mode) {
					if (*(buf_pnt + 6) == 0) {
						gpib_read(false); // read until EOS condition
					} else if (*(buf_pnt + 7) == 101) {
						gpib_read(true); // read until EOI flagged
					}
					/*else if (*(buf_pnt + 6) == 32) {
						// read until specified character
					}*/
				// +test
				} else if (strncmp((char*)buf_pnt, (char*)testBuf, 5) == 0) {
					serPutString(STRING_AND_LENGTH("testing"));
					serPutByte(eot_char);
				// +eos:N
				} else if (strncmp((char*)buf_pnt, (char*)eosBuf, 5) == 0) {
					eos = atoi((char*)(buf_pnt + 5)); // Parse out the end of string byte
					eos_string[0] = eos;
					eos_string[1] = 0;
					eos_code = 4;
				// ++eos {0|1|2|3}
				} else if (strncmp((char*)buf_pnt + 1, (char*)eosBuf, 4) == 0) {
					if (*(buf_pnt + 5) == 0) {
						serPutNumDec(eos_code);
						serPutByte(eot_char);
					} else if (*(buf_pnt + 5) == 32) {
						eos_code = atoi((char*)(buf_pnt + 6));
						switch (eos_code) {
						case 0:
							eos_code = 0;
							eos_string[0] = 13;
							eos_string[1] = 10;
							eos_string[2] = 0;
							eos = 10;
							break;
						case 1:
							eos_code = 1;
							eos_string[0] = 13;
							eos_string[1] = 0;
							eos = 13;
							break;
						case 2:
							eos_code = 2;
							eos_string[0] = 10;
							eos_string[1] = 0;
							eos = 10;
							break;
						default:
							eos_code = 3;
							eos_string[0] = 0;
							eos = 0;
							break;
						}
					}
				// +eoi:{0|1}
				} else if (strncmp((char*)buf_pnt, (char*)eoiBuf, 5) == 0) {
					eoiUse = !!atoi((char*)(buf_pnt + 5));	//	Parse out the end of string byte
				// ++eoi {0|1}
				} else if (strncmp((char*)buf_pnt + 1, (char*)eoiBuf, 4) == 0) {
					if (*(buf_pnt + 5) == 0) {
						serPutNumDec(eoiUse);
						serPutByte(eot_char);
					} else if (*(buf_pnt + 5) == 32) {
						eoiUse = !!atoi((char*)(buf_pnt + 6));
					}
				// +strip:{0|1}
				} else if (strncmp((char*)buf_pnt, (char*)stripBuf, 7) == 0) {
					strip = !!atoi((char*)(buf_pnt + 7));	//	Parse out the end of string byte
				// +ver
				} else if (strncmp((char*)buf_pnt, (char*)versionBuf, 4) == 0) {
					serPutNumDec(version);
					serPutByte(eot_char);
				// ++ver
				} else if (strncmp((char*)buf_pnt, (char*)verBuf, 5) == 0) {
					serPutString(STRING_AND_LENGTH("Version "));
					serPutNumDec(version);
					serPutByte(eot_char);
				// +get
				} else if (strncmp((char*)buf_pnt, (char*)getCmdBuf, 4) == 0 && mode) {
					if (*(buf_pnt + 5) == 0) {
						writeError = writeError || addressTarget(partnerAddress);
						cmd_buf[0] = CMD_GET;
						gpib_cmd(cmd_buf, 1);
					} /* else if (*(buf_pnt + 5) == 32) {
						TODO: Add support for specified addresses
					} */
				// ++trg
				} else if (strncmp((char*)buf_pnt, (char*)trgBuf, 5) == 0 && mode) {
					if (*(buf_pnt + 5) == 0) {
						writeError = writeError || addressTarget(partnerAddress);
						cmd_buf[0] = CMD_GET;
						gpib_cmd(cmd_buf, 1);
					}
					/*else if (*(buf_pnt + 5) == 32) {
						TODO: Add support for specified addresses
					}*/
				// +autoread:{0|1}
				} else if (strncmp((char*)buf_pnt, (char*)autoReadBuf, 10) == 0) {
					autoRead = !!atoi((char*)(buf_pnt + 10));
				// ++auto {0|1}
				} else if (strncmp((char*)buf_pnt, (char*)autoBuf, 6) == 0) {
					if (*(buf_pnt + 6) == 0) {
						serPutNumDec(autoRead);
						serPutByte(eot_char);
					} else if (*(buf_pnt + 6) == 32) {
						autoRead = !!atoi((char*)(buf_pnt + 7));
					}
				// +reset
				} else if (strncmp((char*)buf_pnt, (char*)resetBuf, 6) == 0) {
					_delay_ms(1);
					reset_cpu();
				// ++rst
				} else if (strncmp((char*)buf_pnt, (char*)rstBuf, 5) == 0) {
					_delay_ms(1);
					reset_cpu();
				// +debug:{0|1}
				} else if (strncmp((char*)buf_pnt, (char*)debugBuf, 7) == 0) {
					debug = !!atoi((char*)(buf_pnt + 7));
				// ++debug {0|1}
				} else if (strncmp((char*)buf_pnt + 1, (char*)debugBuf, 6) == 0) {
					if (*(buf_pnt + 7) == 0) {
						serPutNumDec(debug);
						serPutByte(eot_char);
					} else if (*(buf_pnt + 7) == 32) {
						debug = atoi((char*)(buf_pnt + 8)) == 1;
					}
				// ++clr
				} else if ((strncmp((char*)buf_pnt, (char*)clrBuf, 5) == 0) && mode) {
					// This command is special in that we must
					// address a specific instrument.
					writeError = writeError || addressTarget(partnerAddress);
					cmd_buf[0] = CMD_SDC;
					writeError = writeError || gpib_cmd(cmd_buf, 1);
				// ++eot_enable {0|1}
				} else if (strncmp((char*)buf_pnt, (char*)eotEnableBuf, 12) == 0) {
					if (*(buf_pnt + 12) == 0) {
						serPutNumDec(eot_enable);
						serPutByte(eot_char);
					} else if (*(buf_pnt + 12) == 32) {
						eot_enable = !!atoi((char*)(buf_pnt + 13));
					}
				// ++eot_char N
				} else if (strncmp((char*)buf_pnt, (char*)eotCharBuf, 10) == 0) {
					if (*(buf_pnt + 10) == 0) {
						serPutNumDec(eot_char);
						serPutByte(eot_char);
					} else if (*(buf_pnt + 10) == 32) {
						eot_char = atoi((char*)(buf_pnt + 11));
					}
				// ++ifc
				} else if ((strncmp((char*)buf_pnt, (char*)ifcBuf, 5) == 0) && mode) {
					AssertIfc();	//	Assert interface clear.
					_delay_us(150);
					DeassertIfc();	//	Finishing clearing interface
				// ++llo
				} else if ((strncmp((char*)buf_pnt, (char*)lloBuf, 5) == 0) && mode) {
					writeError = writeError || addressTarget(partnerAddress);
					cmd_buf[0] = CMD_LLO;
					writeError = writeError || gpib_cmd(cmd_buf, 1);
				// ++loc
				} else if ((strncmp((char*)buf_pnt, (char*)locBuf, 5) == 0) && mode) {
					writeError = writeError || addressTarget(partnerAddress);
					cmd_buf[0] = CMD_GTL;
					writeError = writeError || gpib_cmd(cmd_buf, 1);
				// ++lon {0|1}
				} else if ((strncmp((char*)buf_pnt, (char*)lonBuf, 5) == 0) && !mode) {
					if (*(buf_pnt + 5) == 0) {
						serPutNumDec(listen_only);
						serPutByte(eot_char);
					} else if (*(buf_pnt + 5) == 32) {
						listen_only = atoi((char*)(buf_pnt + 6));
						if (listen_only != 1) {
							listen_only = 0;	//	If non-bool sent, set to disable
						}
					}
				// ++mode {0|1}
				} else if (strncmp((char*)buf_pnt, (char*)modeBuf, 6) == 0) {
					if (*(buf_pnt + 6) == 0) {
						serPutNumDec(mode);
						serPutByte(eot_char);
					} else if (*(buf_pnt + 6) == 32) {
						mode = !!atoi((char*)(buf_pnt + 7));
						initGpibPins();
						if (mode) {
							assignGpibController(0);
						}
					}
				// ++savecfg {0|1}
				} else if (strncmp((char*)buf_pnt, (char*)savecfgBuf, 9) == 0) {
					if (*(buf_pnt + 9) == 0) {
						serPutNumDec(save_cfg);
						serPutByte(eot_char);
					} else if (*(buf_pnt + 9) == 32) {
						save_cfg = !!atoi((char*)(buf_pnt + 10));
						if (save_cfg) {
							eeprom_write_byte((uint8_t*)&ConfigData.mode,			mode);
							eeprom_write_byte((uint8_t*)&ConfigData.partnerAddr,	partnerAddress);
							eeprom_write_byte((uint8_t*)&ConfigData.eotChar,		eot_char);
							eeprom_write_byte((uint8_t*)&ConfigData.eotEn,			eot_enable);
							eeprom_write_byte((uint8_t*)&ConfigData.eosCode,		eos_code);
							eeprom_write_byte((uint8_t*)&ConfigData.eoiUse,			eoiUse);
							eeprom_write_byte((uint8_t*)&ConfigData.aRead,			autoRead);
							eeprom_write_byte((uint8_t*)&ConfigData.lstnOnl,		listen_only);
							eeprom_write_byte((uint8_t*)&ConfigData.saveCfg,		save_cfg);
						}
					}
				// ++srq
				} else if ((strncmp((char*)buf_pnt, (char*)srqBuf, 5) == 0) && mode) {
					serPutNumDec(!ReadSrq());
					serPutByte(eot_char);
				// ++spoll N
				} else if ((strncmp((char*)buf_pnt, (char*)spollBuf, 7) == 0) && mode) {
					if (*(buf_pnt + 7) == 0) {
						serial_poll(partnerAddress);
					} else if (*(buf_pnt + 7) == 32) {
						serial_poll(atoi((char*)(buf_pnt + 8)));
					}
				// ++status
				} else if ((strncmp((char*)buf_pnt, (char*)statusBuf, 8) == 0) && !mode) {
					if (*(buf_pnt + 8) == 0) {
					   serPutNumDec(status_byte);
					   serPutByte(eot_char);
					} else if (*(buf_pnt + 8) == 32) {
						status_byte = atoi((char*)(buf_pnt + 9));
					}
				} else {
					if (debug) {
						serPutString(STRING_AND_LENGTH("Unrecognized command."));
						serPutByte(eot_char);
					}
				}
			} else {
				//	Not an internal command, send to bus

				//	Command all talkers and listeners to stop, and tell target to listen
				if (mode) {
					writeError = writeError || addressTarget(partnerAddress);
					// Set the controller into talker mode
					cmd_buf[0] = myAddress + 0x40;
					writeError = writeError || gpib_cmd(cmd_buf, 1);
				}

				// Send out command to the bus
#ifdef VERBOSE_DEBUG
				serPutString(STRING_AND_LENGTH("gpib_write: "));
				serPutNumDec(buf_pnt);
				serPutByte(eot_char);
#endif

				if (mode || device_talk) {
					//	If have an EOS char, need to output termination byte to instrument
					if (eos_code != 3) {
						writeError = writeError || gpib_write(buf_pnt, 0, 0);
						if (!writeError)
							writeError = gpib_write(eos_string, 0, eoiUse);
#ifdef VERBOSE_DEBUG
						serPutString(STRING_AND_LENGTH("eos_string: "));
						serPutString(strlen(eos_string), eos_string);
						serPutByte(eot_char);
#endif
					} else {
						writeError = writeError || gpib_write(buf_pnt, 0, 1);
					}
				}

				// If cmd contains a question mark -> is a query
				if (autoRead && mode) {
					if ((strchr((char*)buf_pnt, '?') != NULL) && !writeError) {
						gpib_read(eoiUse);
					} else if (writeError) {
						writeError = false;
					}
				}
			}	//	Internal command

			bufLength = 0;
		}	//	Serial input

		if (!mode) {
			//	When in device mode we should be checking the status of the
			//	ATN line to see what we should be doing
			if (!ReadAtn()) {
				AssertNdac();
				gpib_receive(cmd_buf);	//	Get the CMD byte sent by the controller
				RaiseNrfd();
				if (cmd_buf[0] == partnerAddress + 0x40) {
					device_talk = true;
#ifdef VERBOSE_DEBUG
					serPutString(STRING_AND_LENGTH("Instructed to talk"));
					serPutByte(eot_char);
#endif
				} else if (cmd_buf[0] == partnerAddress + 0x20) {
					device_listen = true;
#ifdef VERBOSE_DEBUG
					serPutString(STRING_AND_LENGTH("Instructed to listen"));
					serPutByte(eot_char);
#endif
				} else if (cmd_buf[0] == CMD_UNL) {
					device_listen = false;
#ifdef VERBOSE_DEBUG
					serPutString(STRING_AND_LENGTH("Instructed to stop listen"));
					serPutByte(eot_char);
#endif
				} else if (cmd_buf[0] == CMD_UNT) {
					device_talk = false;
#ifdef VERBOSE_DEBUG
					serPutString(STRING_AND_LENGTH("Instructed to stop talk"));
					serPutByte(eot_char);
#endif
				} else if (cmd_buf[0] == CMD_SPE) {
					device_srq = true;
#ifdef VERBOSE_DEBUG
					serPutString(STRING_AND_LENGTH("SRQ start"));
					serPutByte(eot_char);
#endif
				} else if (cmd_buf[0] == CMD_SPD) {
					device_srq = false;
#ifdef VERBOSE_DEBUG
					serPutString(STRING_AND_LENGTH("SRQ end"));
					serPutByte(eot_char);
#endif
				} else if (cmd_buf[0] == CMD_DCL) {
					serPutNumDec(CMD_DCL);
					serPutByte(eot_char);
					device_listen = false; device_talk = false;
					device_srq = false; status_byte = 0;
				} else if ((cmd_buf[0] == CMD_LLO) && (device_listen)) {
					serPutNumDec(CMD_LLO);
					serPutByte(eot_char);
				} else if ((cmd_buf[0] == CMD_GTL) && (device_listen)) {
					serPutNumDec(CMD_GTL);
					serPutByte(eot_char);
				} else if ((cmd_buf[0] == CMD_GET) && (device_listen)) {
					serPutNumDec(CMD_GET);
					serPutByte(eot_char);
				}
				RaiseNdac();
			} else {
				_delay_us(10);
				if (ReadAtn()) {
					if (device_listen) {
						AssertNdac();
#ifdef VERBOSE_DEBUG
						serPutString(STRING_AND_LENGTH(Starting device mode gpib_read));
						serPutByte(eot_char);
#endif
						gpib_read(eoiUse);
						device_listen = false;
					} else if (device_talk && device_srq) {
						gpib_write(&status_byte, 1, 0);
						device_talk = false; device_srq = false;
					}
				}
			}

		}

	}	//	Main loop

}
