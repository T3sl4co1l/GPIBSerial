#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED

#define F_CPU				32000000UL

#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/atomic.h>
#include <util/delay.h>
#include "bufserial.h"
#include "ccpwrite.h"

/*	Pin and port definitions (for serial see bufserial.h)  */

//	Note: DIO pins must be wired in order: PIN[0..7] = DIO[1..8]
#define PORT_DIO		PORTD
#define ReadDio()		(PORT_DIO.IN)
#define WriteDio(a)		(PORT_DIO.OUT = (a))
#define AssertDio()		(PORT_DIO.DIRSET = 0xff)
#define DeassertDio()	(PORT_DIO.DIRCLR = 0xff)

#define TalkEnable()	(PORT_TE.OUTSET = BIT_TE)
#define TalkDisable()	(PORT_TE.OUTCLR = BIT_TE)
#define PullupEnable()	(PORT_PE.OUTSET = BIT_PE)
#define PullupDisable()	(PORT_PE.OUTCLR = BIT_PE)

#define AssertAtn()		(PORT_ATN.OUTCLR = BIT_ATN, PORT_REN.DIRSET = BIT_ATN)
#define DeassertAtn()	(PORT_ATN.DIRCLR = BIT_ATN, PORT_ATN.OUTSET = BIT_ATN)
#define AssertEoi()		(PORT_EOI.OUTCLR = BIT_EOI, PORT_EOI.DIRSET = BIT_EOI)
#define RaiseEoi()		(PORT_EOI.OUTSET = BIT_EOI, PORT_EOI.DIRSET = BIT_EOI)
#define DeassertEoi()	(PORT_EOI.DIRCLR = BIT_EOI, PORT_EOI.OUTSET = BIT_EOI)
#define AssertSrq()		(PORT_SRQ.OUTCLR = BIT_SRQ, PORT_SRQ.DIRSET = BIT_SRQ)
#define DeassertSrq()	(PORT_SRQ.DIRCLR = BIT_SRQ, PORT_SRQ.OUTSET = BIT_SRQ)
#define AssertRen()		(PORT_REN.OUTCLR = BIT_REN, PORT_REN.DIRSET = BIT_REN)
#define DeassertRen()	(PORT_REN.DIRCLR = BIT_REN, PORT_REN.OUTSET = BIT_REN)
#define AssertIfc()		(PORT_IFC.OUTCLR = BIT_IFC, PORT_IFC.DIRSET = BIT_IFC)
#define DeassertIfc()	(PORT_IFC.DIRCLR = BIT_IFC, PORT_IFC.OUTSET = BIT_IFC)
#define AssertDav()		(PORT_DAV.OUTCLR = BIT_DAV, PORT_DAV.DIRSET = BIT_DAV)
#define RaiseDav()		(PORT_DAV.OUTSET = BIT_DAV, PORT_DAV.DIRSET = BIT_DAV)
#define DeassertDav()	(PORT_DAV.DIRCLR = BIT_DAV)
#define AssertNdac()	(PORT_NDAC.OUTCLR = BIT_NDAC, PORT_NDAC.DIRSET = BIT_NDAC)
#define RaiseNdac()		(PORT_NDAC.OUTSET = BIT_NDAC, PORT_NDAC.DIRSET = BIT_NDAC)
#define DeassertNdac()	(PORT_NDAC.DIRCLR = BIT_NDAC, PORT_NDAC.OUTSET = BIT_NDAC)
#define AssertNrfd()	(PORT_NRFD.OUTCLR = BIT_NRFD, PORT_NRFD.DIRSET = BIT_NRFD)
#define RaiseNrfd()		(PORT_NRFD.OUTSET = BIT_NRFD, PORT_NRFD.DIRSET = BIT_NRFD)
#define DeassertNrfd()	(PORT_NRFD.DIRCLR = BIT_NRFD, PORT_NRFD.OUTSET = BIT_NRFD)

#define ReadAtn()		(!!(PORT_ATN.IN & BIT_ATN))
#define ReadDav()		(!!(PORT_DAV.IN & BIT_DAV))
#define ReadEoi()		(!!(PORT_EOI.IN & BIT_EOI))
#define ReadNdac()		(!!(PORT_NDAC.IN & BIT_NDAC))
#define ReadNrfd()		(!!(PORT_NRFD.IN & BIT_NRFD))
#define ReadSrq()		(!!(PORT_SRQ.IN & BIT_SRQ))

#define PIN_HB			PIN3
#define BIT_HB			(1 << PIN3)
#define PORT_HB			PORTB
#define PIN_TP			PIN5
#define BIT_TP			(1 << PIN5)
#define PORT_TP			PORTB
#define PIN_REN			PIN0
#define BIT_REN			(1 << PIN0)
#define PORT_REN		PORTE
#define PIN_EOI			PIN1
#define BIT_EOI			(1 << PIN1)
#define PORT_EOI		PORTE
#define PIN_DAV			PIN2
#define BIT_DAV			(1 << PIN2)
#define PORT_DAV		PORTE
#define PIN_NRFD		PIN3
#define BIT_NRFD		(1 << PIN3)
#define PORT_NRFD		PORTE
#define PIN_NDAC		PIN4
#define BIT_NDAC		(1 << PIN4)
#define PORT_NDAC		PORTE
#define PIN_ATN			PIN5
#define BIT_ATN			(1 << PIN5)
#define PORT_ATN		PORTE
#define PIN_SRQ			PIN7
#define BIT_SRQ			(1 << PIN7)
#define PORT_SRQ		PORTE
#define PIN_IFC			PIN6
#define BIT_IFC			(1 << PIN6)
#define PORT_IFC		PORTE
#define PIN_SC			PIN4
#define BIT_SC			(1 << PIN4)
#define PORT_SC			PORTC
#define PIN_TE			PIN5
#define BIT_TE			(1 << PIN5)
#define PORT_TE			PORTC
#define PIN_PE			PIN6
#define BIT_PE			(1 << PIN6)
#define PORT_PE			PORTC
#define PIN_DC			PIN7
#define BIT_DC			(1 << PIN7)
#define PORT_DC			PORTC

//	Define this ONLY if the control bits are on the same port
#define PORT_CTRL		PORT_ATN

//	GPIB commands
#define CMD_DCL			0x14
#define CMD_UNL			0x3f
#define CMD_UNT			0x5f
#define CMD_GET			0x08
#define CMD_SDC			0x04
#define CMD_LLO			0x11
#define CMD_GTL			0x01
#define CMD_SPE			0x18
#define CMD_SPD			0x19

//	Timer shortcuts
#define enableTimerInt()	TCC0.INTCTRLA = TC_OVFINTLVL_HI_gc
#define disableTimerInt()	TCC0.INTCTRLA = 0
#define stopTimer()			disableTimerInt()
void startTimer(uint32_t time);

#define ResetWdt()			wdt_reset()
#define ResetCpu()			RST.CTRL = RST_SWRST_bm

#ifndef numelem
#define numelem(x)			( sizeof(x) / sizeof(x[0]) )
#endif

#define VALID_EEPROM_CODE	0xaa

#define SER_ECHO

typedef struct eeprom_settings_s {
	uint8_t valEepCode;		//	VALID_EEPROM_CODE
	uint8_t mode;			//	mode
	uint8_t partnerAddr;	//	partnerAddress
	uint8_t eotChar;		//	eot_char
	uint8_t eotEn;			//	eot_enable
	uint8_t eosCode;		//	eos_code
	uint8_t eoiUse;			//	eoiUse
	uint8_t aRead;			//	autoread
	uint8_t lstnOnl;		//	listen_only
	uint8_t saveCfg;		//	save_cfg
} eeprom_settings_t;

int main(void);
void pic_main(void);
void initialize(void);
void setSCDCMode(bool mode);
void setCtrlMode(bool mode);
void initGpibPins();
bool assignGpibController(uint8_t address);
bool gpib_cmd(const uint8_t* bytes, uint8_t length);
bool gpib_write(const uint8_t* bytes, uint8_t length, bool useEOI);
uint8_t myprintf(const char* s, ...);
bool _gpib_write(const uint8_t* bytes, uint8_t length, bool attention, bool useEOI);
uint8_t gpib_receive(uint8_t* byt);
bool gpib_read(bool read_until_eoi);
bool addressTarget(uint8_t address);
void serial_poll(uint8_t address);

#endif // MAIN_H_INCLUDED
