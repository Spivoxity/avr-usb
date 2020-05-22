/* usbtiny.c -- based on FabISP Firmware */

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for _delay_ms() */
#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include "usbdrv.h"
#include "mylib.h"

typedef unsigned word;

enum {
	// Generic requests
	USBTINY_ECHO,		// 0x00: echo test
	USBTINY_READ,		// 0x01: read byte (wIndex:address)
	USBTINY_WRITE,		// 0x02: write byte (wIndex:address, wValue:value)
	USBTINY_CLR,		// 0x03: clear bit (wIndex:address, wValue:bitno)
	USBTINY_SET,		// 0x04: set bit (wIndex:address, wValue:bitno)
	// Programming requests
	USBTINY_POWERUP,	// 0x05: apply power (wValue:SCK-period, wIndex:RESET)
	USBTINY_POWERDOWN,	// 0x06: remove power from chip
	USBTINY_SPI,		// 0x07: issue SPI command (wValue:c1c0, wIndex:c3c2)
	USBTINY_POLL_BYTES,	// 0x08: set poll bytes for write (wValue:p1p2)
	USBTINY_FLASH_READ,	// 0x09: read flash (wIndex:address)
	USBTINY_FLASH_WRITE,	// 0x0A: write flash (wIndex:address, wValue:timeout)
	USBTINY_EEPROM_READ,	// 0x0B: read eeprom (wIndex:address)
	USBTINY_EEPROM_WRITE,	// 0x0C: write eeprom (wIndex:address, wValue:timeout)
};

// Programmer output pins:
//	LED	PA2
//	RESET	PA3
//	SCK	PA4
//	MOSI	PA6
#define	PORT		PORTA
#define	DDR		DDRA
#define	POWER_MASK	(1 << 2)
#define	RESET_MASK	(1 << 3)
#define	SCK_MASK	(1 << 4)
#define	MOSI_MASK	(1 << 6)
#define ALL_MASK	(POWER_MASK | RESET_MASK | SCK_MASK | MOSI_MASK)

// Programmer input pins:
//	MISO	PA5
#define	PIN		PINA
#define	MISO_MASK	(1 << 5)

// Local data
static	byte		sck_period;	// SCK period in microseconds (1..250)
static	byte		poll1;		// first poll byte for write
static	byte		poll2;		// second poll byte for write
static	word		address;	// read/write address
static	word		timeout;	// write timeout in usec
static	byte		cmd0;		// current read/write command byte
static	byte		cmd[4];		// SPI command buffer
static	byte		res[4];		// SPI result buffer

// Delay exactly <sck_period> times 0.5 microseconds (6 cycles).
__attribute__((always_inline))
static inline void delay (void) {
	asm volatile(
		"	mov	__tmp_reg__,%0	\n"
		"0:	rjmp	1f		\n"
		"1:	nop			\n"
		"	dec	__tmp_reg__	\n"
		"	brne	0b		\n"
		: : "r" (sck_period));
}

// Issue one SPI command.
static void spi(byte* cmd, byte* res) {
     byte i;
     byte c;
     byte r;
     byte mask;

     for (i = 0; i < 4; i++) {
          c = *cmd++;
          r = 0;
          for (mask = 0x80; mask; mask >>= 1) {
               if (c & mask) PORT |= MOSI_MASK;
               delay();
               PORT |= SCK_MASK;
               delay();
               r <<= 1;
               if (PIN & MISO_MASK) r++;
               PORT &= ~ MOSI_MASK;
               PORT &= ~ SCK_MASK;
          }
          *res++ = r;
     }
}

// Create and issue a read or write SPI command.
static void spi_rw(void) {
     word a;

     a = address++;
     if (cmd0 & 0x80)
          a <<= 1;           // eeprom
     cmd[0] = cmd0;
     if (a & 1)
          cmd[0] |= 0x08;
     cmd[1] = a >> 9;
     cmd[2] = a >> 1;
     spi(cmd, res);
}

static uchar dataBuffer[8];  /* buffer must stay valid when usbFunctionSetup returns */

// Handle a non-standard SETUP packet.
extern usbMsgLen_t usbFunctionSetup(byte data[8]) {
     byte bit;
     byte mask;
     byte* addr;
     byte req;
     byte i;

     // Generic requests
     req = data[1];
     if (req == USBTINY_ECHO) {
          for (i = 0; i < 8; i++)
               dataBuffer[i] = data[i];
          usbMsgPtr = dataBuffer;
          return 8;
     }

     addr = (byte*) (int) data[4];

     if (req == USBTINY_READ) {
          dataBuffer[0] = *addr;
          usbMsgPtr = dataBuffer;
          return 1;
     }

     if (req == USBTINY_WRITE) {
          *addr = data[2];
          return 0;
     }

     bit = data[2] & 7;
     mask = 1 << bit;

     if (req == USBTINY_CLR) {
          *addr &= ~ mask;
          return 0;
     }

     if (req == USBTINY_SET) {
          *addr |= mask;
          return 0;
     }

     // Programming requests
     if (req == USBTINY_POWERUP) {
          sck_period = data[2];
          mask = POWER_MASK;
          if (data[4])
               mask |= RESET_MASK;
          DDR  |= ALL_MASK;
          PORT = (PORT & ~ ALL_MASK) | mask;
          return 0;
     }

     if (req == USBTINY_POWERDOWN) {
          PORT |= RESET_MASK;
          PORT &= ~ ALL_MASK;
          DDR  &= ~ ALL_MASK;
          return 0;
     }

     if (! PORT)
          return 0;

     if (req == USBTINY_SPI) {
          spi(data + 2, dataBuffer);
          usbMsgPtr = dataBuffer;
          return 4;
     }

     if (req == USBTINY_POLL_BYTES) {
          poll1 = data[2];
          poll2 = data[3];
          return 0;
     }

     address = * (word*) &data[4];

     if (req == USBTINY_FLASH_READ) {
          cmd0 = 0x20;
          return USB_NO_MSG; // usbFunctionRead() will be called to get the data
     }

     if (req == USBTINY_EEPROM_READ) {
          cmd0 = 0xa0;
          return USB_NO_MSG; // usbFunctionRead() will be called to get the data
     }

     timeout = * (word*) &data[2];

     if (req == USBTINY_FLASH_WRITE) {
          cmd0 = 0x40;
          return USB_NO_MSG;	// data will be received by usbFunctionWrite()
     }

     if (req == USBTINY_EEPROM_WRITE) {
          cmd0 = 0xc0;
          return USB_NO_MSG;	// data will be received by usbFunctionWrite()
     }
     return 0;
}

// Handle an IN packet.
extern byte usbFunctionRead(byte* data, byte len) {
     byte i;

     for (i = 0; i < len; i++) {
          spi_rw();
          data[i] = res[3];
     }
     return len;
}

// Handle an OUT packet.
extern byte usbFunctionWrite(byte* data, byte len) {
     byte i;
     word usec;
     byte r;

     for (i = 0; i < len; i++) {
          cmd[3] = data[i];
          spi_rw();
          cmd[0] ^= 0x60;	// turn write into read
          for (usec = 0; usec < timeout; usec += 32 * sck_period) {	
               // when timeout > 0, poll until byte is written
               spi(cmd, res);
               r = res[3];
               if (r == cmd[3] && r != poll1 && r != poll2)
                    break;
          }
     }
        
     return 1;
}

int main(void) {
     byte i;

     wdt_enable(WDTO_1S);
     /* Even if you don't use the watchdog, turn it off here. On newer devices,
      * the status of the watchdog (on/off, period) is PRESERVED OVER RESET!
      */

     /* RESET status: all port bits are inputs without pull-up.
      * That's the way we need D+ and D-. Therefore we don't need any
      * additional hardware initialization.
      */
     usbInit();
     usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
     i = 0;
     while (--i) {             /* fake USB disconnect for > 250 ms */
          wdt_reset();
          _delay_ms(1);
     }
     usbDeviceConnect();
     sei();
     for (;;) {                /* main event loop */
          wdt_reset();
          usbPoll();
     }
     return 0;
}

