// jiggler.c
// Mouse jiggler based on VUSB hid-mouse example
// Mike Spivey

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for _delay_ms() */
#include <avr/pgmspace.h>   /* required by usbdrv.h */

#include "mylib.h"
#include "usbdrv.h"

#define DELTA 1                 // Distance to move
#define PERIOD 10000            // Period between moves (ms)

#ifdef TINY84
#define led_init()  DDRA |= 0x4;
#define led_on()  PORTA |= 0x4
#define led_off()  PORTA &= ~0x4
#endif

#ifdef TINY85
#define led_on()  PORTB |= 0x8
#define led_init()  DDRB |= 0x8;
#define led_off()  PORTB &= ~0x8
#endif

// USB interface

#ifdef TINY85
#include "osccal.c"

void hadUsbReset(void) {
     cli();
     calibrateOscillator();
     sei();
}
#endif

const PROGMEM char usbHidReportDescriptor[52] = {
     0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
     0x09, 0x02,                    // USAGE (Mouse)
     0xa1, 0x01,                    // COLLECTION (Application)
     0x09, 0x01,                    //   USAGE (Pointer)
     0xA1, 0x00,                    //   COLLECTION (Physical)
     0x05, 0x09,                    //     USAGE_PAGE (Button)
     0x19, 0x01,                    //     USAGE_MINIMUM
     0x29, 0x03,                    //     USAGE_MAXIMUM
     0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
     0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
     0x95, 0x03,                    //     REPORT_COUNT (3)
     0x75, 0x01,                    //     REPORT_SIZE (1)
     0x81, 0x02,                    //     INPUT (Data,Var,Abs)
     0x95, 0x01,                    //     REPORT_COUNT (1)
     0x75, 0x05,                    //     REPORT_SIZE (5)
     0x81, 0x03,                    //     INPUT (Const,Var,Abs)
     0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
     0x09, 0x30,                    //     USAGE (X)
     0x09, 0x31,                    //     USAGE (Y)
     0x09, 0x38,                    //     USAGE (Wheel)
     0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
     0x25, 0x7F,                    //     LOGICAL_MAXIMUM (127)
     0x75, 0x08,                    //     REPORT_SIZE (8)
     0x95, 0x03,                    //     REPORT_COUNT (3)
     0x81, 0x06,                    //     INPUT (Data,Var,Rel)
     0xC0,                          //   END_COLLECTION
     0xC0,                          // END COLLECTION
};

/* This is the same report descriptor as seen in a Logitech mouse. The data
 * described by this descriptor consists of 4 bytes:
 *      .  .  .  .  . B2 B1 B0 .... one byte with mouse button states
 *     X7 X6 X5 X4 X3 X2 X1 X0 .... 8 bit signed relative coordinate x
 *     Y7 Y6 Y5 Y4 Y3 Y2 Y1 Y0 .... 8 bit signed relative coordinate y
 *     W7 W6 W5 W4 W3 W2 W1 W0 .... 8 bit signed relative coordinate wheel
 */

typedef struct{
     uchar   buttonMask;
     char    dx;
     char    dy;
     char    dWheel;
} report_t;

static report_t reportBuffer;
static uchar idleRate;   // Repeat rate for keyboards, never used for mice

void setReport(int delta) {
     reportBuffer.buttonMask = 0;
     reportBuffer.dx = delta;
     reportBuffer.dy = 0;
     reportBuffer.dWheel = 0;
}

usbMsgLen_t usbFunctionSetup(uchar data[8]) {
     usbRequest_t *rq = (void *) data;

    /* The following requests are never used. But since they are required by
       the specification, we implement them in this example. */
    if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS) {
         /* class request type */
         switch (rq->bRequest) {
         case USBRQ_HID_GET_REPORT:
              /* wValue: ReportType (highbyte), ReportID (lowbyte) */
              /* we only have one report type, so don't look at wValue */
              usbMsgPtr = (void *) &reportBuffer;
              return sizeof(reportBuffer);
         case USBRQ_HID_GET_IDLE:
              usbMsgPtr = &idleRate;
              return 1;
         case USBRQ_HID_SET_IDLE:
              idleRate = rq->wValue.bytes[1];
              return 0;
         }
    } else {
         /* no vendor specific requests implemented */
    }

    return 0;   // default for unimplemented requests
}

volatile unsigned ticks = 0;
volatile unsigned long millis = 0;

ISR_NOBLOCK ISR(TIM0_OVF_vect) {
     // Use local copies of volatile variables
     unsigned t = ticks;
     unsigned long m = millis;

     t += 64*256;
     while (t >= (unsigned) (F_CPU/1000)) {
          t -= F_CPU/1000; m++;
     }

     ticks = t;
     millis = m;
}

unsigned long now(void) {
     unsigned long m;
     cli();
     m = millis;
     sei();
     return m;
}

void init_clock(void) {
     // Timer 0: prescale by 64, enable interrupt on overflow
     // Interrupt rate is 12 MHz / (64*256) ~= 732 Hz on Tiny84
     sbi(TCCR0B, CS01);
     sbi(TCCR0B, CS00);
#ifdef TINY84
     sbi(TIMSK0, TOIE0);
#endif
#ifdef TINY85
     sbi(TIMSK, TOIE0);
#endif
}

int main(void) {
     unsigned long last_move = 0;
     int delta = DELTA;

     usbInit();
     usbDeviceDisconnect();  // enforce re-enumeration
     delay_ms(250);
     usbDeviceConnect();

     led_init();
     init_clock();
     sei();

     while (1) {
          unsigned long t = now();

          usbPoll();

          if (t - last_move > 100) led_off();

          if (t - last_move > PERIOD && usbInterruptIsReady()){
               setReport(delta);
               usbSetInterrupt((void *) &reportBuffer, sizeof(reportBuffer));
               led_on();
               delta = -delta;
               last_move = t;
          }
     }
}
