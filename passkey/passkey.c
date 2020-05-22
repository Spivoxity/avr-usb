/* passkey.c */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

#include "mylib.h"
#include "usbdrv.h"

static void hardwareInit(void) {
     usbInit();
     usbDeviceDisconnect();
     delay_ms(250);
     usbDeviceConnect();

#ifdef TINY84
     DDRA |= 0x4; // LED as output
#endif

#ifdef TINY85
     DDRB |= 0x8; // LED as output
#endif
}

#ifdef TINY85
#include "osccal.c"

void hadUsbReset(void) {
     cli();
     calibrateOscillator();
     sei();
}
#endif

static uchar    reportBuffer[2];    /* buffer for HID reports */
static uchar    idleRate;           /* in 4 ms units */

const PROGMEM char usbHidReportDescriptor[35] = {   /* USB report descriptor */
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION
};

/* We use a simplifed keyboard report descriptor which does not support the
 * boot protocol. We don't allow setting status LEDs and we only allow one
 * simultaneous key press (except modifiers). We can therefore use short
 * 2 byte input reports.
 * The report descriptor has been created with usb.org's "HID Descriptor Tool"
 * which can be downloaded from http://www.usb.org/developers/hidpage/.
 * Redundant entries (such as LOGICAL_MINIMUM and USAGE_PAGE) have been omitted
 * for the second INPUT item.
 */

/* Keyboard usage values, see usb.org's HID-usage-tables document, chapter
 * 10 Keyboard/Keypad Page for more codes.
 */
#define MOD_SHIFT_LEFT      (1<<1)

#define KEY_NONE 0x00 // No key pressed
#define KEY_A 0x04 // Keyboard a and A
#define KEY_B 0x05 // Keyboard b and B
#define KEY_C 0x06 // Keyboard c and C
#define KEY_D 0x07 // Keyboard d and D
#define KEY_E 0x08 // Keyboard e and E
#define KEY_F 0x09 // Keyboard f and F
#define KEY_G 0x0a // Keyboard g and G
#define KEY_H 0x0b // Keyboard h and H
#define KEY_I 0x0c // Keyboard i and I
#define KEY_J 0x0d // Keyboard j and J
#define KEY_K 0x0e // Keyboard k and K
#define KEY_L 0x0f // Keyboard l and L
#define KEY_M 0x10 // Keyboard m and M
#define KEY_N 0x11 // Keyboard n and N
#define KEY_O 0x12 // Keyboard o and O
#define KEY_P 0x13 // Keyboard p and P
#define KEY_Q 0x14 // Keyboard q and Q
#define KEY_R 0x15 // Keyboard r and R
#define KEY_S 0x16 // Keyboard s and S
#define KEY_T 0x17 // Keyboard t and T
#define KEY_U 0x18 // Keyboard u and U
#define KEY_V 0x19 // Keyboard v and V
#define KEY_W 0x1a // Keyboard w and W
#define KEY_X 0x1b // Keyboard x and X
#define KEY_Y 0x1c // Keyboard y and Y
#define KEY_Z 0x1d // Keyboard z and Z

#define KEY_1 0x1e // Keyboard 1 and !
#define KEY_2 0x1f // Keyboard 2 and @
#define KEY_3 0x20 // Keyboard 3 and #
#define KEY_4 0x21 // Keyboard 4 and $
#define KEY_5 0x22 // Keyboard 5 and %
#define KEY_6 0x23 // Keyboard 6 and ^
#define KEY_7 0x24 // Keyboard 7 and &
#define KEY_8 0x25 // Keyboard 8 and *
#define KEY_9 0x26 // Keyboard 9 and (
#define KEY_0 0x27 // Keyboard 0 and )

#define KEY_APOSTROPHE 0x34 // Keyboard ' and @
#define KEY_ENTER 0x28 // Keyboard Return (ENTER)
#define KEY_SPACE 0x2c // Keyboard Spacebar
#define KEY_MINUS 0x2d // Keyboard - and _
#define KEY_EQUAL 0x2e // Keyboard = and +
#define KEY_LEFTBRACE 0x2f // Keyboard [ and {
#define KEY_RIGHTBRACE 0x30 // Keyboard ] and }
#define KEY_BACKSLASH 0x31 // Keyboard \ and |
#define KEY_HASHTILDE 0x32 // Keyboard Non-US # and ~
#define KEY_SEMICOLON 0x33 // Keyboard ; and :
#define KEY_GRAVE 0x35 // Keyboard ` and ~
#define KEY_COMMA 0x36 // Keyboard , and <
#define KEY_DOT 0x37 // Keyboard . and >
#define KEY_SLASH 0x38 // Keyboard / and ?
#define KEY_102ND 0x64 // Keyboard Non-US \ and |

#define X {0, 0}
#define K(x) {0, KEY_##x}
#define S(x) {MOD_SHIFT_LEFT, KEY_##x}

static const uchar keymap[128][2] PROGMEM = {
     X, X, X, X, X, X, X, X,                           // C-@ABCDEFG
     X, X, X, X, X, K(ENTER), X, X,                    // C-HIJKLMNO 
     X, X, X, X, X, X, X, X,                           // C-PQRSTUVW
     X, X, X, X, X, X, X, X,                           // C-XYZ[\]^_
     K(SPACE), S(1), S(2), K(HASHTILDE),               //  !"#
     S(4), S(5), S(7), K(APOSTROPHE),                  // $%&'
     S(9), S(0), S(8), S(EQUAL),                       // ()*+
     K(COMMA), K(MINUS), K(DOT), K(SLASH),             // ,-./
     K(0), K(1), K(2), K(3),                           // 0123
     K(4), K(5), K(6), K(7),                           // 4567
     K(8), K(9), S(SEMICOLON), K(SEMICOLON),           // 89:;
     S(COMMA), K(EQUAL), S(DOT), S(SLASH),             // <=>?
     S(APOSTROPHE), S(A), S(B), S(C),                  // @ABC
     S(D), S(E), S(F), S(G),                           // DEFG
     S(H), S(I), S(J), S(K),                           // HIJK
     S(L), S(M), S(N), S(O),                           // LMNO
     S(P), S(Q), S(R), S(S),                           // PQRS
     S(T), S(U), S(V), S(W),                           // TUVW
     S(X), S(Y), S(Z), K(LEFTBRACE),                   // XYZ[
     K(BACKSLASH), K(RIGHTBRACE), S(6), S(MINUS),      // \]^_
     K(GRAVE), K(A), K(B), K(C),                       // `abc
     K(D), K(E), K(F), K(G),                           // defg
     K(H), K(I), K(J), K(K),                           // hijk
     K(L), K(M), K(N), K(O),                           // lmno
     K(P), K(Q), K(R), K(S),                           // pqrs
     K(T), K(U), K(V), K(W),                           // tuvw
     K(X), K(Y), K(Z), S(LEFTBRACE),                   // xyz{
     S(BACKSLASH), S(RIGHTBRACE), S(BACKSLASH), X      // |}~
};

static const uchar no_key[2] PROGMEM = { 0, 0 };

#define NKEYS 128

static unsigned nkeys = 0;

static uchar keybuf[NKEYS][2];

static int index = -1;

static void decode(char *s) {
     nkeys = 0;
     for (; *s != '\0'; s++) {
          int c = *s;
          uchar m = pgm_read_byte(&keymap[c][0]);
          uchar k = pgm_read_byte(&keymap[c][1]);
          if (nkeys > 0 && k == keybuf[nkeys-1][1]) {
               keybuf[nkeys][0] = 0;
               keybuf[nkeys][1] = 0;
               nkeys++;
          }
          keybuf[nkeys][0] = m;
          keybuf[nkeys][1] = k;
          nkeys++;
     }
     keybuf[nkeys][0] = 0;
     keybuf[nkeys][1] = 0;
     nkeys++;
}

static const uchar *keyPressed(void) {
     if (index >= 0) {
          const uchar *result = keybuf[index++];
          if (index >= nkeys) index = -1;
          return result;
     }

     return no_key;
}

static void buildReport(const uchar *key) {
     reportBuffer[0] = key[0];
     reportBuffer[1] = key[1];
}

uchar usbFunctionSetup(uchar data[8]) {
     usbRequest_t *rq = (void *) data;

     usbMsgPtr = reportBuffer;
     if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS) {
          /* class request type */
          if (rq->bRequest == USBRQ_HID_GET_REPORT) {
               /* wValue: ReportType (highbyte), ReportID (lowbyte) */
               /* we only have one report type, so don't look at wValue */
               buildReport(keyPressed());
               return sizeof(reportBuffer);
          } else if (rq->bRequest == USBRQ_HID_GET_IDLE) {
               usbMsgPtr = &idleRate;
               return 1;
          } else if (rq->bRequest == USBRQ_HID_SET_IDLE) {
               idleRate = rq->wValue.bytes[1];
          }
     } else {
          /* no vendor specific requests implemented */
     }

     return 0;
}

int main(void) {
     int butstate = 0, next;

     hardwareInit();
     sei();

     decode(PASSWORD "\r");

     while (1) {
          usbPoll();
          if (index >= 0) {
               if (usbInterruptIsReady()) {
                    buildReport(keyPressed());
                    usbSetInterrupt(reportBuffer, sizeof(reportBuffer));
               }
          } else {
               next = 0;

#ifdef TINY84
               PORTA &= ~0x4;
               if ((PINA & 0x2) == 0) next = 1;
#endif

#ifdef TINY85
               PORTB &= ~0x8;
               if ((PINB & 0x1) == 0) next = 1;
#endif

               if (!next)
                    butstate = 0;
               else if (!butstate) {
                    butstate = 1;
                    index = 0;
#ifdef TINY84
                    PORTA |= 0x4;
#endif

#ifdef TINY85
                    PORTB |= 0x8;
#endif
               }
          }
     }

     return 0;
}
