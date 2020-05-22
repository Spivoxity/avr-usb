#include "mylib.h"

#define ADDR 0x20

#define IOCON_BANK0 0x0a        // Address of IOCON when Bank 0 active
#define IOCON_BANK1 0x15        // Address of IOCON when Bank 1 active

// Addresses when Bank 1 is active
#define GPIOA 0x09
#define IODIRA 0x00
#define GPIOB 0x19
#define IODIRB 0x10

#define CMD_CLEAR_DISPLAY 0x01

#define CMD_RETURN_HOME 0x02

#define CMD_ENTRY_MODE_SET 0x04
  #define ENTRY_RIGHT 0x00
  #define ENTRY_LEFT 0x02
  #define ENTRY_SHIFT_INC 0x01
  #define ENTRY_SHIFT_DEC 0x00

#define CMD_DISPLAY_CTRL 0x08
  #define DISPLAY_ON 0x04
  #define CURSOR_ON 0x02
  #define BLINK_ON 0x01

#define CMD_CURSOR_SHIFT 0x10
  #define DISPLAY_MOVE 0x08
  #define CURSOR_MOVE  0x00
  #define MOVE_RIGHT 0x04
  #define MOVE_LEFT 0x00

#define CMD_FUNCTION_SET 0x20

#define CMD_CGRAM_ADDR 0x40

#define CMD_DDRAM_ADDR 0x80
  #define LINE1 0x00
  #define LINE2 0x40

// Bits in GPIOA
#define E 0x08
#define RW 0x10
#define RS 0x20

#define BKL 7

byte bklight = 0;

void lcd_backlight(byte bkl) {
     bklight = ~bkl & BKL;
     i2c_write(ADDR, GPIOA, bklight);
}

static void lcd_write(byte cmd, byte bits) {
  i2c_write(ADDR, GPIOB, cmd);
  i2c_write(ADDR, GPIOA, bklight|bits|E);
  i2c_write(ADDR, GPIOA, bklight|bits);
}

void lcd_clear(void) {
     lcd_write(CMD_CLEAR_DISPLAY, 0);
}

void lcd_home(void) {
     lcd_write(CMD_RETURN_HOME, 0);
}

void lcd_char(char c) {
     lcd_write(c, RS);
}

void lcd_message(const char *s) {
     int i;
     for (i = 0; s[i] != '\0'; i++) 
          lcd_char(s[i]);
}

void lcd_printf(const char *fmt, ...) {
     va_list va;
     va_start(va, fmt);
     do_print(lcd_char, fmt, va);
     va_end(va);
}

void lcd_init(void) {
     i2c_write(ADDR, IOCON_BANK1, 0);
  
     static byte init[22] = {
          0b11111111,   // IODIRA    Initially all as inputs
          0b11111111,   // IODIRB
          0,            // IPOLA
          0,            // IPOLB
          0,            // GPINTENA  Disable interrupt-on-change
          0,            // GPINTENB
          0,            // DEFVALA
          0,            // DEFVALB
          0,            // INTCONA
          0,            // INTCONB
          0,            // IOCON
          0,            // IOCON
          0,            // GPPUA
          0,            // GPPUB
          0,            // INTFA
          0,            // INTFB
          0,            // INTCAPA
          0,            // INTCAPB
          0,            // GPIOA
          0,            // GPIOB
          0,            // OLATA     0 on all outputs; side effect of
          0             // OLATB     turning on R+G+B backlight LEDs.
     };

     // Initialise to known state
     i2c_write_block(ADDR, 0, init, 22);
     
     // Now switch addressing mode to Bank 1, non-sequential
     i2c_write(ADDR, IOCON_BANK0, 0b10100000);

     // Enable outputs: backlight should come on
     i2c_write(ADDR, IODIRA, 0xc0);
     i2c_write(ADDR, IODIRB, 0x00);

     /* We need to configure at least the pins as inputs initially,
        so as to avoid a state where the R/W pin on the LCD is asserted
        but the data pins are configured as outputs by the MCP23017.
        Once the output latch contains a zero for R/W, it's safe to
        reconfigure the pins as outputs. */
        
     lcd_write(0x30, 0);        // Voodoo initalisation sequence
     lcd_write(0x30, 0);
     lcd_write(0x30, 0);
     lcd_write(0x38, 0);        // 2 lines of 5x8 characters
     lcd_clear();
     lcd_write(CMD_CURSOR_SHIFT | CURSOR_MOVE | MOVE_RIGHT, 0);
     lcd_write(CMD_ENTRY_MODE_SET | ENTRY_LEFT | ENTRY_SHIFT_DEC, 0);
     lcd_write(CMD_DISPLAY_CTRL | DISPLAY_ON, 0);
}
