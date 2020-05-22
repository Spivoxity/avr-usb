#ifndef mylib_h
#define mylib_h

#include <inttypes.h>
#include <stdarg.h>

typedef uint8_t byte;

typedef uint16_t word;

typedef uint8_t bool;
#define true 1
#define false 0

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// Timing

void delay_ms(unsigned ms);


// PRINTK

char *itoa(int n);

typedef void (*prfun)(char);

void do_print(prfun out, const char *fmt, va_list va);


// I2C

void i2c_init(void);

byte i2c_read_byte(byte dev);
void i2c_write_byte(byte dev, byte val);

byte i2c_read(byte dev, byte cmd);
void i2c_write(byte dev, byte cmd, byte val);

void i2c_read_block(byte dev, byte cmd, byte *data, byte n);
void i2c_write_block(byte dev, byte cmd, byte *data, byte n);

// My LCD board

#define RED 1
#define GREEN 2
#define BLUE 4

void lcd_init(void);
void lcd_clear(void);
void lcd_home(void);
void lcd_backlight(byte colour);
void lcd_message(const char *msg);
void lcd_printf(const char *fmt, ...);

// Simple serial output

void serial_init(int baud);
void serial_print(const char *message);
void printk(const char *fmt, ...);

#endif
