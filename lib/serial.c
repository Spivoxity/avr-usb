#include "mylib.h"
#include <avr/io.h>

void serial_init(int baud) {
     uint16_t baud_setting;
     bool use_u2x = true;

#if F_CPU == 16000000UL
  // hardcoded exception for compatibility with the bootloader shipped
  // with the Duemilanove and previous boards and the firmware on the 8U2
  // on the Uno and Mega 2560.
     if (baud == 57600)
          use_u2x = false;
#endif

try_again:
     if (use_u2x) {
          UCSR0A = 1 << U2X0;
          baud_setting = (F_CPU / 4 / baud - 1) / 2;
     } else {
          UCSR0A = 0;
          baud_setting = (F_CPU / 8 / baud - 1) / 2;
     }
  
     if ((baud_setting > 4095) && use_u2x) {
          use_u2x = false;
          goto try_again;
     }

     // assign the baud_setting, a.k.a. ubbr (USART Baud Rate Register)
     UBRR0H = baud_setting >> 8;
     UBRR0L = baud_setting;

     sbi(UCSR0B, TXEN0);
}

static void putch(char ch) {
     // Send and wait
     UDR0 = ch;
     while (!(UCSR0A & _BV(UDRE0))) { }
}

static void serial_putch(char ch) {
     if (ch == '\n') putch('\r');
     putch(ch);
}

void serial_print(const char *message) {
     int i;
    
     for (i = 0; message[i] != '\0'; i++)
          serial_putch(message[i]);
}

void printk(const char *fmt, ...) {
     va_list va;
     va_start(va, fmt);
     do_print(serial_putch, fmt, va);
     va_end(va);
}
