#include <stdarg.h>
#include "mylib.h"

char *itoa(int n) {
     unsigned t, i;
     byte neg = 0;
     static char buf[10];

     if (n >= 0)
          t = n;
     else {
          neg = 1; t = -n;
     }

     i = 9; buf[i] = '\0';
     do {
          buf[--i] = t%10 + '0';
          t /= 10;
     } while (t > 0);

     if (neg) buf[--i] = '-';

     return &buf[i];
}

static char hex[] = "0123456789abcdef";

void do_print(prfun out, const char *fmt, va_list va) {
     int i, j;
     char *buf;

     for (i = 0; fmt[i] != '\0'; i++) {
          if (fmt[i] != '%')
               out(fmt[i]);
          else {
               switch (fmt[++i]) {
               case 'd':
                    buf = itoa(va_arg(va, int));
                    for (j = 0; buf[j] != '\0'; j++)
                         out(buf[j]);
                    break;
               case 'x':
                    j = va_arg(va, int);
                    out(hex[(j>>4)&0xf]);
                    out(hex[j&0xf]);
                    break;
               case 's':
                    buf = va_arg(va, char *);
                    for (j = 0; buf[j] != '\0'; j++)
                         out(buf[j]);
                    break;
               default:
                    out(fmt[i]);
               }
          }
     }
}
                    
