#include "mylib.h"
#include <util/delay.h>

void delay_ms(unsigned ms) {
     while (ms-- > 0)
          _delay_ms(1);
}
