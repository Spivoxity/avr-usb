#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>
#include "mylib.h"

/* I2C over USI */

#define I2C_NO_ACK_ON_ADDRESS   0x01  // Slave did not acknowledge address
#define I2C_NO_ACK_ON_DATA      0x02  // Slave did not acknowledge data
#define I2C_MISSING_START_CON   0x03  // Start Condition not detected on bus
#define I2C_MISSING_STOP_CON    0x04  // Stop Condition not detected on bus

#if defined(__AVR_ATtiny25__) | defined(__AVR_ATtiny45__) \
     | defined(__AVR_ATtiny85__)
#define DDR_USI DDRB
#define PORT_USI PORTB
#define PIN_USI PINB
#define PORT_SDA PORTB0
#define PORT_SCL PORTB2
#define PIN_SDA PINB0
#define PIN_SCL PINB2
#endif

#if defined(__AVR_ATtiny84__) | defined(__AVR_ATtiny44__)
#define DDR_USI DDRA
#define PORT_USI PORTA
#define PIN_USI PINA
#define PORT_SDA PORTA6
#define PORT_SCL PORTA4
#define PIN_SDA PINA6
#define PIN_SCL PINA4
#endif

// For use with _delay_us() for 100kHz bus speed
#define T2    5 		// >4,7us
#define T4    4 		// >4,0us

#define TRUE  1
#define FALSE 0
#define NULL 0

uint8_t i2c_error;


/* LOW LEVEL ROUTINES */

/* i2c_init -- initialize for i2c */
void i2c_init(void) {
     // Enable pullups
     PORT_USI |= _BV(PIN_SDA);
     PORT_USI |= _BV(PIN_SCL);
  
     // Set SDA and SCL as outputs
     DDR_USI |= _BV(PIN_SCL);
     DDR_USI |= _BV(PIN_SDA);
  
     // Initialize data register
     USIDR = 0xFF;

     // Disable interrupts, set to i2c mode, software strobe
     USICR = _BV(USIWM1)|_BV(USICS1)|_BV(USICLK);

     // Clear flags
     USISR = _BV(USISIF)|_BV(USIOIF)|_BV(USIPF)|_BV(USIDC);
}

/* start -- generate a start signal */
static uint8_t start(void) {
     /* Release SCL and wait for it to go high */
     PORT_USI |= _BV(PIN_SCL);
     while (!(PORT_USI & _BV(PIN_SCL))) {}
     _delay_us(T2);

     /* Generate start condition */
     PORT_USI &= ~_BV(PIN_SDA); // Pull SDA low.
     _delay_us(T4);                         
     PORT_USI &= ~_BV(PIN_SCL); // Pull SCL low.
     PORT_USI |= _BV(PIN_SDA);  // Release SDA.

     /* Check result */
     if (!(USISR & _BV(USISIF))) {
          i2c_error = I2C_MISSING_START_CON;  
          return FALSE;
     }

     return TRUE;
}

/* stop -- release the bus */
static uint8_t stop(void) {
     /* Pull SDA low and release SCL */
     PORT_USI &= ~_BV(PIN_SDA);
     PORT_USI |= _BV(PIN_SCL);

     /* Wait for SCL to go high */
     while (!(PIN_USI & _BV(PIN_SCL))) {}
     _delay_us(T4);

     /* Release SDA */
     PORT_USI |= _BV(PIN_SDA);
     _delay_us(T2);
  
     /* Check the result */
     if (!(USISR & _BV(USIPF))) {
          i2c_error = I2C_MISSING_STOP_CON;    
          return FALSE;
     }
     
     return TRUE;
}

/* Flag values to count 8 bits or 1 bit */
#define FLAGS8 (_BV(USISIF)|_BV(USIOIF)|_BV(USIPF)|_BV(USIDC))
#define FLAGS1 (_BV(USISIF)|_BV(USIOIF)|_BV(USIPF)|_BV(USIDC)|(0xE<<USICNT0))

/* transfer -- shift data through the interface */
static uint8_t transfer(uint8_t flags) {
     uint8_t data;

     /* Set USISR from supplied flags */
     USISR = flags;

     do { 
          _delay_us(T2);

          /* Generate positve edge and wait for SCL to go high. */
          USICR = _BV(USIWM1)|_BV(USICS1)|_BV(USICLK)|_BV(USITC);
          while (!(PIN_USI & _BV(PIN_SCL))) {}

          /* Generate negative edge */
          _delay_us(T4);
          USICR = _BV(USIWM1)|_BV(USICS1)|_BV(USICLK)|_BV(USITC);
     } while (!(USISR & _BV(USIOIF)));
  
     /* Read out data, release SDA and re-enable as output */
     _delay_us(T2);
     data = USIDR;
     USIDR = 0xFF;
     DDR_USI |= _BV(PIN_SDA);

     return data;
}

static uint8_t write_byte(uint8_t x) {
     uint8_t ack;

     /* Pull SCL low, load sdata, count clocks */
     PORT_USI &= ~_BV(PIN_SCL);
     USIDR = x;
     transfer(FLAGS8);
      
     /* Read and verify (N)ACK from slave */
     DDR_USI  &= ~_BV(PIN_SDA);
     ack = transfer(FLAGS1);
     if (ack & 1)
          return FALSE;
     return TRUE;
}

static uint8_t read_byte(uint8_t ack) {
     uint8_t data;

     /* Enable SDA as input and shift in the byte */
     DDR_USI &= ~_BV(PIN_SDA);
     data = transfer(FLAGS8);

     /* Send ACK or (for the last byte) NACK */
     USIDR = ack;
     transfer(FLAGS1);

     return data;
}

/*  i2c_write_block -- general write command */
void i2c_write_block(uint8_t dev, uint8_t cmd, uint8_t *msg, uint8_t n) {
     int i;

     i2c_error = 0;

     if (!start()) return;

     if (!write_byte(dev << 1)) {
          i2c_error = I2C_NO_ACK_ON_ADDRESS;
          return;
     }

     if (!write_byte(cmd)) {
          i2c_error = I2C_NO_ACK_ON_DATA;
          return;
     }

     for (i = 0; i < n; i++) {
          if (!write_byte(msg[i])) {
               i2c_error = I2C_NO_ACK_ON_DATA;
               return;
          }
     }

     stop();
}

/*  i2c_read_stuff -- read entire message */
static void i2c_read_stuff(uint8_t dev, uint8_t *msg, uint8_t n) {
     int i;

     i2c_error = 0;

     if (!start()) return;

     if (!write_byte((dev << 1)|1)) {
          i2c_error = I2C_NO_ACK_ON_ADDRESS;
          return;
     }

     for (i = 0; i < n-1; i++)
          msg[i] = read_byte(0x00);
     msg[n-1] = read_byte(0xff);

     stop();
}



/* INTERFACE ROUTINES */

uint8_t i2c_read_byte(uint8_t dev) {
     uint8_t buf;
     i2c_read_stuff(dev, &buf, 1);
     return buf;
}

void i2c_write_byte(uint8_t dev, uint8_t val) {
     i2c_write_block(dev, val, NULL, 0);
}

uint8_t i2c_read(uint8_t dev, uint8_t cmd) {
     i2c_write_byte(dev, cmd);
     return i2c_read_byte(dev);
}

void i2c_write(uint8_t dev, uint8_t cmd, uint8_t val) {
     i2c_write_block(dev, cmd, &val, 1);
}

void i2c_read_block(uint8_t dev, uint8_t cmd, uint8_t *buf, uint8_t n) {
     i2c_write_byte(dev, cmd);
     i2c_read_stuff(dev, buf, n);
}
