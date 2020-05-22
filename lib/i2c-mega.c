#include "mylib.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>

#define NULL 0

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#ifndef TWI_FREQ
#define TWI_FREQ 100000L
#endif

static volatile bool busy;

static volatile uint8_t twi_slarw;

static uint8_t command;
static uint8_t *buffer;
static volatile uint8_t bufferIndex;
static volatile uint8_t bufferLength;

static volatile uint8_t twi_error;

void i2c_init(void) {
     busy = false;
  
     // activate internal pullups for twi.
     sbi(PORTC, 4);
     sbi(PORTC, 5);

     // initialize twi prescaler and bit rate
     cbi(TWSR, TWPS0);
     cbi(TWSR, TWPS1);
     TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;

     /* twi bit rate formula from atmega128 manual pg 204
        SCL Frequency = CPU Clock Frequency / (16 + (2 * TWBR))
        note: TWBR should be 10 or higher for master mode
        It is 72 for a 16mhz Wiring board with 100kHz TWI */

     // enable twi module, acks, and twi interrupt
     TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
}

static uint8_t i2c_read_stuff(uint8_t addr, uint8_t* data, uint8_t length) {
     // reset error state (0xFF.. no error occured)
     twi_error = 0xFF;

     // initialize buffer iteration vars
     buffer = data;
     bufferIndex = 0;
     bufferLength = length;

     // build sla+w, slave device address + w bit
     twi_slarw = (addr << 1) | TW_READ;

     // send start condition
     busy = true;
     TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);

     // wait for read operation to complete
     while (busy) { }

     return bufferIndex;
}

void i2c_write_block(uint8_t addr, uint8_t cmd, uint8_t* data, uint8_t n) {
     busy = true;
     // reset error state (0xFF.. no error occured)
     twi_error = 0xFF;

     // initialize buffer iteration vars
     command = cmd;
     buffer = data;
     bufferIndex = 0;
     bufferLength = n;
  
     // build sla+w, slave device address + w bit
     twi_slarw = (addr << 1) | TW_WRITE;
  
     // send start condition, enable interrupt
     TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE) | _BV(TWSTA);

     // wait for write operation to complete
     while (busy) { }
  
/*
     if (twi_error == 0xFF)
          return 0;	// success
     else if (twi_error == TW_MT_SLA_NACK)
          return 2;	// error: address send, nack received
     else if (twi_error == TW_MT_DATA_NACK)
          return 3;	// error: data send, nack received
     else
          return 4;	// other twi error
*/
}

/* 
 * Function twi_reply
 * Desc     sends byte or readys receive line
 * Input    ack: byte indicating to ack or to nack
 * Output   none
 */
static void twi_reply(bool ack) {
     // transmit master read ready signal, with or without ack
     if (ack)
          TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
     else
	  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT);
}

/* 
 * Function twi_stop
 * Desc     relinquishes bus master status
 * Input    none
 * Output   none
 */
static void twi_stop(void)
{
  // send stop condition
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTO);

  // wait for stop condition to be exectued on bus
  // TWINT is not set after a stop condition!
  while(TWCR & _BV(TWSTO)){
    continue;
  }

  // update twi state
  busy = false;
}

/* 
 * Function twi_releaseBus
 * Desc     releases bus control
 * Input    none
 * Output   none
 */
void twi_releaseBus(void) {
     // release bus
     TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT);

     // update twi state
     busy = false;
}

ISR(TWI_vect) {
     switch(TW_STATUS){
     // All Master
     case TW_START:     // sent start condition
     case TW_REP_START: // sent repeated start condition
          // copy device address and r/w bit to output register and ack
          TWDR = twi_slarw;
          twi_reply(1);
          break;

     // Master Transmitter
     case TW_MT_SLA_ACK:  // slave receiver acked address
          // send command byte
          TWDR = command;
          twi_reply(1);
          break;
     case TW_MT_DATA_ACK: // slave receiver acked data
          // if there is data to send, send it, otherwise stop 
          if (bufferIndex >= bufferLength)
               twi_stop();
          else {
               // copy data to output register and ack
               TWDR = buffer[bufferIndex++];
               twi_reply(1);
          }
          break;
    case TW_MT_SLA_NACK:  // address sent, nack received
         twi_error = TW_MT_SLA_NACK;
         twi_stop();
         break;
    case TW_MT_DATA_NACK: // data sent, nack received
         twi_error = TW_MT_DATA_NACK;
         twi_stop();
         break;
    case TW_MT_ARB_LOST: // lost bus arbitration
         twi_error = TW_MT_ARB_LOST;
         twi_releaseBus();
         break;

    // Master Receiver
    case TW_MR_DATA_ACK: // data received, ack sent
         // put byte into buffer
         buffer[bufferIndex++] = TWDR;
         // Fall through
    case TW_MR_SLA_ACK:  // address sent, ack received
         // prepare ack if more bytes are expected, otherwise nack
         if(bufferIndex >= bufferLength-1)
              // Prepare NACK in response to last byte
              twi_reply(0);
         else
              twi_reply(1);
         break;
    case TW_MR_DATA_NACK: // data received, nack sent
         // put final byte into buffer
         buffer[bufferIndex++] = TWDR;
         twi_stop();
         break;
    case TW_MR_SLA_NACK: // address sent, nack received
         twi_stop();
         break;
    // TW_MR_ARB_LOST handled by TW_MT_ARB_LOST case

    // Slave Receiver
    case TW_SR_SLA_ACK:   // addressed, returned ack
    case TW_SR_GCALL_ACK: // addressed generally, returned ack
    case TW_SR_ARB_LOST_SLA_ACK:   // lost arbitration, returned ack
    case TW_SR_ARB_LOST_GCALL_ACK: // lost arbitration, returned ack
    case TW_SR_DATA_ACK:       // data received, returned ack
    case TW_SR_GCALL_DATA_ACK: // data received generally, returned ack
    case TW_SR_STOP: // stop or repeated start condition received
    case TW_SR_DATA_NACK:       // data received, returned nack
    case TW_SR_GCALL_DATA_NACK: // data received generally, returned nack
         break;
    
    // Slave Transmitter
    case TW_ST_SLA_ACK:          // addressed, returned ack
    case TW_ST_ARB_LOST_SLA_ACK: // arbitration lost, returned ack
    case TW_ST_DATA_ACK: // byte sent, ack returned
    case TW_ST_DATA_NACK: // received nack, we are done 
    case TW_ST_LAST_DATA: // received ack, but we are done already!
         break;

    // All
    case TW_NO_INFO:   // no state information
         break;
    case TW_BUS_ERROR: // bus error, illegal stop/start
         twi_error = TW_BUS_ERROR;
         twi_stop();
         break;
  }
}


// INTERFACE ROUTINES

byte i2c_read_byte(byte dev) {
     byte buf;
     i2c_read_stuff(dev, &buf, 1);
     return buf;
}

void i2c_write_byte(byte dev, byte val) {
     i2c_write_block(dev, val, NULL, 0);
}

byte i2c_read(byte dev, byte cmd) {
     i2c_write_byte(dev, cmd);
     return i2c_read_byte(dev);
}

void i2c_write(byte dev, byte cmd, byte val) {
     i2c_write_block(dev, cmd, &val, 1);
}

void i2c_read_block(byte dev, byte cmd, byte *buf, byte n) {
     i2c_write_byte(dev, cmd);
     i2c_read_stuff(dev, buf, n);
}
