# USBtiny programmer using ATtiny84

all: usbtiny.hex

MCU = attiny84
ISP = $(USBTINY)
CGFLAGS = -fno-exceptions -ffunction-sections -fdata-sections 
DEFINES = -DF_CPU=12000000L
INCLUDES = -I . -I ../lib -I ../usbdrv
CFLAGS = -c -g -Os -Wall -mmcu=$(MCU) $(CGFLAGS) $(DEFINES) $(INCLUDES)
VPATH = ../usbdrv

usbtiny.elf: usbtiny.o usbdrv.o usbdrvasm.o

usbdrv.o: usbconfig.h

set-fuses: force
	$(AVRDUDE) -U lfuse:w:0xff:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m 

include ../lib/Makerules
