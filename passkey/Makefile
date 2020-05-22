# Test programs for ATtiny84

all: passkey84.hex passkey85.hex

CGFLAGS = -fno-exceptions -ffunction-sections -fdata-sections 
INCLUDES = -I ../lib -I ../usbdrv -I .
CFLAGS = -g -Os -Wall $(CGFLAGS) $(INCLUDES) $(TARGET)
VPATH = ../lib

passkey84.elf tiny84/%.o: TARGET = -mmcu=attiny84 -DF_CPU=12000000L
passkey85.elf tiny85/%.o: TARGET = -mmcu=attiny85 -DF_CPU=16000000L

tiny84/%.o: CFLAGS += -DTINY84 -I tiny84
tiny85/%.o: CFLAGS += -DTINY85 -I tiny85

%/passkey.o: CFLAGS += -DPASSWORD='"$(PASSWORD)"'

USB = usbdrv.o usbdrvasm.o oddebug.o
PASSKEY = passkey.o mycore.o $(USB)

passkey84.elf: $(PASSKEY:%=tiny84/%)
passkey85.elf: $(PASSKEY:%=tiny85/%)

%.elf:
	avr-gcc $(CFLAGS) -Wl,--gc-sections -o $@ $^ 
	avr-size $@

tiny84/%.o: %.c
	avr-gcc $(CFLAGS) -c $< -o $@

tiny85/%.o: %.c
	avr-gcc $(CFLAGS) -c $< -o $@

tiny84/%.o: %.S
	avr-gcc $(CFLAGS) -c $< -o $@

tiny85/%.o: %.S
	avr-gcc $(CFLAGS) -c $< -o $@

%.hex: %.elf
	avr-objcopy -O ihex -R .eeprom $< $@

VPATH = ../usbdrv ../lib

clean: force
	rm -f tiny*/*.o passkey8?.elf passkey8?.hex

force:

###

