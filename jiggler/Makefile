# Test programs for ATtiny84

all: jiggler84.hex jiggler85.hex

CGFLAGS = -fno-exceptions -ffunction-sections -fdata-sections 
INCLUDES = -I ../lib -I ../usbdrv -I .
CFLAGS = -g -Os -Wall $(CGFLAGS) $(INCLUDES) $(TARGET)
VPATH = ../usbdrv ../lib

jiggler84.elf tiny84/%.o: TARGET = -mmcu=attiny84
jiggler85.elf tiny85/%.o: TARGET = -mmcu=attiny85

tiny84/%.o: CFLAGS += -DTINY84 -DF_CPU=12000000L -I tiny84
tiny85/%.o: CFLAGS += -DTINY85 -DF_CPU=16000000L -I tiny85

USB = usbdrv.o usbdrvasm.o
JIGGLER = jiggler.o mycore.o $(USB)

jiggler84.elf: $(JIGGLER:%=tiny84/%)
jiggler85.elf: $(JIGGLER:%=tiny85/%)

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

clean: force
	rm -f tiny*/*.o jiggler8?.elf jiggler8?.hex

force:
