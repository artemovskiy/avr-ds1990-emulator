SOURCES = $(wildcard *.c)
OBJECTS = $(patsubst %.c, build/%.o, $(SOURCES))

build/%.o: %.c
	avr-gcc -Os -DF_CPU=16000000 -mmcu=atmega168 -c $< -o $@

list:
	echo $(OBJECTS)

prep: $(OBJECTS)
	avr-gcc -mmcu=atmega168 $(OBJECTS) -o build/main.elf
	avr-objcopy -O ihex -R .eeprom build/main.elf build/main.hex
	
prog: prep
	avrdude -c usbasp-clone -p m168 -U flash:w:build/main.hex:a 

clean:
	rm $(OBJECTS)