
all: test test2

#dshot.pio.h: dshot.pio
#	pioasm dshot.pio dshot.pio.h

test: dshot.pio.h test.c motor-dshot.c piolib.c pio_rp1.c
	gcc -O2 -Iinclude -Wall -o test test.c motor-dshot.c piolib.c pio_rp1.c

test2: dshot.pio.h test2.c motor-dshot.c piolib.c pio_rp1.c
	gcc -O2 -Iinclude -Wall -o test2 test2.c motor-dshot.c piolib.c pio_rp1.c

clean: always
	rm -f *~ test test2

run: always
	sudo ./test 19


.PHONY: always



