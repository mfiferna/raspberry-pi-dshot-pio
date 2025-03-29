all: test test2 libdshot.so dshot_wrapper

#dshot.pio.h: dshot.pio
#	pioasm dshot.pio dshot.pio.h

test: dshot.pio.h test.c motor-dshot.c piolib.c pio_rp1.c
	gcc -O2 -Iinclude -Wall -o test test.c motor-dshot.c piolib.c pio_rp1.c

test2: dshot.pio.h test2.c motor-dshot.c piolib.c pio_rp1.c
	gcc -O2 -Iinclude -Wall -o test2 test2.c motor-dshot.c piolib.c pio_rp1.c

libdshot.so: motor-dshot.c piolib.c pio_rp1.c
	gcc -shared -fPIC -O2 -Iinclude -Wall -o libdshot.so motor-dshot.c piolib.c pio_rp1.c

dshot_wrapper: dshot_wrapper.py
	python3 dshot_wrapper.py build_ext --inplace

clean: always
	rm -f *~ test test2 libdshot.so dshot_wrapper

run: always
	sudo ./test 19

.PHONY: always
