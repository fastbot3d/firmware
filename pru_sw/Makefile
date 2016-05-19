#
# Just a minimal Makefile for now to install the most basic components
#
# Currently installs the assembler and the app loader library
#
PREFIX?=/usr/local

all:
	cd utils/pasm_src && ./linuxbuild
	cd src && CROSS_COMPILE=$(CROSS_COMPILE) make
	cd test && CROSS_COMPILE=$(CROSS_COMPILE) make

clean:
	rm -f utils/pasm
	cd src && CROSS_COMPILE=$(CROSS_COMPILE) make clean
	cd test && CROSS_COMPILE=$(CROSS_COMPILE) make clean

install:
	install -m 0755 -d $(DESTDIR)$(PREFIX)/bin
	install -m 0755 utils/pasm $(DESTDIR)$(PREFIX)/bin
