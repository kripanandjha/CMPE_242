# ----------------------------------------------------------------------------
# Makefile for building tapp
#
# Copyright 2010 FriendlyARM (http://www.arm9.net/)
#

ifndef DESTDIR
DESTDIR			   ?= /tmp/
endif

CFLAGS				= -Wall -O2 -std=gnu99 -pthread
LDFLAGS				= -lm -lglfw -lGL -lGLU -lglut
CC					= arm-linux-gnueabihf-gcc
INSTALL				= install

TARGET				= i2c


all: $(TARGET)

i2c: lsm303_jetson.o
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)


install: $(TARGET)
	$(INSTALL) $^ $(DESTDIR)/usr/bin

clean distclean:
	rm -rf *.o $(TARGET)


# ----------------------------------------------------------------------------

.PHONY: $(PHONY) install clean distclean

# End of file
# vim: syntax=make

