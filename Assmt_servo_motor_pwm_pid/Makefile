# ----------------------------------------------------------------------------
# Makefile for building tapp
#
# Copyright 2010 FriendlyARM (http://www.arm9.net/)
#

ifndef DESTDIR
DESTDIR			   ?= /tmp/FriendlyARM/mini6410/rootfs
endif

CFLAGS				= -Wall -O2
CC					= arm-linux-g++
INSTALL				= install

TARGET				= pwm_motor


all: $(TARGET)

pwm_motor: armsvr_pid_pwm_i2c_uart.o
	$(CC) $(CFLAGS) $^ -lm -o $@ -std=gnu++0x

armsvr_pid_pwm_i2c_uart.o: armsvr_pid_pwm_i2c_uart.cpp
	$(CC) $(CFLAGS) $^ -lm -c -o $@ -std=gnu++0x

install: $(TARGET)
	$(INSTALL) $^ $(DESTDIR)/usr/bin

clean distclean:
	rm -rf *.o $(TARGET)


# ----------------------------------------------------------------------------

.PHONY: $(PHONY) install clean distclean

# End of file
# vim: syntax=make

