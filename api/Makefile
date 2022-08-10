CC           = gcc
INCL         = -I ../log -I ../crc
CFLAGS      ?= -O2 -Wall -Wstrict-prototypes
SRCS         = $(wildcard *.c) $(wildcard ../log/*.c) $(wildcard ../crc/*.c)
OBJS        := $(SRCS:%.c=%.o)

all:	libxdcomms.so

libxdcomms.so: $(OBJS)
	$(CC) $(CFLAGS) -shared -pthread -o $@ $^

%.o:%.c
	$(CC) $(CFLAGS) $(INCL) -fPIC -c $< -o $@

clean: 
	rm -f $(OBJS) *.so

