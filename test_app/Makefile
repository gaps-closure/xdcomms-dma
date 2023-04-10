CC          ?= gcc
CFLAGS      ?= -O2 -Wall -Wstrict-prototypes
INCL         = -I ../log -I ../crc
SRCS         = $(wildcard *.c) $(wildcard ../log/*.c) $(wildcard ../crc/*.c) $(wildcard ../api/*.c) $(wildcard ./6month-demo/*.c)
OBJS        := $(SRCS:%.c=%.o)

all: 	app_req_rep

app_req_rep:	$(OBJS)
# HAL veriosn:
#	$(CC) $(CFLAGS) -o $@ app_req_rep.o ../api/xdcomms.o ../appgen/6month-demo/gma.o ../log/log.o ../appgen/6month-demo/float.o  -l zmq
# DMA version:
	$(CC) $(CFLAGS) -o $@ $(OBJS) -pthread

%.o:	%.c
	$(CC) $(CFLAGS) $(INCL) -c $< -o $@

clean:
	rm -f $(OBJS) app_req_rep
