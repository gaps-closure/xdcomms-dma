CC          ?= gcc
CFLAGS      ?= -O2 -Wall -Wstrict-prototypes
INCL         = -I ../log -I ../crc
OBJS         = $(wildcard *.o) $(wildcard ../api/*.o) $(wildcard ../log/*.o) $(wildcard ../crc/*.o) $(wildcard ./6month-demo/*.o)

all: 	app_req_rep

app_req_rep:	app_req_rep.c $(OBJS)
# HAL veriosn:
#	$(CC) $(CFLAGS) -o $@ app_req_rep.o ../api/xdcomms.o ../appgen/6month-demo/gma.o ../log/log.o ../appgen/6month-demo/float.o  -l zmq
# DMA version:
	$(CC) $(CFLAGS) -o $@ $(OBJS) -pthread

%.o:	%.c
	$(CC) $(CFLAGS) $(INCL) -c $< -o $@

clean:
	rm -f *.o app_req_rep
