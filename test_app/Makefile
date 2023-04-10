CC          ?= gcc
CFLAGS      ?= -O2 -Wall -Wstrict-prototypes
INCL         = -I ../log -I ../crc

all: 	app_req_rep

app_req_rep:	app_req_rep.c app_req_rep.o
# HAL veriosn:
#	$(CC) $(CFLAGS) -o $@ app_req_rep.o ../api/xdcomms.o ../appgen/6month-demo/gma.o ../log/log.o ../appgen/6month-demo/float.o  -l zmq
# DMA version:
	$(CC) $(CFLAGS) -o $@ app_req_rep.o ../api/xdcomms_dma.o ../log/log.o ../crc/crc.o ./6month-demo/gma.o ./6month-demo/float.o -pthread

%.o:	%.c
	$(CC) $(CFLAGS) $(INCL) -c $< -o $@

clean:
	rm -f *.o app_req_rep
