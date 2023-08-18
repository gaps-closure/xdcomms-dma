CC          ?= gcc
CFLAGS      ?= -O2 -Wall -Wstrict-prototypes
INCL         = -I ~/gaps/xdcomms-dma/api -I ~/gaps/xdcomms-dma/log -I ~/gaps/xdcomms-dma/crc
LDFLAGS     += -L ~/gaps/xdcomms-dma/api -lxdcomms
SRCS         = $(wildcard *.c) $(wildcard ./6month-demo/*.c)
OBJS        := $(SRCS:%.c=%.o)

all: 	app_req_rep

app_req_rep:	$(OBJS)
	$(CC) $(CFLAGS) -o $@ $(OBJS) -pthread $(LDFLAGS) 

%.o:	%.c
	$(CC) $(CFLAGS) $(INCL) -c $< -o $@

clean:
	$(RM) $(OBJS) app_req_rep
