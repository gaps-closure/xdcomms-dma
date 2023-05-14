CC          ?= gcc
CFLAGS      ?= -O2 -Wall -Wstrict-prototypes
INCL         = -I ../log -I ../crc
SRCS         = $(wildcard *.c) $(wildcard ../log/*.c) $(wildcard ../crc/*.c) $(wildcard ../api/*.c) $(wildcard ./6month-demo/*.c)
OBJS        := $(SRCS:%.c=%.o)

all: 	app_req_rep

app_req_rep:	$(OBJS)
	$(CC) $(CFLAGS) -o $@ $(OBJS) -pthread -ljson-c 

%.o:	%.c
	$(CC) $(CFLAGS) $(INCL) -c $< -o $@

clean:
	rm -f $(OBJS) app_req_rep
