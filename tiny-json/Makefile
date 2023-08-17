CC          ?= gcc
CFLAGS      ?= -O2 -Wall -Wstrict-prototypes

all:	tiny-json.o

%.o:	%.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm *.o
