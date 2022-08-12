CC	?= gcc
CFLAGS	?= -O2 -Wall -Wstrict-prototypes
SRCS	=$(wildcard *.c)
OBJS	=$(SRCS:.c=.o)

all: $(OBJS)

%.o:%.c
	$(CC) $(CFLAGS) -fPIC -c $< -o $@

clean:
	rm -f *.o
