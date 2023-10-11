APP     = xdcomms
NAME   := xdcomms
MAJOR   = 1.0
MINOR   = 1
VERSION = $(MAJOR).$(MINOR)

INCL         = -I ../log -I ../crc -I ../tiny-json
CFLAGS      ?= -Wall -Wstrict-prototypes -g #-O2
SRCS        := $(wildcard *.c) $(wildcard ../log/*.c) $(wildcard ../crc/*.c) $(wildcard ../tiny-json//*.c)
OBJS        := $(SRCS:%.c=%.o)
#$(info OBJS="$(OBJS)")

all:	lib$(NAME).so

lib$(NAME).so.$(VERSION): $(OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJS) -shared -pthread -Wl,-soname,lib$(NAME).so.$(MAJOR) -o lib$(NAME).so.$(VERSION)

lib$(NAME).so: lib$(NAME).so.$(VERSION)
	rm -f lib$(NAME).so.$(MAJOR) lib$(NAME).so
	ln -s lib$(NAME).so.$(VERSION) lib$(NAME).so.$(MAJOR)
	ln -s lib$(NAME).so.$(MAJOR) lib$(NAME).so

%.o:	%.c
	$(CC) $(CFLAGS) $(INCL) -fPIC -c $< -o $@

clean: 
	$(RM) $(OBJS) *.so.* *.so

