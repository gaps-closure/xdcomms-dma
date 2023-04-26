APP     = xdcomms
NAME   := xdcomms
MAJOR   = 1.0
MINOR   = 1
VERSION = $(MAJOR).$(MINOR)

INCL         = -I ../log -I ../crc 
CFLAGS      ?= -O2 -Wall -Wstrict-prototypes
SRCS        := $(wildcard *.c) $(wildcard ../log/*.c) $(wildcard ../crc/*.c)
OBJS        := $(SRCS:%.c=%.o)
XX := $(wildcard *.c)
$(info XX="$(XX)")
$(info OBJS="$(OBJS)")
$(info SRCS="$(SRCS)")


all:	lib$(NAME).so

lib$(NAME).so.$(VERSION): $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) -shared -pthread -Wl,-soname,lib$(NAME).so.$(MAJOR) -o lib$(NAME).so.$(VERSION)

lib$(NAME).so: lib$(NAME).so.$(VERSION)
	rm -f lib$(NAME).so.$(MAJOR) lib$(NAME).so
	ln -s lib$(NAME).so.$(VERSION) lib$(NAME).so.$(MAJOR)
	ln -s lib$(NAME).so.$(MAJOR) lib$(NAME).so

%.o:%.c
	$(CC) $(CFLAGS) $(INCL) -fPIC -c $< -o $@

clean: 
	rm -f $(OBJS) *.so.* *.so

