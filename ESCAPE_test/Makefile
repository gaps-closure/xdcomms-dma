.PHONY: clean help

APPS	:= memtest
MEMTEST	:= memtest

IDIR	:= include
CC	:= gcc
CFLAGS	:= -g -I$(IDIR) -O3 -Wall -Werror -std=gnu99
#LIBS	:= -lpthread -lnuma -ljemalloc
ODIR	:= ./

_OBJ_UP	:= memtest.o
OBJ_UP	:= $(patsubst %,$(ODIR)/%,$(_OBJ_UP))


all: $(APPS)


$(ODIR)/%.o: %.c $(DEPS)
		$(CC) -c -o $@ $< $(CFLAGS)

$(ODIR)/%.s: %.c $(DEPS)
		$(CC) -S -o $@ $< $(CFLAGS)


$(PGACCESS): $(OBJ_UP)
		$(CC) -o $@ $^ $(LIBS)


clean:
		rm -f $(ODIR)/*.o $(ODIR)/*.s $(APPS)


help:
		@echo "usage: make <target>"
			@echo ""
				@echo "targets: $(APPS)"
					@echo ""

