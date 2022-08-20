obj-m += sue_donimous.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
	cp sue_donimous.ko ..

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
