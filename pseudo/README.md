# The Sue Donimous driver

This directory is a placehodler for a pseudo device driver for testing
applications relying on dma-proxy kernel module without requiring the hardware
device.

This is developed for easing testing of the CLOSURE xdcomms-dma without the
hassle of having to repeatedly building the firware using cross-compilers
based on Petalinux,  flashing it onto the ZCU102+ board, booting, testing, and
copying over the logs.

Each test can now be doen in seconds and in paralle on multiple
general-purpose machines, thereby speeding up development, testing, and
debugging.

## Building and Installing Module on Ubuntu Linux 

```bash
sudo apt install build-essential linux-headers-`uname -r`
git clone git@github.com:gaps-closure/xdcomms-dma.git
cd xdcomms-dma/pseudo
git checkout rk
make

sudo ./sue_donimous_load

# rx0 and tx0 have an easter egg
cat /dev/sue_donimous_rx0 
echo "All your cats are belong to me." > /dev/sue_donimous_tx0
cat /dev/sue_donimous_rx0

sudo ./sue_donimous_unload
dmesg
```

## Testing

```
cd  ..\test
make
./run.sh
dmesg
```

## References

** Simple module that logs on intall/remove **
Robert W. Oliver II,  Writing a simple linux kernel module. 
[URL](https://blog.sourcerer.io/writing-a-simple-linux-kernel-module-d9dc3762c234)

** Simple read-only character device that returns ASCII art for a random live/dead cat"
Ole Andreas W. Lyngvær, Writing a pseudo-device driver on Linux.
[URL](https://lyngvaer.no/log/writing-pseudo-device-driver)

** Simple character device with ioctl support **
Ole Andreas W. Lyngvær, Writing a pseudo-device driver on Linux.
Tristan, Linux driver ioctl example.
[URL](https://github.com/Tristaan/linux-driver-ioctl-example)

** Linux kernel labs mmap documentation */
Linux kernel labs, Memory mapping.
[URL] (https://linux-kernel-labs.github.io/refs/heads/master/labs/memory_mapping.html)

** Simple module with mmap support **
Alessandro Rubini, and Jonathan Corbet, REALLY simple memory mapping demonstration.
The mmap device operation, from Linux Device Drivers book (see book reference further below).
[URL](https://github.com/martinezjavier/ldd3/blob/master/simple/simple.c)

** The dma-proxy AXI-DMA module that we want to emulate **
Xilinx, Linux DMA from User Space 2.0.
[URL](https://xilinx-wiki.atlassian.net/wiki/spaces/A/pages/1027702787/Linux+DMA+From+User+Space+2.0)

** A text book on writing Linux device drivers **
Jonathan Corbet, Alessandro Rubini, and Greg Kroah-Hartman, Linux Device Drivers, Third edition.
[URL](https://lwn.net/Kernel/LDD3/)


