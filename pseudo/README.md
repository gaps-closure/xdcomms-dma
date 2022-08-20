# The Sue Donimous driver

This directory is a placehodler for a pseudo device driver for testing applications 
relying on dma-proxy kernel module without requiring the hardware device.

This is developed for easing testing of the CLOSURE xdcomms-dma without the hassle
of having to repeatedly building the firware using cross-compilers based on Petalinux,  
flashing it onto the ZCU102+ board, booting, testing, and copying over the logs.

Each test can now be doen in seconds and in paralle on multiple general-purpose machines,
thereby speeding up development, testing, and debugging.

## Building and Installing Module on Ubuntu Linux 

```bash
sudo apt install build-essential linux-headers-`uname -r`
git clone git@github.com:gaps-closure/xdcomms-dma.git
cd xdcomms-dma/pseudo
git checkout rk
make

sudo insmod ../sue_donimous.ko
lsmod | grep sue_donimous
grep sue_donimous </proc/devices
# 236 sue_donimous_rx
# make the device node with the assigned major number, e.g., 236
sudo mknod /dev/sue_donimous_rx c 236 0
cat /dev/sue_donimous_rx
cat /dev/sue_donimous_rx # cat changes
cat /dev/sue_donimous_rx
sudo rmmod sue_donimous
lsmod | grep sue_donimous
dmesg
```

## Testing

## References

Xilinx, Linux DMA from User Space 2.0.
[URL](https://xilinx-wiki.atlassian.net/wiki/spaces/A/pages/1027702787/Linux+DMA+From+User+Space+2.0)

Jonathan Corbet, Alessandro Rubini, and Greg Kroah-Hartman, Linux Device Drivers, Third edition.
[URL](https://lwn.net/Kernel/LDD3/)

Robert W. Oliver II,  Writing a simple linux kernel module.
[URL](https://blog.sourcerer.io/writing-a-simple-linux-kernel-module-d9dc3762c234)

Ole Andreas W. Lyngvær, Writing  apseudo-device driver on Linux.
[URL](https://lyngvaer.no/log/writing-pseudo-device-driver)

Tristan, Linux driver ioctl example.
[URL](https://github.com/Tristaan/linux-driver-ioctl-example)
