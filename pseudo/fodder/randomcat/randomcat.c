#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/random.h>
#include <linux/init.h>
#include <linux/fs.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Rajesh Krishnan");
MODULE_DESCRIPTION("sue_donimous: Linux Kernel Module for pseudo dma-proxy device driver");
MODULE_VERSION("0.01");
MODULE_SUPPORTED_DEVICE("sue_donimous_rx0");

static int major;
static int busy; /* is the device already opened?   */
static int done; /* has the file already been read? */

static ssize_t sue_donimous_read(struct file *, char *, size_t, loff_t *);
static int     sue_donimous_open(struct inode *, struct file *);
static int     sue_donimous_release(struct inode *, struct file *);

static struct file_operations fops = { /* callbacks for file operations */
  .read = sue_donimous_read,
  .open = sue_donimous_open,
  .release = sue_donimous_release
};

static char dcat[] =
  "\033[38;5;238m ,                          ,,\"'\n"
  "  ▚,                      ,\"=|\n"
  "  '▒\"UL  .  -= ▔▔  =+=  J'\"░/,\n"
  "   \E}     ▔               ▙' _\n"
  "   ]                       ▞\n"
  "    '░   < \033[38;5;226mX\033[38;5;238m >"
  "     < \033[38;5;226mX\033[38;5;238m >  E\n"
  "  ───-                    G-───\n"
  "  __─-''        `/       ''-─__\n"
  "    ,-'\" ,▗     ▁▁      K\"'-.\n"
  "            =_   \033[38;5;203mU\033[38;5;238m _ # '\"\n"
  "              ' ' \"\033[0m\n";


static int __init sue_donimous_init(void) {
  printk(KERN_INFO "sue_donimous: installing module and registering character device\n");
  major = register_chrdev(0, "sue_donimous_rx", &fops); /* register as a character device */
  done = busy = 0;
  if (major < 0) printk(KERN_ALERT "register_chrdev %d", major);
  return 0;
}

static void __exit sue_donimous_exit(void) {
  printk(KERN_INFO "sue_donimous: unregistering character device and removing module\n");
  unregister_chrdev(major, "sue_donimous_rx");
  return;
}

static int sue_donimous_release(struct inode *ino, struct file *fp) {
  done = busy = 0;
  module_put(THIS_MODULE); /* we're finished */
  return 0;
}

static int sue_donimous_open(struct inode *ino, struct file *fp) {
  if (busy) return -EBUSY;     /* if device is in use, reply with busy error */
  busy = 1;                    /* toggle device as busy */
  try_module_get(THIS_MODULE); /* tell the system that we're live */
  return 0;
}

static ssize_t sue_donimous_read(struct file *fp, char *buf, size_t n, loff_t *of) {
  ssize_t len = sizeof(dcat)/sizeof(dcat[0]); /* get length of dcat */
  char rand;

  get_random_bytes(&rand, sizeof(rand));
  if (rand > 0) {
    dcat[0xce] = '>';
    dcat[0xee] = '>';
    dcat[0x190] = ' ';
  } else {
    dcat[0xce] = 'X';
    dcat[0xee] = 'X';
    dcat[0x190] = 'U';
  }

  if (done) return 0;

  /* Use copy_to_user() and put_user() when moving memory from kernel to userspace */
  if (copy_to_user(buf, dcat, len)) printk(KERN_ALERT "copy_on_user");

  done = 1;
  return len;
}

module_init(sue_donimous_init);
module_exit(sue_donimous_exit);