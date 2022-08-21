#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/moduleparam.h>
#include <linux/dmaengine.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/random.h>
#include <linux/of_dma.h>
#include <linux/kdev_t.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/errno.h>
#include <linux/cdev.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <asm/page.h>

#include "../api/dma-proxy.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Rajesh Krishnan");
MODULE_DESCRIPTION("sue_donimous: Linux Kernel Module for pseudo dma-proxy device driver");
MODULE_VERSION("0.01");

static int       done;
static int       xmajor;  
module_param(xmajor, int, 0);

static int        sue_donimous_open(struct inode *, struct file *);
static int        sue_donimous_release(struct inode *, struct file *);
static int        sue_donimous_mmap(struct file *, struct vm_area_struct *);
static long       sue_donimous_ioctl(struct file *, unsigned int, unsigned long);
static ssize_t    sue_donimous_write(struct file *, const char *, size_t, loff_t *);
static ssize_t    sue_donimous_read(struct file *, char *, size_t, loff_t *);
void              sue_donimous_vma_open(struct vm_area_struct *);
void              sue_donimous_vma_close(struct vm_area_struct *);

static struct file_operations f_ops = {
  .owner          = THIS_MODULE,
  .open           = sue_donimous_open,
  .release        = sue_donimous_release,
  .mmap           = sue_donimous_mmap,
  .unlocked_ioctl = sue_donimous_ioctl
};

static struct file_operations fr_ops = {
  .owner          = THIS_MODULE,
  .open           = sue_donimous_open,
  .release        = sue_donimous_release,
  .read           = sue_donimous_read,
  .mmap           = sue_donimous_mmap,
  .unlocked_ioctl = sue_donimous_ioctl
};

static struct file_operations fw_ops = {
  .owner          = THIS_MODULE,
  .open           = sue_donimous_open,
  .release        = sue_donimous_release,
  .write          = sue_donimous_write,
  .mmap           = sue_donimous_mmap,
  .unlocked_ioctl = sue_donimous_ioctl
};

static struct vm_operations_struct vm_ops = {
  .open =  sue_donimous_vma_open,
  .close = sue_donimous_vma_close,
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
  "              ' ' \"\033[0m\n\0";


/* Following structs represent a single channel of DMA, tx or rx. */
/* replace with backing device */
struct proxy_bd {
  struct completion cmp;
  dma_cookie_t cookie;
  dma_addr_t dma_handle;
  struct scatterlist sglist;
};

struct dma_proxy_channel {
  struct channel_buffer *buffer_table_p;    /* user to kernel space interface */
  dma_addr_t buffer_phys_addr;
  struct device *proxy_device_p;            /* character device support */
  struct device *dma_device_p;
  dev_t dev_node;
  struct cdev cdev;
  struct class *class_p;
  struct proxy_bd bdtable[BUFFER_COUNT];
  struct dma_chan *channel_p;              /* dma support */
  u32 direction;                           /* DMA_MEM_TO_DEV or DMA_DEV_TO_MEM */
  int bdindex;
};

struct dma_proxy_channel sue_donimous_rx0;
struct dma_proxy_channel sue_donimous_tx0;
struct dma_proxy_channel sue_donimous_rx1;
struct dma_proxy_channel sue_donimous_tx1;

void sue_donimous_vma_open(struct vm_area_struct *vma) {
  printk(KERN_NOTICE "VMA open, vir %lx, phy %lx\n", vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
}

void sue_donimous_vma_close(struct vm_area_struct *vma) {
  printk(KERN_NOTICE "VMA close\n");
}

static ssize_t sue_donimous_read(struct file *fp, char *buf, size_t n, loff_t *of) {
  ssize_t len = sizeof(dcat)/sizeof(dcat[0]); /* get length of dcat */
  char rand;
  char *end;

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
  end = memchr(dcat, 0, len);
  len = (end == NULL) ? len : (end - dcat);
  /* Use copy_to_user() and put_user() when moving memory from kernel to userspace */
  if (copy_to_user(buf, dcat, len)) printk(KERN_ALERT "copy_to_user");
  done = 1;
  return len;
}

static ssize_t sue_donimous_write(struct file *fp, const char *buf, size_t n, loff_t *of) {
  ssize_t len = sizeof(dcat);
  memset(dcat, 0, len);
  len = len > n ? n : len;
  memcpy(dcat, buf, len);
  if (copy_from_user(dcat, buf, len)) printk(KERN_ALERT "copy_from_user");
  printk(KERN_INFO "sue_donimous: received %zu chars, copied %zu chars\n", n, len);
  return len;
}

static int sue_donimous_release(struct inode *ino, struct file *fp) {
  module_put(THIS_MODULE); /* we're finished */
  return 0;
}

static int sue_donimous_open(struct inode *ino, struct file *fp) {
  fp->private_data = container_of(ino->i_cdev, struct dma_proxy_channel, cdev);
  done = 0;
  try_module_get(THIS_MODULE); /* tell the system that we're live */
  return 0;
}

static int sue_donimous_mmap(struct file *fp, struct vm_area_struct *vma) {
  struct dma_proxy_channel *pchannel_p = (struct dma_proxy_channel *)fp->private_data;

/*
  return dma_mmap_coherent(pchannel_p->dma_device_p, vma, pchannel_p->buffer_table_p, 
                           pchannel_p->buffer_phys_addr, vma->vm_end - vma->vm_start);
  */

  if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
                      vma->vm_end - vma->vm_start,
                      vma->vm_page_prot))
    return -EAGAIN;
  vma->vm_ops = &vm_ops;
  vma->vm_private_data = fp->private_data;
  sue_donimous_vma_open(vma);
  pchannel_p->buffer_table_p = (struct channel_buffer *)vma->vm_start;
  pchannel_p->buffer_phys_addr = vma->vm_pgoff << PAGE_SHIFT;
  return 0;
}

static void start_transfer(struct dma_proxy_channel *pchannel_p) { return; }

static void wait_for_transfer(struct dma_proxy_channel *pchannel_p) {
  /* XXX: do actual transfer -- read or write to backing device here */
  /* XXX: use direction to determine */
  /* XXX: use bdindex to determine where to copy from */

  return;
}

static long sue_donimous_ioctl(struct file *fp, unsigned int cmd, unsigned long arg) {
  struct dma_proxy_channel *pchannel_p = (struct dma_proxy_channel *)fp->private_data; 

  /* Get the bd index from the input argument as all commands require it */
  copy_from_user(&pchannel_p->bdindex, (int *)arg, sizeof(pchannel_p->bdindex));

  switch(cmd) {
    case START_XFER:
      start_transfer(pchannel_p);
      break;
    case FINISH_XFER:
      wait_for_transfer(pchannel_p); 
      break;
    case XFER:
      start_transfer(pchannel_p);
      wait_for_transfer(pchannel_p);
      break;
  }
  return 0;
}

static void mkchan(struct dma_proxy_channel *dp, int maj, int min, 
                   struct file_operations *fops, u32 direction) {
  int err;

  dp->buffer_table_p = NULL;    /* filled by mmap */
  dp->buffer_phys_addr = 0;     /* filled by mmap */
  dp->proxy_device_p = NULL;
  dp->dma_device_p = NULL;
  dp->dev_node = MKDEV(maj, min);

  // XXX: alloc_pages here?

  cdev_init(&(dp->cdev), fops);
  dp->cdev.owner = THIS_MODULE;
  err = cdev_add(&(dp->cdev), dp->dev_node, 1);
  if (err) printk (KERN_NOTICE "Error %d adding sue_donimous%d", err, min);

  // ignore dp->bdtable;
  dp->class_p = NULL;
  dp->channel_p = NULL;
  dp->direction = direction;
  dp->bdindex = 0;
}

static int __init sue_donimous_init(void) {
  int result;
  dev_t dev;
  printk(KERN_INFO "sue_donimous: installing module and registering device\n");
  if (xmajor) {
    dev = MKDEV(xmajor, 0);
    result = register_chrdev_region(dev, 4, "sue_donimous");
  } else {
    result = alloc_chrdev_region(&dev, 0, 4, "sue_donimous");
    xmajor = MAJOR(dev);
  }
  if (result < 0) {
    printk(KERN_WARNING "sue_donimous: unable to get major %d\n", xmajor);
    return result;
  }
  if (xmajor == 0) xmajor = result;
  mkchan(&sue_donimous_rx0, xmajor, 0, &fr_ops, DMA_DEV_TO_MEM);
  mkchan(&sue_donimous_tx0, xmajor, 1, &fw_ops, DMA_MEM_TO_DEV);
  mkchan(&sue_donimous_rx1, xmajor, 2, &f_ops,  DMA_DEV_TO_MEM);
  mkchan(&sue_donimous_tx1, xmajor, 3, &f_ops,  DMA_MEM_TO_DEV);
  return 0;
}

static void __exit sue_donimous_exit(void) {
  printk(KERN_INFO "sue_donimous: unregistering devices and removing module\n");
  cdev_del(&sue_donimous_rx0.cdev);
  cdev_del(&sue_donimous_tx0.cdev);
  cdev_del(&sue_donimous_rx1.cdev);
  cdev_del(&sue_donimous_tx1.cdev);
  unregister_chrdev_region(MKDEV(xmajor, 0), 4);
  return;
}

module_init(sue_donimous_init);
module_exit(sue_donimous_exit);

