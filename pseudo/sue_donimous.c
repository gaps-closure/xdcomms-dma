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

#define SUE_DONIMOUS 1        // BUFFER_COUNT = 16 
#include "../api/dma-proxy.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Rajesh Krishnan");
MODULE_DESCRIPTION("sue_donimous: Linux Kernel Module for pseudo dma-proxy device driver");
MODULE_VERSION("0.01");

static int       done;
static int       xmajor = 0;
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

struct proxy_bd {
  struct completion cmp;                  /* ignored */
  dma_cookie_t cookie;                    /* ignored */
  dma_addr_t dma_handle;                  /* ignored */
  struct scatterlist sglist;              /* ignored */
};

struct dma_proxy_channel {
  struct channel_buffer *buffer_table_p;  /* vir kernel buf - mmap to user */
  dma_addr_t buffer_phys_addr;            /* phy addr of above */
  struct device *proxy_device_p;          /* ignored */
  struct device *dma_device_p;            /* ignored */
  dev_t dev_node;                         /* maj,min for sue_donimous_* */
  struct cdev cdev;                       /* allocated cdev for channel */
  struct class *class_p;                  /* ignored */
  struct proxy_bd bdtable[BUFFER_COUNT];  /* ignored */
  struct dma_chan *channel_p;             /* ignored */
  u32 direction;                          /* DMA_MEM_TO_DEV or  DMA_DEV_TO_MEM */
  int bdindex;                            /* index into the channel_buffer */
};

struct trans_buffer {         /* internal transfer buffer to mimic device DMA */
  struct channel_buffer cb;
  char pipenum;
  char full;
};

/* global memory for channels, chaninel buffers, and transfer buffers */
struct dma_proxy_channel     sue_donimous_rx0;
struct dma_proxy_channel     sue_donimous_tx0;
struct dma_proxy_channel     sue_donimous_rx1;
struct dma_proxy_channel     sue_donimous_tx1;
static struct channel_buffer *buf_rx0;
static struct channel_buffer *buf_tx0;
static struct channel_buffer *buf_rx1;
static struct channel_buffer *buf_tx1;
static struct trans_buffer   xbuf[BUFFER_COUNT];

/* connect TX1 -> RX0 and RX1 -> TX0 */
#define BUF_RX0_PIPENUM      1
#define BUF_TX0_PIPENUM      2
#define BUF_RX1_PIPENUM      2
#define BUF_TX1_PIPENUM      1

void sue_donimous_vma_open(struct vm_area_struct *vma) {
  struct dma_proxy_channel *pchannel_p = (struct dma_proxy_channel *)vma->vm_private_data;
  printk(KERN_NOTICE "VMA open: vir %lx, len %lx, off %lx, phy %llx\n", 
          vma->vm_start, (vma->vm_end - vma->vm_start), vma->vm_pgoff, pchannel_p->buffer_phys_addr);
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

  vma->vm_private_data = fp->private_data;
  vma->vm_pgoff = (pchannel_p->buffer_phys_addr >> PAGE_SHIFT);
  vma->vm_flags |= VM_DONTEXPAND;
  vma->vm_ops = &vm_ops;

  if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff, 
                      vma->vm_end - vma->vm_start,
                      vma->vm_page_prot)) return -EAGAIN;
  sue_donimous_vma_open(vma);
  return 0;
}

static void start_transfer(struct dma_proxy_channel *pchannel_p) { return; }

/* returns 0 on success, -1 on no free buffer, -2 on error */
static int fake_transfer(void *buf, int lbytes, char pipenum, u32 direction) {
  int i;
  if(direction == DMA_MEM_TO_DEV) {
    /* find empty wait buffer */
    for (i=0; i < BUFFER_COUNT; i++) 
      if (xbuf[i].full == 0) break;
    /* if not found return timeout */
    if (i >= BUFFER_COUNT) return -1;
    /* set pipenum to match channel, copy from channel, and mark wait buffer full */
    xbuf[i].pipenum = pipenum;
    if (memcpy(xbuf[i].cb.buffer, buf, lbytes) != xbuf[i].cb.buffer) return -2;
    xbuf[i].full = 1;
    return 0;
  } else if (direction == DMA_DEV_TO_MEM) {
    /* find full wait buffer with matching pipenum */
    for (i=0; i < BUFFER_COUNT; i++) 
      if ((xbuf[i].full != 0) && (xbuf[i].pipenum == pipenum)) break;
    /* if not found return timeout */
    if (i >= BUFFER_COUNT) return -1;
    /* do memcpy to buf, and mark wait buffer empty */
    if (memcpy(buf, xbuf[i].cb.buffer, lbytes) != buf) return -2;
    xbuf[i].full = 0;
    return 0;
  } else return -2;
}

static void wait_for_transfer(struct dma_proxy_channel *pchannel_p) {
  struct channel_buffer *cbuf = pchannel_p->buffer_table_p;
  int                    bdindex = pchannel_p->bdindex;
  u32                    direction = pchannel_p->direction;
  char                  *devname;
  char                   pipenum;
  int                    ret = 0;
  unsigned long          timeout = msecs_to_jiffies(3000);
  enum dma_status        status;

  if((bdindex < 0) || (bdindex >= BUFFER_COUNT)) goto err_return;

  if (cbuf == buf_rx0) {
    devname = "sue_donimous_rx0";
    pipenum = BUF_RX0_PIPENUM;
    if(direction != DMA_DEV_TO_MEM) goto err_return;
  } else if (cbuf == buf_tx0) {
    devname = "sue_donimous_tx0";
    pipenum = BUF_TX0_PIPENUM;
    if(direction != DMA_MEM_TO_DEV) goto err_return;
  } else if (cbuf == buf_rx1) {
    devname = "sue_donimous_rx1";
    pipenum = BUF_RX1_PIPENUM;
    if(direction != DMA_DEV_TO_MEM) goto err_return;
  } else if (cbuf == buf_tx1) {
    devname = "sue_donimous_tx1";
    pipenum = BUF_TX1_PIPENUM;
    if(direction != DMA_MEM_TO_DEV) goto err_return;
  } else goto err_return;

  // printk(KERN_NOTICE "devname: %s, pipe: %d, direction: %d, bdindex: %d\n", devname, pipenum, direction, bdindex);

  cbuf[bdindex].status = PROXY_BUSY;

  /* What was in the original driver */
  // timeout = wait_for_completion_timeout(&pchannel_p->bdtable[bdindex].cmp, timeout);
  // status = dma_async_is_tx_complete(pchannel_p->channel_p, pchannel_p->bdtable[bdindex].cookie, NULL, NULL);

  /* fake transfer returns 0 on success, -1 on no free buffer, -2 on error */
  ret = fake_transfer((void *)cbuf[bdindex].buffer, cbuf[bdindex].length, pipenum, direction);

  if (ret == 0) {
    timeout = 1;
    status = DMA_COMPLETE;
  } else if (ret == -1) {
    timeout = 0;
  } else {
    status = DMA_ERROR; /* there is no in progress as we memcpy entire buffer */
  }

  if (timeout == 0)  {
   cbuf[bdindex].status  = PROXY_TIMEOUT;
    printk(KERN_ERR "DMA timed out\n");
  } else if (status != DMA_COMPLETE) {
    cbuf[bdindex].status = PROXY_ERROR;
    printk(KERN_ERR "DMA returned completion callback status of: %s\n",
                     status == DMA_ERROR ? "error" : "in progress");
  } else {
    cbuf[bdindex].status = PROXY_NO_ERROR;
  }

  return;

err_return:
  if((bdindex > 0) && (bdindex < BUFFER_COUNT)) cbuf[bdindex].status = PROXY_ERROR;
  printk(KERN_ERR "invalid settings for transfer\n");
  return;
}

static long sue_donimous_ioctl(struct file *fp, unsigned int cmd, unsigned long arg) {
  struct dma_proxy_channel *pchannel_p = (struct dma_proxy_channel *)fp->private_data;

  /* Get the bd index from the input argument as all commands require it */
  if (copy_from_user(&pchannel_p->bdindex, (int *)arg, sizeof(pchannel_p->bdindex)))
    printk(KERN_ALERT "copy_from_user");

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

inline static unsigned long mceil(unsigned long a, unsigned long b) {
  return (a + b - 1)/b;
}

static void mkchan(struct dma_proxy_channel *pchannel_p, int maj, int min,
                   struct file_operations *fops, u32 direction,
                   struct channel_buffer *buf) {
  int err;
  pchannel_p->buffer_table_p = buf;
  pchannel_p->buffer_phys_addr = virt_to_phys(pchannel_p->buffer_table_p);
  printk (KERN_NOTICE "Alloc vir: %p phy: %llx\n",
           pchannel_p->buffer_table_p, pchannel_p->buffer_phys_addr);

  pchannel_p->proxy_device_p = NULL;
  pchannel_p->dma_device_p = NULL;
  pchannel_p->dev_node = MKDEV(maj, min);

  cdev_init(&(pchannel_p->cdev), fops);
  pchannel_p->cdev.owner = THIS_MODULE;
  err = cdev_add(&(pchannel_p->cdev), pchannel_p->dev_node, 1);
  if (err) printk (KERN_NOTICE "Error %d adding sue_donimous%d", err, min);

  // ignore pchannel_p->bdtable;
  pchannel_p->class_p = NULL;
  pchannel_p->channel_p = NULL;
  pchannel_p->direction = direction;
  pchannel_p->bdindex = 0;
}

void *alloc_mmap_pages(unsigned int npages) {
  int i;
  void *mem = kmalloc(PAGE_SIZE * npages, GFP_KERNEL);
  printk(KERN_INFO "sue_donimous: requesting %ld\n", PAGE_SIZE * npages);
  if (!mem) printk(KERN_INFO "sue_donimous: could not alloc requested\n");
  if (!mem) return mem;
  for(i = 0; i < npages * PAGE_SIZE; i += PAGE_SIZE)
    SetPageReserved(virt_to_page(((unsigned long)mem) + i));
  return mem;
}

void free_mmap_pages(void *mem, unsigned int npages) {
  int i;
  if(!mem) return;
  for(i = 0; i < npages * PAGE_SIZE; i += PAGE_SIZE)
    ClearPageReserved(virt_to_page(((unsigned long)mem) + i));
  kfree(mem);
}

static inline unsigned int myceil(unsigned int a, unsigned int b) { return (a + b - 1) / b; }

static int __init sue_donimous_init(void) {
  dev_t dev;
  int result;
  int i;
  unsigned int npages = myceil(sizeof(struct channel_buffer) * BUFFER_COUNT, PAGE_SIZE);

  printk(KERN_INFO "sue_donimous: installing module and registering device\n");

  for (i=0; i < BUFFER_COUNT; i++) xbuf[i].full = 0;

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

  buf_rx0 = (struct channel_buffer *) alloc_mmap_pages(npages);
  buf_tx0 = (struct channel_buffer *) alloc_mmap_pages(npages);
  buf_rx1 = (struct channel_buffer *) alloc_mmap_pages(npages);
  buf_tx1 = (struct channel_buffer *) alloc_mmap_pages(npages);
  
  mkchan(&sue_donimous_rx0, xmajor, 0, &fr_ops, DMA_DEV_TO_MEM, buf_rx0);
  mkchan(&sue_donimous_tx0, xmajor, 1, &fw_ops, DMA_MEM_TO_DEV, buf_tx0);
  mkchan(&sue_donimous_rx1, xmajor, 2, &f_ops,  DMA_DEV_TO_MEM, buf_rx1);
  mkchan(&sue_donimous_tx1, xmajor, 3, &f_ops,  DMA_MEM_TO_DEV, buf_tx1);

  if (!buf_rx0 || !buf_tx0 || !buf_rx1 || !buf_tx1)
    printk(KERN_ERR "Kernel memory allocation failed\n");

  return 0;
}

static void __exit sue_donimous_exit(void) {
  unsigned int npages = myceil(sizeof(struct channel_buffer) * BUFFER_COUNT, PAGE_SIZE);
  printk(KERN_INFO "sue_donimous: unregistering devices and removing module\n");
  cdev_del(&sue_donimous_rx0.cdev);
  cdev_del(&sue_donimous_tx0.cdev);
  cdev_del(&sue_donimous_rx1.cdev);
  cdev_del(&sue_donimous_tx1.cdev);
  unregister_chrdev_region(MKDEV(xmajor, 0), 4);
  free_mmap_pages(buf_rx0, npages);
  free_mmap_pages(buf_tx0, npages);
  free_mmap_pages(buf_rx1, npages);
  free_mmap_pages(buf_tx1, npages);
  return;
}

module_init(sue_donimous_init);
module_exit(sue_donimous_exit);

