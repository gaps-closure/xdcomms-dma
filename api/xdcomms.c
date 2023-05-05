/*
 * Cross Domain (XD) Communication API between Partitioned Applicaitons
 * and a GAP's Cross Domain Guard (CDG)
 *   v0.4, May 2023
 *
 * TODO: Add hton and ntoh for shm operations
 *
 * Library supports direct commicaiton between partitioned application
 * and CDGs, without a separate HAL daemon (linked with ZMQ).
 *
 * v0.4 MAY 2023: adds hardware abstractions layer that defines
 * abstracted one-way channels defined in 'xdcomms.h'. To relect this
 * abstraction, this file is renamed from sdcomms-dma.c' to 'xdcomms.c'.
 * Currently, it supports:
 *   - MIND-DMA: with Direct Memory Access specific structures in 'dma-proxy.h'
 *   - ESCAPE-SHM: with Shared Memory specific structures in 'shm-h'
 * Configuration is done using environment variables. Example below
 *
 * v0.3 OCTOBER 2022: Supprts direct transfers between CLOSURE and the
 * MIND-DMA CDG. The app connect to the MIND proxy DMA driver, which
 * uses kernel space DMA control to the XILINX AXI DMA / MCDMA driver
 * on the GE MIND ZCU102 FPGA board. For testing, it can also
 * communicate without FPGA hardware using a Pseudo driver emulation
 */

/* Example configuration using test application (found in ../test/)
 * a) Enclave 2 responds to a request from enclave 1 over DMA channels:
 *   DMARXDEV=sue_donimous_rx1 DMATXDEV=sue_donimous_tx1 ./app_req_rep -e 2
 *   DMARXDEV=sue_donimous_rx0 DMATXDEV=sue_donimous_tx0 ./app_req_rep
 * b)Enclave 2 responds to a request from enclave 1 over SHM channels:
 *
 *
 */

#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <time.h>
#include <sys/time.h>
#include <stdint.h>
#include <signal.h>
#include <string.h>
#include <sched.h>
#include <errno.h>
#include <sys/param.h>

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <assert.h>
#include <pthread.h>

#include "log.h"
#include "crc.h"
#include "xdcomms.h"
#include "dma-proxy.h"
#include "shm.h"

#define XDCOMMS_PRINT_STATE
#define NAME_LEN_MAX  64

// Fixed mmap configuration (channel_buffer in DMA device, shm_channel in SHM device)
typedef struct _memmap {
  int              prot;      // Mmap protection field (e.g., Read and/or write)
  int              flags;     // mmap'ed flags (e.g., SHARED)
  unsigned long    phys_addr; // mmap'ed physical address
  unsigned long    len;       // mmap'ed memory length
  void            *virt_addr; // Mmaped virtual address of packet buffer structure
  unsigned long    offset;    // Offset from mmap_virt_addr to channel info
} memmap;

// Dynamic (per packet) Received Packet information
typedef struct _pkt_info {
  char      newd;      // RX thread received new packet (xdcomms resets after reading)
  gaps_tag  tag;       // Received tag
  size_t    data_len;  // length of data
  uint8_t  *data;      // data buffer
  int       tid;       // transaction ID
} pkt_info;

// Channel configuration (with device abstraction)
typedef struct channel {
  uint32_t         ctag;           // Compressed tag (unique index) - used to search for channel
  char             dir;            // Receive (from network) or Transmit (to network): 'r' or 't'
  char             dev_type[4];    // device type: e.g., shm (ESCAPE) or dma (MIND)
  char             dev_name[NAME_LEN_MAX];   // Device name: e.g., /dev/mem or /dev/sue_dominous
  int              fd;             // Device file descriptor (set when device openned)
  int              retries;        // number of RX polls (everhmy RX_POLL_INTERVAL_NSEC) before timeout
  pthread_mutex_t  lock;           // Ensure RX thread does not write while xdcomms reads
  memmap           mm;             // Mmap configuration
  pkt_info         pinfo;          // Last received packet info
  shm_channel     *shm_addr;       // Pointer to mmap'ed Shared Memory structure
} chan;

// Channel list
typedef struct _chan_list {
  char             dev_name[NAME_LEN_MAX];
  chan            *cp;
  char            dir1;
  char            dir2;
} chan_list;

/* RX thread arguments when starting thread */
typedef struct _thread_args {
  chan            *cp;               // Channel RX thread is looking for
  int             buffer_id_start;  // Device buffer index
} thread_args;

void rcvr_thread_start(chan *cp);

codec_map  cmap[DATA_TYP_MAX];       // maps data type to its data encode + decode functions
chan       chan_info[GAPS_TAG_MAX];  // array of buffers to store local channel info per tag
pthread_mutex_t chan_create;

/**********************************************************************/
/* A) Codec map table to store encoding and decoding function pointers   */
/**********************************************************************/
void cmap_print_one(codec_map *cm) {
  fprintf(stderr, "[typ=%d ", cm->data_type);
  fprintf(stderr, "e=%p ",    cm->encode);
  fprintf(stderr, "d=%p] ",    cm->decode);
}

void cmap_print(void) {
  codec_map  *cm;
  fprintf(stderr, "%s: ", __func__);
  for(cm = cmap; cm->valid != 0; cm++) cmap_print_one(cm);
  fprintf(stderr, "\n");
}

codec_map *cmap_find(int data_type) {
  codec_map  *cm;
  for(cm = cmap; cm->valid != 0; cm++) {
    if (cm->data_type == data_type) return (cm);
  }
  log_warn("Could not find registered data typ = %d\n", data_type);
  return (NULL);
}

/* Create packet (serialize data and add header) */
void cmap_encode(uint8_t *data, uint8_t *buff_in, size_t *buff_len, gaps_tag *tag) {
  codec_map  *cm = cmap_find(tag->typ);
  cm->encode (data, buff_in, buff_len);
  log_buf_trace("API <- raw app data:", buff_in, *buff_len);
  log_buf_trace("    -> encoded data:", data,    *buff_len);
}

/* Decode data from packet */
void cmap_decode(uint8_t *data, size_t data_len, uint8_t *buff_out, gaps_tag *tag) {
  codec_map  *cm = cmap_find(tag->typ);
  cm->decode (buff_out, data, &data_len);
  log_buf_trace("API -> raw app data:", data,     data_len);
  log_buf_trace("    <- decoded data:", buff_out, data_len);
}

/**********************************************************************/
/* B) Tag Compression / Decompression                                    */
/**********************************************************************/
/* Compress teg (tag -> ctag) from a 3 member stuct to a uint32_t*/
void ctag_encode(uint32_t *ctag, gaps_tag *tag) {
  uint32_t ctag_h;
  ctag_h = ((CTAG_MOD * (
                         ( CTAG_MOD * ((tag->mux) % CTAG_MOD)) +
                                  ((tag->sec)  % CTAG_MOD))
             ) + ((tag->typ) % CTAG_MOD));
  *ctag = htonl(ctag_h);
}

/* Decompress teg (ctag -> tag) */
void ctag_decode(uint32_t *ctag, gaps_tag *tag) {
  uint32_t ctag_h = ntohl(*ctag);
  tag->mux = (ctag_h & 0xff0000) >> 16 ;
  tag->sec = (ctag_h &   0xff00) >> 8;
  tag->typ = (ctag_h &     0xff);
}

/**********************************************************************/
/* C) Device open                                               */
/**********************************************************************/
/* Open channel. Returns fd, mmap-va and mmap-len */
void dma_open_channel(chan *cp) {
  int buffer_count = TX_BUFFER_COUNT;
  if ((cp->dir) == 'r') buffer_count = RX_BUFFER_COUNT;

  // a) Open device
  if ((cp->fd = open(cp->dev_name, O_RDWR)) < 1) FATAL;
  // b) mmpp device
  cp->mm.len = sizeof(struct channel_buffer) * buffer_count;
  cp->mm.virt_addr = mmap(NULL, cp->mm.len, cp->mm.prot, cp->mm.flags, cp->fd, cp->mm.phys_addr);
  if (cp->mm.virt_addr == MAP_FAILED) FATAL;
  log_debug("Opened and mmap'ed DMA channel %s: mmap_virt_addr=0x%x, len=0x%x fd=%d", cp->dev_name, cp->mm.virt_addr, cp->mm.len, cp->fd);
}

// Open DMA channel given Physical address and length. Returns fd, mmap-va and mmap-len
void shm_open_channel(chan *cp) {
  void          *pa_virt_addr;
  unsigned long  pa_phys_addr, pa_mmap_len;       /* page aligned physical address (offset) */

  // a) Open device
  if ((cp->fd = open(cp->dev_name, O_RDWR | O_SYNC)) == -1) FATAL;
  
  // b) mmpp device: reduce address to be a multiple of page size and add the diff to length
  pa_phys_addr       = cp->mm.phys_addr & ~MMAP_PAGE_MASK;
  pa_mmap_len        = cp->mm.len + cp->mm.phys_addr - pa_phys_addr;
  pa_virt_addr       = mmap(0, pa_mmap_len, cp->mm.prot, cp->mm.flags, cp->fd, pa_phys_addr);
  if (pa_virt_addr == (void *) MAP_FAILED) FATAL;   // MAP_FAILED = -1
  cp->mm.virt_addr = pa_virt_addr + cp->mm.phys_addr - pa_phys_addr;   // add offset to page aligned addr
  log_debug("Opened and mmap'ed SHM channel %s: mmap_virt_addr=%p, len=0x%x fd=%d", cp->dev_name, cp->mm.virt_addr, cp->mm.len, cp->fd);
}

// Open channel device (based on name and type) and return its channel structure
void open_device(chan *cp) {
  log_trace("%s of type=%s name=%s", __func__, cp->dev_type, cp->dev_name);
  if      (strcmp(cp->dev_type, "dma") == 0) dma_open_channel(cp);
  else if (strcmp(cp->dev_type, "shm") == 0) shm_open_channel(cp);
  else {log_warn("Unknown type=%s (name=%s)", cp->dev_type, cp->dev_name); FATAL;}
}

// If new device, then open it (and remember it in local list)
void dev_open_if_new(chan *cp) {
  static chan_list  clist[MAX_DEV_COUNT];
  static int        once=1;
  int               i;
  
  if (once==1) {
    // a) initialize list
    for (i=0; i<MAX_DEV_COUNT; i++) {
      clist[i].cp = NULL;
      clist[i].dir1 = (char) 0;
      clist[i].dir2 = (char) 0;
    }
    once = 0;
  }
  
  for(i=0; i<MAX_DEV_COUNT; i++) {
    // b) See if device is not open
    if (clist[i].cp == NULL) {   // Not set, so put new device name into list
      clist[i].cp = cp;
      clist[i].dir1 = cp->dir;
      strcpy(clist[i].dev_name, cp->dev_name);
      open_device(cp);           // Open new device
      log_debug("%s: Opened new device %s dir=%c ctag=0x%08x i=%d", __func__, cp->dev_name, cp->dir, cp->ctag, i);
      return;  // new device
    }
    // b2) See if device is already open
    if (strcmp(cp->dev_name, clist[i].cp->dev_name) == 0) {    // Already set
      if ((clist[i].dir1) != cp->dir) {
        if ((clist[i].dir2) != cp->dir) {
          // Device shared for TX and RX (e.g., SHM), so copy matching device info
          clist[i].dir2 = cp->dir;
          cp->fd = clist[i].cp->fd;
          cp->mm.virt_addr = clist[i].cp->mm.virt_addr;
          log_debug("%s: %s device now shared for TX and RX (e.g., SHM) i=%d", __func__, cp->dev_name, i);
          return;
        }
      }
      log_trace("%s: %s device is not new device nor newly shared device i=%d", __func__, cp->dev_name, i);
      return;
    }
  }
  FATAL;    // Only here if list cannot store all devices (> MAX_DEV_COUNT)
}

/**********************************************************************/
/* D) Channel Info: Print, Create, Find (for all devices and TX/RX)   */
/**********************************************************************/
void chan_print(chan *cp) {
  fprintf(stderr, "  chan info %08x: dir=%c typ=%s nam=%s fd=%d\n", cp->ctag, cp->dir, cp->dev_type, cp->dev_name, cp->fd);
  fprintf(stderr, "  mmap len=0x%lx [pa=0x%lx va=%p off=0x%lx prot=0x%x flag=0x%x]\n",  cp->mm.len, cp->mm.phys_addr, cp->mm.virt_addr, cp->mm.offset, cp->mm.prot, cp->mm.flags);
  fprintf(stderr, "  ret=%d every %d ns newd=%d rx_buf_ptr=%p len=%lx tid=%d tag=<%d,%d,%d>\n", cp->retries, RX_POLL_INTERVAL_NSEC, cp->pinfo.newd, cp->pinfo.data, cp->pinfo.data_len, cp->pinfo.tid, cp->pinfo.tag.mux, cp->pinfo.tag.sec, cp->pinfo.tag.typ);
}

// Initialize channel information for all (GAPS_TAG_MAX) possible tags
//   Set retries from one of three possible timeout values (in msecs).
//   In order of precedence (highest first) they are the:
//    a) Input parameter specified in a xdc_sub_socket_non_blocking() call
//       (this is the only way to specify a different value for each flow).
//    b) Environment variable (TIMEOUT_MS) speciied when starting app
//    c) Default (RX_POLL_TIMEOUT_MSEC_DEFAULT) from xdcomms.h
void chan_init_all_once(void) {
  static int once=1;
  int        i, t_in_ms;
  char      *t_env;
  if (once==1) {
    t_in_ms = ((t_env = getenv("TIMEOUT_MS")) == NULL) ? RX_POLL_TIMEOUT_MSEC_DEFAULT : atoi(t_env);
    if (pthread_mutex_init(&chan_create, NULL) != 0)   FATAL;
    for(i=0; i < GAPS_TAG_MAX; i++) {
      chan_info[i].ctag       = 0;
      chan_info[i].mm.prot    = PROT_READ | PROT_WRITE;
      chan_info[i].mm.flags   = MAP_SHARED;
      chan_info[i].retries    = (t_in_ms * NSEC_IN_MSEC)/RX_POLL_INTERVAL_NSEC;
      chan_info[i].pinfo.newd = 0;
      chan_info[i].pinfo.data = NULL;
      if (pthread_mutex_init(&(chan_info[i].lock), NULL) != 0)   FATAL;
    }
    once=0;
  }
}

// Get channel device type
void get_dev_type(char *dev_type, char *env_type, char *def_type) {
  (env_type == NULL) ? strcat(dev_type, def_type) : strcat(dev_type, env_type);
}
// Get channel device name (*dev_name) from enivronment or default (for that type)
void get_dev_name(char *dev_name, char *env_name, char *def_name_dma, char *def_name_shm, char *dev_type) {
  strcpy(dev_name, "/dev/");        // prefix device name
  if      ((strcmp(dev_type, "dma")) == 0) {
    (env_name == NULL) ? strcat(dev_name, def_name_dma) : strcat(dev_name, env_name);
  }
  else if ((strcmp(dev_type, "shm")) == 0) {
    (env_name == NULL) ? strcat(dev_name, def_name_shm) : strcat(dev_name, env_name);
  }
  else FATAL;
}

// Get channel device number (*val) from enivronment or default (for that type)
void get_dev_val(unsigned long *val, char *env_val, unsigned long def_val_dma, unsigned long def_val_shm, char *dev_type) {
  if (env_val == NULL) {
    if       (strcmp(dev_type, "dma") == 0) *val = def_val_dma;
    else if  (strcmp(dev_type, "shm") == 0) *val = def_val_shm;
    else     FATAL;
  }
  else       *val = strtol(env_val, NULL, 16);
}

// Initialize configuration for a new tag
void chan_init_config_one(chan *cp, uint32_t ctag, char dir) {
  cp->ctag = ctag;
  cp->dir  = dir;

  if (dir == 't') { // TX
    get_dev_type(cp->dev_type,     getenv("DEV_TYPE_TX"), "dma");
    get_dev_name(cp->dev_name,     getenv("DEV_NAME_TX"), "dma_proxy_tx", "mem", cp->dev_type);
    // *val = (unsigned long) strtol(env_val, NULL, 16);
    get_dev_val (&(cp->mm.offset), getenv("DEV_OFFS_TX"), 0x0, 0x0, cp->dev_type);
    get_dev_val (&(cp->mm.len),    getenv("DEV_MMAP_LE"), (sizeof(struct channel_buffer) * TX_BUFFER_COUNT), SHM_MMAP_LEN_ESCAPE, cp->dev_type);
  }
  else {            // RX
    get_dev_type(cp->dev_type,     getenv("DEV_TYPE_RX"), "dma");
    get_dev_name(cp->dev_name,     getenv("DEV_NAME_RX"), "dma_proxy_rx", "mem", cp->dev_type);
    get_dev_val (&(cp->mm.offset), getenv("DEV_OFFS_RX"), 0x0, SHM_MMAP_LEN_HOST, cp->dev_type);
    get_dev_val (&(cp->mm.len),    getenv("DEV_MMAP_LE"), (sizeof(struct channel_buffer) * RX_BUFFER_COUNT), SHM_MMAP_LEN_ESCAPE, cp->dev_type);
    log_trace("%s: Len SHM = %x type=%s", __func__, SHM_MMAP_LEN_ESCAPE, cp->dev_type);
  }
  get_dev_val(&(cp->mm.phys_addr), getenv("DEV_MMAP_AD"), DMA_ADDR_HOST, SHM_MMAP_ADDR_HOST, cp->dev_type);
//  log_trace("%s Env Vars: type=%s name=%s off=%s mlen=%s (%", __func__, getenv("DEV_TYPE_TX"), getenv("DEV_NAME_TX"), getenv("DEV_OFFS_TX"), getenv("DEV_MMAP_LE"));
//  log_trace("%s Env Vars: type=%s name=%s off=%s mlen=%s", __func__, getenv("DEV_TYPE_RX"), getenv("DEV_NAME_RX"), getenv("DEV_OFFS_RX"), getenv("DEV_MMAP_LE"));
}

void shm_info_print(shm_channel *cip) {
  int            i, j;
  unsigned long  len_bytes;
  
  fprintf(stderr, "  shm channel info %08x (%p): last=%d next=%d (max=%d ga=%ld gb=%ld ut=0x%lx crc=0x%04x)\n", cip->cinfo.ctag, cip, cip->pkt_index_last, cip->pkt_index_next, cip->cinfo.pkt_index_max, cip->cinfo.ms_guard_time_aw, cip->cinfo.ms_guard_time_bw, cip->cinfo.unix_seconds, cip->cinfo.crc16);
  for (i=0; i<PKT_INDEX_MAX; i++) {
    len_bytes = cip->pinfo[i].data_length;
    fprintf(stderr, "  %d: len=%ld tid=0x%lx", i, len_bytes, cip->pinfo[i].transaction_ID);
    if (len_bytes > 0) {
      fprintf(stderr, " data=");
      for (j=0; j<(len_bytes/4); j++) fprintf(stderr, "%08x ", cip->pdata[i].data[j]);
    }
    fprintf(stderr, "\n");
  }
}

// After openning SHM device, initialize SHM configuration
void shm_init_config_one(chan *cp) {
  int i;
  cinfo  *cip = &(cp->shm_addr->cinfo);

  log_trace("%s: START cp=%p", __func__, cp);
  log_trace("%s: va=%p + off=%lx = %lx", __func__, cp->mm.virt_addr, cp->mm.offset, cp->shm_addr);
  log_trace("shm_channel size s=%lx c=%ld i=%lx d=%lx", sizeof(shm_channel), sizeof(cinfo), sizeof(pinfo), sizeof(pdata));

  cp->shm_addr->pkt_index_next = 0;
  cp->shm_addr->pkt_index_last = -1;

  cip->ctag               = cp->ctag;
  cip->pkt_index_max      = PKT_INDEX_MAX;
  cip->ms_guard_time_aw   = DEFAULT_MS_GUARD_TIME_AW;
  cip->ms_guard_time_bw   = DEFAULT_MS_GUARD_TIME_BW;
  cip->unix_seconds       = time(NULL);
  cip->crc16              = 0;
  cip->crc16              = crc16((uint8_t *) &cip, sizeof(cinfo));
//  log_trace("%s %08x %c Pnters: va=%p vc=%p ci=%p vd=%p vn=%p", __func__, cp->ctag, cp->dir, cp->shm_addr, cip, &(cp->shm_addr->pinfo), &(cp->shm_addr->pdata), &(cp->shm_addr->pkt_index_next));
  
  for (i=0; i<PKT_INDEX_MAX; i++) {
    cp->shm_addr->pinfo[i].data_length    = 0;
    cp->shm_addr->pinfo[i].transaction_ID = 0;
  }
#ifdef  XDCOMMS_PRINT_STATE
  shm_info_print(cp->shm_addr);
#endif  // XDCOMMS_PRINT_STATE
}

// Return pointer to Rx packet buffer for specified tag
//  a) If first call, then initialize all channels
//  b) If first call for a tag: 1) config channel info, 2) open device if new, 3) start thread
chan *get_chan_info(gaps_tag *tag, char dir) {
  uint32_t  ctag;
  int       i;
  chan     *cp;
  
  /* a) Initilize all channels (after locking from other application threads) */
  pthread_mutex_lock(&chan_create);
  chan_init_all_once();

  /* b) Find info for this tag */
  ctag_encode(&ctag, tag);                 // Encoded ctag
  for(i=0; i < GAPS_TAG_MAX; i++) {        // Break on finding tag or empty
    cp = &(chan_info[i]);
    if (cp->ctag == ctag) break;           // found existing slot for tag
    if (cp->ctag == 0) {                   // found empty slot (before tag)
      chan_init_config_one(cp, ctag, dir);              // 1) Configure new tag
      dev_open_if_new(cp);                              // 2) Open device (if not already open)
      log_trace("%s: Using %s device %s for ctag=0x%08x dir=%c", __func__, cp->dev_type, cp->dev_name, cp->ctag, cp->dir);
      if ((strcmp(cp->dev_type, "shm")) == 0) {
        cp->shm_addr = cp->mm.virt_addr + cp->mm.offset;
        if ((cp->dir) == 't') shm_init_config_one(cp);  // 3) Configure SHM structure for new channel
      }
      if ((cp->dir) == 'r') rcvr_thread_start(cp);      // 4) Start rx thread for new receive tag
      break;
    }
#ifdef  XDCOMMS_PRINT_STATE
    chan_print(cp);
#endif
  }

  /* c) Unlock and return chan_info pointer */
  if (i >= GAPS_TAG_MAX) FATAL;
  log_trace("%s %d: ctag=0x%08x", __func__, i, ctag);
  pthread_mutex_unlock(&chan_create);
  return (cp);
}
                  
/**********************************************************************/
/* E) BW Packet processing                                               */
/**********************************************************************/
void bw_print(bw *p) {
  fprintf(stderr, "%s: ", __func__);
  fprintf(stderr, "ctag=%u ", ntohl(p->message_tag_ID));
  fprintf(stderr, "crc=%02x ", ntohs(p->crc16));
  log_buf_trace("Data", (uint8_t *) p->data, ntohs(p->data_len));
  fprintf(stderr, "\n");
}

uint16_t bw_crc_calc(bw *pkt) {
  return (crc16((uint8_t *) pkt, sizeof(pkt->message_tag_ID) + sizeof (pkt->data_len)));
}

/* Get size of packet (= header length + data length) */
int bw_get_packet_length(bw *pkt, size_t data_len) {
  return (sizeof(pkt->message_tag_ID) + sizeof(pkt->data_len) + sizeof(pkt->crc16) + data_len);
}

/* Convert tag to local host format */
void bw_len_encode (uint16_t *out, size_t len) {
  *out = ntohs((uint16_t) len);
}

/* Convert tag to local host format */
void bw_len_decode (size_t *out, uint16_t len) {
  *out = ntohs(len);
}

/* b) Create CLOSURE packet header */
void bw_gaps_header_encode(bw *p, size_t *p_len, uint8_t *buff_in, size_t *buff_len, gaps_tag *tag) {
  ctag_encode(&(p->message_tag_ID), tag);
  bw_len_encode(&(p->data_len), *buff_len);
  p->crc16 = htons(bw_crc_calc(p));
  *p_len = bw_get_packet_length(p, *buff_len);
}

/**********************************************************************/
/* F) Device write functions                                              */
/**********************************************************************/
/* Use DMA ioctl operations to tx or rx data */
int dma_start_to_finish(int fd, int *buffer_id_ptr, struct channel_buffer *cbuf_ptr) {
//  log_trace("START_XFER (fd=%d, id=%d buf_ptr=%p unset-status=%d)", fd, *buffer_id_ptr, cbuf_ptr, cbuf_ptr->status);
//  time_trace("DMA Proxy transfer 1 (fd=%d, id=%d)", fd, *buffer_id_ptr);
  ioctl(fd, START_XFER,  buffer_id_ptr);
//  time_trace("DMA Proxy transfer 2 (fd=%d, id=%d)", fd, *buffer_id_ptr);
  ioctl(fd, FINISH_XFER, buffer_id_ptr);
//  time_trace("DMA Proxy transfer FINISHED (fd=%d, id=%d): status=%d", fd, *buffer_id_ptr, cbuf_ptr->status);
  if (cbuf_ptr->status != PROXY_NO_ERROR) {
//    log_trace("DMA Proxy transfer error (fd=%d, id=%d): status=%d (BUSY=1, TIMEOUT=2, ERROR=3)", fd, *buffer_id_ptr, cbuf_ptr->status);
    return -1;
  }
  return 0;
}

void dma_send(chan *cp, void *adu, gaps_tag *tag) {
  struct channel_buffer  *dma_tx_chan = (struct channel_buffer *) cp->mm.virt_addr;
  bw        *p;               // Packet pointer
  int       buffer_id=0;      // Use only a single buffer
  size_t    adu_len;    // encoder calculates length */
  size_t    packet_len;
  
  time_trace("XDC_Tx1 ready to encode for ctag=%08x", cp->ctag);
  cmap_encode(cp->mm.virt_addr, adu, &adu_len, tag);
  time_trace("XDC_Tx2 ready to send data for ctag=%08x typ=%s len=%ld", cp->ctag, cp->dev_type, adu_len);
  p = (bw *) &(dma_tx_chan->buffer);      // point to a DMA packet buffer */
  bw_gaps_header_encode(p, &packet_len, adu, &adu_len, tag);  /* Put packet into channel buffer */
  dma_tx_chan->length = packet_len;
  if (packet_len <= sizeof(bw)) log_buf_trace("TX_PKT", (uint8_t *) &(dma_tx_chan->buffer), packet_len);
  dma_start_to_finish(cp->fd, &buffer_id, dma_tx_chan);
  time_trace("XDC_Tx3 sent data for tag=<%d,%d,%d> len=%ld", tag->mux, tag->sec, tag->typ, packet_len);
  log_debug("XDCOMMS tx packet tag=<%d,%d,%d> len=%ld", tag->mux, tag->sec, tag->typ, packet_len);
//  log_trace("%s: Buffer id = %d packet pointer=%p", buffer_id, p);
//  time_trace("Tx packet end: tag=<%d,%d,%d> pkt-len=%d", tag->mux, tag->sec, tag->typ, packet_len);
}

// Dumb memory copy using 8-byte words
void naive_memcpy(unsigned long *d, const unsigned long *s, unsigned long len_in_words) {
  for (int i = 0; i < len_in_words; i++) *d++ = *s++;
}

void shm_send(chan *cp, void *adu, gaps_tag *tag) {
  int     pkt_index_now = cp->shm_addr->pkt_index_next;
  int     pkt_index_nxt = (pkt_index_now + 1) % cp->shm_addr->cinfo.pkt_index_max;
  size_t  adu_len=0;    // encoder calculates length */

  log_debug("%s TX index=%d len=%ld", __func__, pkt_index_now, adu_len);
  chan_print(cp);
  if (cp->shm_addr->pkt_index_last == pkt_index_nxt) {
    cp->shm_addr->pkt_index_last = ((cp->shm_addr->pkt_index_last) + 1) % cp->shm_addr->cinfo.pkt_index_max;
    // XXX: Wait ms_guard_time_bw
  }
  time_trace("XDC_Tx1 ready to encode for ctag=%08x", cp->ctag);
  cmap_encode((uint8_t *) &(cp->shm_addr->pdata->data[pkt_index_now]), adu, &adu_len, tag);
//  naive_memcpy(cp->shm_addr->pdata[pkt_index_nxt].data, adu, adu_len);  // TX adds new data
  cp->shm_addr->pinfo[pkt_index_now].data_length = adu_len;

  time_trace("XDC_Tx2 ready to send data for ctag=%08x typ=%s len=%ld", cp->ctag, cp->dev_type, adu_len);
  cp->shm_addr->pkt_index_next = pkt_index_nxt;           // TX updates RX
  if (cp->shm_addr->pkt_index_last < 0) cp->shm_addr->pkt_index_last = pkt_index_now;
#ifdef  XDCOMMS_PRINT_STATE
  shm_info_print(cp->shm_addr);
#endif  // XDCOMMS_PRINT_STATE
  exit(22);
}

/* Asynchronously send ADU to DMA driver in 'bw' packet */
void asyn_send(void *adu, gaps_tag *tag) {
  chan       *cp;        // abstract channel struct pointer for any device type

  // a) Open channel once (and get device type, device name and channel struct
  log_debug("Start of %s", __func__);
  cp = get_chan_info(tag, 't');
  pthread_mutex_lock(&(cp->lock));
  // b) encode packet into TX buffer and send */
  if (strcmp(cp->dev_type, "dma") == 0) dma_send(cp, adu, tag);
  if (strcmp(cp->dev_type, "shm") == 0) shm_send(cp, adu, tag);
  pthread_mutex_unlock(&(cp->lock));
}

/**********************************************************************/
/* Device read functions                                              */
/**********************************************************************/
void rcvr_dma(chan *cp, int buffer_id) {
  bw                    *p;
  struct channel_buffer *dma_cb_ptr =  (struct channel_buffer *) cp->mm.virt_addr;

  dma_cb_ptr[buffer_id].length = sizeof(bw);      /* XXX: ALl packets use buffer of Max size */
  log_debug("THREAD-2 waiting for packet (%d %s %s)", __func__, cp->ctag, cp->dev_type, cp->dev_name);
  while (dma_start_to_finish(cp->fd, &buffer_id, &(dma_cb_ptr[buffer_id])) != 0) { ; }
  p = (bw *) &(dma_cb_ptr[buffer_id].buffer);    /* XXX: DMA buffer must be larger than size of BW */
  ctag_decode(&(p->message_tag_ID), &(cp->pinfo.tag));
//  time_trace("XDC_THRD got packet tag=<%d,%d,%d> (fd=%d id=%d)", cp->pinfo.tag.mux, cp->pinfo.tag.sec, cp->pinfo.tag.typ, cp->fd, buffer_id);
  log_trace("THREAD-3 rx packet tag=<%d,%d,%d> buf-id=%d st=%d", cp->pinfo.tag.mux, cp->pinfo.tag.sec, cp->pinfo.tag.typ, buffer_id, dma_cb_ptr[buffer_id].status);
  pthread_mutex_lock(&(cp->lock));
  bw_len_decode(&(cp->pinfo.data_len), p->data_len);
  cp->pinfo.data = (uint8_t *) p->data;
  cp->pinfo.newd = 1;
  pthread_mutex_unlock(&(cp->lock));
}


void rcvr_shm(chan *cp, int buffer_id) {
  static int pkt_index=0;
  
  log_debug("THREAD-2 waiting for packet (%d %s %s) index=(r=%d t=%d)", cp->ctag, cp->dev_type, cp->dev_name, pkt_index, cp->shm_addr->pkt_index_next);
  while (pkt_index == (cp->shm_addr->pkt_index_next)) { ; }
  chan_print (cp);
  log_trace("THREAD-3 %s got packet (index=%d len=%d)", __func__, pkt_index, cp->shm_addr->pinfo[pkt_index].data_length);
  pthread_mutex_lock(&(cp->lock));
  cp->pinfo.data_len = cp->shm_addr->pinfo[pkt_index].data_length;
  cp->pinfo.data     = (uint8_t *) (cp->shm_addr->pdata->data);
  cp->pinfo.newd     = 1;
  pthread_mutex_unlock(&(cp->lock));
  pkt_index++;
}

// Receive packets via DMA in a loop (rate controled by FINISH_XFER blocking call)
void *rcvr_thread_function(thread_args *vargs) {
  chan                  *cp = vargs->cp;
  int                    buffer_id_index = 0;
  int                    buffer_id;

  while (1) {
    log_trace("THREAD-1 %s: fd=%d base_id=%d index=%d", __func__, cp->fd, vargs->buffer_id_start, buffer_id_index);
#if LOG_TRACE >= LOG_LEVEL_MIN
  chan_print (cp);
#endif  // LOG_LEVEL_MIN
    buffer_id = (vargs->buffer_id_start) + buffer_id_index;
    if      (strcmp(cp->dev_type, "dma") == 0) rcvr_dma(cp, buffer_id);
    else if (strcmp(cp->dev_type, "shm") == 0) rcvr_shm(cp, buffer_id);
    else {
      log_fatal("Unsupported device type %s\n", cp->dev_type);
      exit(-1);
    }
//    time_trace("XDC_Rx2 start decode for ctag=<%d,%d,%d>", cp->pinfo.tag.mux, cp->pinfo.tag.sec, cp->pinfo.tag.typ);
    buffer_id_index = (buffer_id_index + 1) % RX_BUFFS_PER_THREAD;
    log_trace("THREAD-4 buf-id=%d index=%d", buffer_id, buffer_id_index);
  }
}

/* Start a receiver thread */
void rcvr_thread_start(chan *cp) {
  static thread_args rxargs;
  static pthread_t   tid;

  /* Open rx channel and receive threads (only once) */
//  log_trace("%s: xdir=%c", __func__, cp->dir);
  pthread_mutex_lock(&chan_create);
  rxargs.cp = cp;
  rxargs.buffer_id_start = 0;
  log_trace("%s: c=0x%08x %s", __func__, cp->ctag, cp->dev_name);
  if (pthread_create(&tid, NULL, (void *) rcvr_thread_function, (void *)&rxargs) != 0) FATAL;
  pthread_mutex_unlock(&chan_create);
}

/* Receive packet from driver (via rx thread), storing data and length in ADU */
int nonblock_recv(void *adu, gaps_tag *tag, chan *cp) {

  pthread_mutex_lock(&(cp->lock));
//  log_trace("%s: Check for received packet on tag=<%d,%d,%d>", __func__, tag->mux, tag->sec, tag->typ);
  if (cp->pinfo.newd != 0) {                            // get packet from buffer if available)
#ifdef  XDCOMMS_PRINT_STATE
    chan_print(cp);
#endif
    cmap_decode(cp->pinfo.data, cp->pinfo.data_len, adu, &(cp->pinfo.tag));   /* Put packet into ADU */
    log_trace("XDCOMMS reads from buff=%p (len=%d)", cp->pinfo.data, cp->pinfo.data_len);
    if ((cp->pinfo.data_len) > 0) log_buf_trace("RX_PKT", cp->pinfo.data, cp->pinfo.data_len);
    cp->pinfo.newd = 0;                      // unmark newdata
//    time_trace("XDC_Rx3 packet copied to ADU: tag=<%d,%d,%d> adu-len=%d", cp->pinfo.tag.mux, cp->pinfo.tag.sec, cp->pinfo.tag.typ, cp->pinfo.data_len);
    log_debug("XDCOMMS rx packet tag=<%d,%d,%d> len=%d", cp->pinfo.tag.mux, cp->pinfo.tag.sec, cp->pinfo.tag.typ, cp->pinfo.data_len);
  }
  pthread_mutex_unlock(&(cp->lock));
  return ((cp->pinfo.data_len) > 0) ? cp->pinfo.data_len : -1;
}

/**********************************************************************/
/* XDCOMMS External API Functions                                     */
/* Some functions are gutted if not relevant to xdcomms-dma           */
/**********************************************************************/
void tag_print (gaps_tag *tag, FILE * fd) {
  fprintf(fd, "[mux=%02u sec=%02u typ=%02u] ", tag->mux, tag->sec, tag->typ);
}

void tag_write (gaps_tag *tag, uint32_t mux, uint32_t sec, uint32_t typ) {
  tag->mux = mux;
  tag->sec = sec;
  tag->typ = typ;
}

void tag_read (gaps_tag *tag, uint32_t *mux, uint32_t *sec, uint32_t *typ) {
  *mux = tag->mux;
  *sec = tag->sec;
  *typ = tag->typ;
}

/* copy tag_in to tag_out */
void tag_cp (gaps_tag *tag_out, gaps_tag *tag_in) {
  tag_out->mux = tag_in->mux;
  tag_out->sec = tag_in->sec;
  tag_out->typ = tag_in->typ;
}

/* Serialize tag onto wire */
void tag_encode (gaps_tag *tag_out, gaps_tag *tag_in) {
  tag_out->mux = htonl(tag_in->mux);
  tag_out->sec = htonl(tag_in->sec);
  tag_out->typ = htonl(tag_in->typ);
}

/* Convert tag to local host format */
void tag_decode (gaps_tag *tag_out, gaps_tag *tag_in) {
  tag_out->mux = ntohl(tag_in->mux);
  tag_out->sec = ntohl(tag_in->sec);
  tag_out->typ = ntohl(tag_in->typ);
}

/* Convert tag to local host format  */
void len_encode (uint32_t *out, size_t len) {
  *out = ntohl((uint32_t) len);
}

/* Convert tag to local host format */
void len_decode (size_t *out, uint32_t in) {
  *out = (uint32_t) htonl(in);
}

/* Set API Logging to a new level */
void xdc_log_level(int new_level) {
  char *ll;
  int   lvl;

  /* XXX: warn if new_level does not match XDCLOGLEVEL */
  if ((ll = getenv("XDCLOGLEVEL")) != NULL) { lvl = atoi(ll); } else { lvl = new_level; }

  if ((lvl >= LOG_TRACE) && (lvl <= LOG_FATAL)) {
    log_set_quiet(0);
    log_set_level(lvl);
    log_trace("Set API log level: %d", lvl);
  }
  else {
    log_trace("Cannot change API to log level %d (min=%d max=%d)\n", __func__, lvl, LOG_TRACE, LOG_FATAL);
  }
}

/* Load Codec Table with ADU encode and decode functions */
/* XXX: must be called at least once so locks are inited and log level defaults set  */
void xdc_register(codec_func_ptr encode, codec_func_ptr decode, int typ) {
  int   i;
  static int do_once = 1;

//  xdc_log_level(LOG_TRACE);            /* Mostly Quiet (LOG_TRACE is the most verbose) */
  time_log_level(LOG_TRACE);          /* Print time with us duration (for tracing performance)  */

  if (do_once == 1) {
    do_once = 0;
    for (i=0; i < DATA_TYP_MAX; i++) cmap[i].valid=0;   /* mark all cmap entries invalid */
  }

  for (i=0; i < DATA_TYP_MAX; i++) {
    if (cmap[i].data_type == typ) break;
    if (cmap[i].valid == 0) break;
  }
  if (i >= DATA_TYP_MAX) {
    log_fatal("CMAP table is full (DATA_TYP_MAX=%d)\n", i);
    exit(EXIT_FAILURE);
  }
  cmap[i].data_type = typ;
  cmap[i].valid     = 1;
  cmap[i].encode    = encode;
  cmap[i].decode    = decode;
  log_debug("API registered new data typ = %d (index=%d)", typ, i);
}

void set_address(char *xdc_addr, char *addr_in, const char *addr_default, int *do_once) { }
char *xdc_set_in(char *addr_in) { return NULL; }
char *xdc_set_out(char *addr_in) { return NULL; }
void *xdc_ctx(void) { return NULL; }
void *xdc_pub_socket(void) { return NULL; }
void *xdc_sub_socket(gaps_tag tag) { return NULL; }
void *xdc_sub_socket_non_blocking(gaps_tag tag, int timeout) {
  log_debug("Start of %s: timeout = %d ms for tag=<%d,%d,%d>", __func__, timeout, tag.mux, tag.sec, tag.typ);
  chan *cp = get_chan_info(&tag, 'r');
//  fprintf(stderr, "timeout = %d ms for tag=<%d,%d,%d>\n", timeout, tag.mux, tag.sec, tag.typ);
  if (timeout > 0) cp->retries = (timeout * NSEC_IN_MSEC)/RX_POLL_INTERVAL_NSEC;     // Set value
  log_trace("%s sets RX retries = %d every %d ns (for ctag=%08x)", __func__, cp->retries, RX_POLL_INTERVAL_NSEC, cp->ctag);
  return NULL;
}
void xdc_asyn_send(void *socket, void *adu, gaps_tag *tag) { asyn_send(adu, tag); }

int  xdc_recv(void *socket, void *adu, gaps_tag *tag) {
  chan            *cp;
  struct timespec  request;
  int              ntries;

  log_debug("Start of %s: tag=<%d,%d,%d>", __func__, tag->mux, tag->sec, tag->typ);
  cp              = get_chan_info(tag, 'r');     // get buffer for tag (to communicate with thread)
  request.tv_sec  = RX_POLL_INTERVAL_NSEC/NSEC_IN_SEC;
  request.tv_nsec = RX_POLL_INTERVAL_NSEC % NSEC_IN_SEC;
  ntries          = 1 + (cp->retries);           // number of tries to rx packet
  log_trace("%s: test %d times every %d (%d.%09d) ns", __func__, ntries, RX_POLL_INTERVAL_NSEC, request.tv_sec, request.tv_nsec);
  while ((ntries--) > 0)  {
    if (nonblock_recv(adu, tag, cp) > 0)  return 0;
//    log_trace("LOOP timeout %s: tag=<%d,%d,%d>: remaining tries = %d ", __func__, tag->mux, tag->sec, tag->typ, ntries);
    nanosleep(&request, NULL);
  }
//  log_trace("%s timeout for tag=<%d,%d,%d>", __func__, tag->mux, tag->sec, tag->typ);
  return -1;
}

/* Receive ADU from HAL - retry until a valid ADU */
void xdc_blocking_recv(void *socket, void *adu, gaps_tag *tag) {
  log_trace("Start of %s tag=<%d,%d,%d>", __func__, tag->mux, tag->sec, tag->typ);
  while (xdc_recv(socket, adu, tag) < 0);
}

