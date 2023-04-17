/*
 * Cross Domain (XD) Communication API between Partitioned Applicaitons
 * and a GAP's Cross Domain Guard (CDG)
 *   v0.4, April 2023
 *
 * Library supports direct commicaiton between partitioned application
 * and CDGs, without a separate HAL daemon (linked with ZMQ).
 *
 * v0.4 MAY 2023:  Supprts communication with MIND and/or ESCAPE CDGs.
 * The xdcomms file abstracts the Hardware-specific communication based
 * on the contents of a HAL configuration file. It supports
 *   - MIND-DMA: MIND specific code from v0.3 into new linked file (hal_MIND_DMA.c)
 *   - ESCAPE-SHM: Shared Memory (SHM) comms in a new linked file (hal_ESCAPE_SHM.c)
 *
 * v0.3 OCTOBER 2022:  Supprts direct transfers between CLOSURE and
 * the MIND-DMA CDG. The app connect to the MIND proxy DMA driver,
 * which uses kernel space DMA control to the XILINX AXI DMA / MCDMA
 * driver on the GE MIND ZCU102 FPGA board. For testing, it can also
 * communicate without FPGA hardware using a Pseudo driver emulation
 */

#include <stdio.h>
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
#include <pthread.h>

#include "dma-proxy.h"
#include "xdcomms.h"

codec_map  cmap[DATA_TYP_MAX];       /* maps data type to its data encode + decode functions */
chan       chan_info[GAPS_TAG_MAX];  // array of buffers to store channel info per tag

pthread_mutex_t txlock;
pthread_mutex_t rxlock;
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

/**********************************************************************/
/* B) Tag Compression / Decompression                                    */
/**********************************************************************/
/* Compress teg (tag -> ctag) */
void ctag_encode(uint32_t *ctag, gaps_tag *tag) {
  uint32_t ctag_h;
  ctag_h = ((CTAG_MOD * (
                         ( CTAG_MOD * ((tag->mux) % CTAG_MOD)) +
                                  ((tag->sec)  % CTAG_MOD))
             ) + ((tag->typ) % CTAG_MOD));
  *ctag = htonl(ctag_h);
}

/* Decode compressed teg (ctag -> tag) */
void ctag_decode(uint32_t *ctag, gaps_tag *tag) {
  uint32_t ctag_h = ntohl(*ctag);
  tag->mux = (ctag_h & 0xff0000) >> 16 ;
  tag->sec = (ctag_h &   0xff00) >> 8;
  tag->typ = (ctag_h &     0xff);
}

/**********************************************************************/
/* C) Channel Info: Print, Create, Find (for all devices and TX/RX)   */
/**********************************************************************/
void chan_print(chan *cp) {
  log_trace("channel %08x: type=%s name=%s fd=%d new=%d lock=%d buf_ptr=%x ret=%d every %d ns", cp->ctag, cp->device_type, cp->device_name, cp->fd, cp->newd, cp->lock, cp->buf_ptr, cp->retries, RX_POLL_INTERVAL_NSEC);
}

/* Return pointer to Rx packet buffer for specified tag */
chan *get_chan_info(gaps_tag *tag) {
  static int once=1;
  uint32_t ctag;
  int i, t_in_ms;
  char *t_env;

  ctag_encode(&ctag, tag);       /* Use encoded ctag as  */
  
  /* a) Initilize all channels (after locking from other application threads) */
  pthread_mutex_lock(&chan_create);
  if(once==1) {
    t_in_ms = ((t_env = getenv("TIMEOUT_MS")) == NULL) ? RX_POLL_TIMEOUT_MSEC_DEFAULT : atoi(t_env);
    once = 0;
    for(i=0; i < GAPS_TAG_MAX; i++) {
      chan_info[i].ctag    = 0;
      chan_info[i].newd    = 0;
      chan_info[i].count   = 0;
      chan_info[i].retries = (t_in_ms * NSEC_IN_MSEC)/RX_POLL_INTERVAL_NSEC;
      if (pthread_mutex_init(&(chan_info[i].lock), NULL) != 0) {
        pthread_mutex_unlock(&rxlock);
        log_fatal("rx_tag_info mutex init has failed failed");
        exit(EXIT_FAILURE);
      }
    }
  }

  /* b) Find info for this tag */
  for(i=0; i < GAPS_TAG_MAX; i++) { /* Break on finding tag or empty whichever is first */
    if (chan_info[i].ctag == ctag) break; /* found existing slot for tag */
    if (chan_info[i].ctag == 0) {          /* found empty slot (before tag) */
      chan_info[i].ctag = ctag;
      break;
    }
  }
  pthread_mutex_unlock(&chan_create);

  log_trace("%s: chan_info entry %d for ctag=%x]", __func__, i, ctag);
  if (i >= GAPS_TAG_MAX) {
    pthread_mutex_unlock(&rxlock);
    log_fatal("TagBuf table is full (GAPS_TAG_MAX=%d)\n", i);
    exit(EXIT_FAILURE);
  }
  /* c) Unlock and return chan_info pointer */
  chan_print(&chan_info[i]);
  return &chan_info[i];
}

/* Return the number of retries for the specified input tag (from the value stored
 * in the rx_tag_info linked-list. If not set, t will calculate the number of
 * retries from one of three possible timeout values (specified in milli-seconds).
 * In order of precedence (highest first) they are the:
 *    a) Input parameter (t_in_ms) specified in a xdc_sub_socket_non_blocking() call
 *       (this is the only way to specify a different value for each flow).
 *    b) Environment variable (TIMEOUT_MS) speciied when starting app (see get_chan_info()).
 *    c) Default (RX_POLL_TIMEOUT_MSEC_DEFAULT) from xdcomms.h (see get_chan_info())
 */
int get_retries(gaps_tag *tag, int t_in_ms) {
  chan *cp = get_chan_info(tag);
  if (t_in_ms > 0) {
    cp->retries = (t_in_ms * NSEC_IN_MSEC)/RX_POLL_INTERVAL_NSEC;;     // Set value
    fprintf(stderr, "Set number of RX retries = %d every %d ns (for ctag=%08x)\n", cp->retries, RX_POLL_INTERVAL_NSEC, cp->ctag);
  }
  return (cp->retries);
}

/**********************************************************************/
/* D) BW Packet processing                                               */
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

/* Create packet (serialize data and add header) */
void bw_gaps_data_encode(bw *p, size_t *p_len, uint8_t *buff_in, size_t *buff_len, gaps_tag *tag) {
  codec_map  *cm = cmap_find(tag->typ);
  
  /* a) serialize data into packet */
  cm->encode (p->data, buff_in, buff_len);
  log_buf_trace("API <- raw app data:", buff_in, *buff_len);
  log_buf_trace("    -> encoded data:", p->data, *buff_len);
  /* b) Create CLOSURE packet header */
  ctag_encode(&(p->message_tag_ID), tag);
  bw_len_encode(&(p->data_len), *buff_len);
  p->crc16 = htons(bw_crc_calc(p));
  /* c) Return packet length */
  *p_len = bw_get_packet_length(p, *buff_len);
}

/* Decode data from packet */
void bw_gaps_data_decode(bw *p, size_t p_len, uint8_t *buff_out, size_t *len_out, gaps_tag *tag) {
  codec_map  *cm = cmap_find(tag->typ);
  
  ctag_decode(&(p->message_tag_ID), tag);
  bw_len_decode(len_out, p->data_len);
  cm->decode (buff_out, p->data, len_out);
  log_buf_trace("API -> raw app data:", p->data,  *len_out);
  log_buf_trace("    <- decoded data:", buff_out, *len_out);
}

/**********************************************************************/
/* Device open functions                                              */
/**********************************************************************/
/* Open channel and save virtual address of buffer pointer */
void *dma_open_channel(char *device_name, int buffer_count) {
  
  log_trace("%s: open DMA channel", __func__);
    chan->fd = open(device_name, O_RDWR);
    if (chan->fd < 1) {
      log_fatal("Unable to open DMA proxy device file: %s (fd=%d)", channel_name, chan->fd);
      exit(EXIT_FAILURE);
    }
    chan->buf_ptr = (struct channel_buffer *) mmap(NULL, sizeof(struct channel_buffer) * buffer_count,
                    PROT_READ | PROT_WRITE, MAP_SHARED, chan->fd, 0);
    if (chan->buf_ptr == MAP_FAILED) {
      log_fatal("Failed to mmap tx channel\n");
      exit(EXIT_FAILURE);
    }
    log_trace("Opened channel %s: buf_ptr=%p, fd=%d", channel_name, chan->buf_ptr, chan->fd);
  }
  return (0); /* success */
}

*destin = mmalloc(PROT_WRITE, MMAP_ADDR_HOST, fd, pa_virt_addr, pa_map_length); // to host mmap

// Open DMA channel sat given Physical address
//   Returns file descriptor and page-aligned address/length, so can deallocate
void *shm_open_channel(char *device_name, int protection, unsigned long phys_addr, int *fd, void **pa_virt_addr, unsigned long *pa_map_length) {
  void          *virt_addr;
  unsigned long  pa_phys_addr;       /* page aligned physical address (offset) */
  int            flags = MAP_SHARED;        // or (|) together bit flags


  if((*fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) FATAL;
  pa_phys_addr   = phys_addr & ~PAGE_MASK;    // Align physical addr (offset) to be at multiple of page size.
  *pa_map_length = (*pa_map_length) + phys_addr - pa_phys_addr;     // Increase len due to phy addr alignment
  *pa_virt_addr  = mmap(0, *pa_map_length, protection, flags, *fd, pa_phys_addr);
  if (*pa_virt_addr == (void *) MAP_FAILED) FATAL;   // MAP_FAILED = -1
  virt_addr      = *pa_virt_addr + phys_addr - pa_phys_addr;   // add offset to page aligned addr
  fprintf(stderr, "    Shared mmap'ed DDR [len=0x%lx Bytes] starts at virtual address %p\n", *pa_map_length, virt_addr);

#ifdef __TEST_USE_MLOCK__
  int lockerr;
  lockerr = mlock(*pa_virt_addr, *pa_map_length);
  if (lockerr) fprintf(stderr, "ERROR: mlock failed \n");  // XXX: ought to bail out
#endif /* __TEST_USE_MLOCK__ */

#ifdef DEBUG
  fprintf(stderr, "Shared mmap'ed DDR [len=0x%lx Bytes] starts at virtual address %p\n", *pa_map_length, virt_addr);
  if (phys_addr > 0) fprintf(stderr, "Using Linux Physical addr %p: up to pa=0x%lX\n",
                                  (void *) phys_addr, phys_addr + (*pa_map_length) - 1);
#endif
  return (virt_addr);
}

// Get channel device name and type
void get_dev_name_and_type(char *device_type, char *device_name) {
  char *user_type = getenv("TYPEDEV"));
  char *user_name = getenv("DMATXDEV"));
  (user_type == NULL) ? strcat(device_type, "dma") : strcpy(device_type, user_type);

  strcpy(device_name, "/dev/");
  if      (strcmp(device_type, "dma") == 0) {
    (user_name == NULL) ? strcat(device_name, "dma_proxy_tx") : strcat(device_name, user_name));
  }
  else if (strcmp(device_type, "shm") == 0) {
    (user_name == NULL) ? strcat(device_name, "mem") : strcat(device_name, user_name));
  }
  else {
    log_fatal("Unsupported device type %s\n", device_type);
    exit(-1);
  }
}

// Open channel device (based on name and type) and return its channel structure
void *open_device(chan *cp) {
  chan_print(cp);
  get_dev_name_and_type(cp->device_type, cp->device_name);
  log_trace("%s of type=%s name=%s", __func__, cp->device_type, cp->device_name);
  chan_print(cp);
  exit(22);
  if (strcmp(device_type, "dma") == 0) {
    return (dma_open_channel(device_type, device_name, 1, TX_BUFFER_COUNT));
  }
  if (strcmp(device_type, "shm") == 0) {
    return (shm_open_channel(device_type, device_name, PROT_WRITE, MMAP_ADDR_HOST, fd));
  }
  else {
    log_fatal("Unsupported device type %s\n", device_type);
    exit(-1);
  }
}
/**********************************************************************/
/* Device write functions                                              */
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
    log_trace("DMA Proxy transfer error (fd=%d, id=%d): status=%d (BUSY=1, TIMEOUT=2, ERROR=3)", fd, *buffer_id_ptr, cbuf_ptr->status);
    return -1;
  }
  return 0;
}

/* Send Packet to DMA driver */
int send_channel_buffer(chan *cp, size_t packet_len, int buffer_id) {
  log_trace("Start of %s: Ready to Write packet (len=%d) to fd=%d (id=%d) ", __func__, packet_len, c->fd, buffer_id);
  cp->buf_ptr[buffer_id].length = packet_len;
  if (packet_len <= sizeof(bw)) log_buf_trace("TX_PKT", (uint8_t *) &(cp->buf_ptr[buffer_id]), packet_len);
  return dma_start_to_finish(cp->fd, &buffer_id, &(cp->buf_ptr[buffer_id]));
}

void dma_send(void *adu, gaps_tag *tag, void *tx_chan) {
  dma_channel  *dma_tx_chan = (dma_channel *) tx_chan;
  bw           *p;                        // Packet pointer
  const int    buffer_id=0;               // Use only a single buffer
  size_t       packet_len, adu_len;       // encoder calculates length */

  p = (bw *) &(dma_tx_chan.buf_ptr,buffer[buffer_id]);    // point to a DMA packet buffer */
  time_trace("XDC_Tx1 ready to encode for tag=<%d,%d,%d>", tag->mux, tag->sec, tag->typ);
  bw_gaps_data_encode(p, &packet_len, adu, &adu_len, tag);  /* Put packet into channel buffer */
  time_trace("XDC_Tx2 ready to send data for tag=<%d,%d,%d> len=%ld", tag->mux, tag->sec, tag->typ, packet_len);
  send_channel_buffer(dma_tx_chan, packet_len, buffer_id);
  time_trace("XDC_Tx3 sent data for tag=<%d,%d,%d> len=%ld", tag->mux, tag->sec, tag->typ, packet_len);
  log_debug("XDCOMMS tx packet tag=<%d,%d,%d> len=%ld", tag->mux, tag->sec, tag->typ, packet_len);
//  log_trace("%s: Buffer id = %d packet pointer=%p", buffer_id, p);
//  time_trace("Tx packet end: tag=<%d,%d,%d> pkt-len=%d", tag->mux, tag->sec, tag->typ, packet_len);
}

/* Asynchronously send ADU to DMA driver in 'bw' packet */
void asyn_send(void *adu, gaps_tag *tag) {
  static int  once=1;                    // Open tx_channels only once
  chan       *cp = get_chan_info(tag);   // channel structure pointer for any device type

  // a) Open channel once (and get device type, device name and channel struct
  log_trace("Start of %s", __func__);
  pthread_mutex_lock(&(cp.lock));
  if (once == 1) {   // Open channel once if needed
    open_device(cp);
    once = 0;
  }
  // b) encode packet into TX buffer and send */
  if (strcmp(cp->device_type, "dma") == 0) dma_send(cp);
  if (strcmp(cp->device_type, "shm") == 0) shm_send(cp);
  pthread_mutex_unlock(&(cp.lock));
}

/**********************************************************************/
/* Device read functions                                              */
/**********************************************************************/
/* Receive packets via DMA in a loop (rate controled by FINISH_XFER blocking call) */
void *rcvr_thread_function(thread_args *vargs) {
  gaps_tag     tag;
  bw          *p;
  rx_tag_info *t;
  chan        *c = vargs->c;
  int          buffer_id_index = 0;
  int          buffer_id;
  unsigned int pkt_length;

  pkt_length = sizeof(bw);      /* XXX: ALl packets use buffer of Max size */
  log_debug("THREAD %s starting: fd=%d base_id=%d", __func__, c->fd, vargs->buffer_id_start);
//  log_trace("THREAD ptrs: a=%p, b=%p, c=%p", vargs, &(c->buf_ptr[0]), c);
  
  while (1) {
    buffer_id = vargs->buffer_id_start + buffer_id_index;
    c->buf_ptr[buffer_id].length = pkt_length;
    if (dma_start_to_finish(c->fd, &buffer_id, &(c->buf_ptr[buffer_id])) == 0) {
      p = (bw *) &(c->buf_ptr[buffer_id]);    /* XXX: DMA buffer must be larger than size of BW */
      ctag_decode(&(p->message_tag_ID), &tag);
      time_trace("XDC_THRD got packet tag=<%d,%d,%d> (fd=%d id=%d)", tag.mux, tag.sec, tag.typ, c->fd, buffer_id);
      log_trace("THREAD rx packet tag=<%d,%d,%d> buf-id=%d st=%d", tag.mux, tag.sec, tag.typ, buffer_id, c->buf_ptr[buffer_id].status);

      /* get buffer for tag, lock buffer, copy packet ptr to buffer, mark as newdata, release lock */
      t = get_chan_info(&tag);
      pthread_mutex_lock(&(t->lock));
//      memcpy(&(t->p), p, sizeof(bw)); /* XXX: optimize, copy only length of actual packet received */
      t->p_ptr = p;
      t->newd = 1;
      rx_tag_info_print(t);
      pthread_mutex_unlock(&(t->lock));
      buffer_id_index = (buffer_id_index + 1) % RX_BUFFS_PER_THREAD;
    }
  }
}

/* Start a receiver thread */
void rcvr_thread_start(void) {
  static int   once=1;
  static chan  rx_channels[1];                         /* Use only a single channel */
  char        *rx_channel_names[1];
  char        *rx;
  static thread_args rxargs[RX_BUFFER_COUNT];
  static pthread_t tid[RX_BUFFER_COUNT];

  /* Open rx channel and receive threads (only once) */
  pthread_mutex_lock(&rxlock);
  if (once==1) {
    log_trace("%s: open rx channel and start receiver thread(s)", __func__);
    rx_channel_names[0] = ((rx = getenv("DMARXDEV")) == NULL) ? "dma_proxy_rx" : rx;
    once = 0;
    dma_open_channel(rx_channels, rx_channel_names, 1, RX_BUFFER_COUNT);
    for (int i=0; i < RX_THREADS; i++) {
      rxargs[i].c = rx_channels;
      rxargs[i].buffer_id_start = i * RX_BUFFS_PER_THREAD;
      if (pthread_create(&tid[i], NULL, (void *) rcvr_thread_function, (void *)&rxargs[i]) != 0) {
        log_fatal("Failed to create rx thread");
        exit(EXIT_FAILURE);
      }
    }
  }
  pthread_mutex_unlock(&rxlock);
}

/* Receive packet from driver (via rx thread), storing data and length in ADU */
int nonblock_recv(void *adu, gaps_tag *tag, chan *cp) {
  bw     *pp;            // packet pointer
  size_t  packet_len=0;  // initialize to check at return
  size_t  adu_len=0;

  pthread_mutex_lock(&(cp->lock));
  log_trace("%s: Check for received packet on tag=<%d,%d,%d>", __func__, tag->mux, tag->sec, tag->typ);
  chan_print(cp);
  exit(24);
  if (cp->newd != 0) {                            // get packet from buffer if available)
    if (strcmp(cp->device_type, "dma") == 0) {    // MIND DMA driver
      pp = cp->buf_ptr;                           // Point to device rx buffer
      time_trace("XDC_Rx2 start decode for tag=<%d,%d,%d>", tag->mux, tag->sec, tag->typ);
      bw_gaps_data_decode(cp->buf_ptr, 0, adu, &adu_len, tag);   /* Put packet into ADU */
      packet_len = bw_get_packet_length(pp, adu_len);
      log_trace("XDCOMMS reads from DMA channel (buf_ptr=%p) len=(b=%d p=%d)", p, adu_len, packet_len);
      if (packet_len <= sizeof(bw)) log_buf_trace("RX_PKT", (uint8_t *) pp, packet_len);
      cp->newd = 0;                      // unmark newdata
      time_trace("XDC_Rx3 packet copied to ADU: tag=<%d,%d,%d> pkt-len=%d adu-len=%d", tag->mux, tag->sec, tag->typ, packet_len, adu_len);
      log_debug("XDCOMMS rx packet tag=<%d,%d,%d> len=%d", tag->mux, tag->sec, tag->typ, packet_len);
    }
    else if (strcmp(cp->device_type, "shm") == 0) {
      log_fatal("%s: Not yet written>", __func__);
    }
    else {
      log_fatal("Unsupported device type %s\n", cp->device_type);
      exit(-1);
    }
  }
  pthread_mutex_unlock(&(t->lock));
  return (adu_len > 0) ? adu_len : -1;
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
    if ((pthread_mutex_init(&txlock, NULL) != 0) || 
        (pthread_mutex_init(&rxlock, NULL) != 0)) {      /* init lock for tx and rx channel use */
      log_fatal("mutex init has failed failed");
      exit(EXIT_FAILURE);
    }
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
  log_debug("%s: timeout = %d ms for tag=<%d,%d,%d>", __func__, timeout, tag.mux, tag.sec, tag.typ);
//  fprintf(stderr, "timeout = %d ms for tag=<%d,%d,%d>\n", timeout, tag.mux, tag.sec, tag.typ);
  get_retries(&tag, timeout);    // APP overrides xdc_recv() timeout  (timeout in milliseconds)
  return NULL;
}
void xdc_asyn_send(void *socket, void *adu, gaps_tag *tag) { asyn_send(adu, tag); }

int  xdc_recv(void *socket, void *adu, gaps_tag *tag) {
  struct timespec request = {(RX_POLL_INTERVAL_NSEC/NSEC_IN_SEC), (RX_POLL_INTERVAL_NSEC % NSEC_IN_SEC) };
  int              ntries = 1 + get_retries(tag, -1);  // number of tries to rx packet
                                                      
  rcvr_thread_start();                   // Start rx thread for all tags (on first xdc_recv() call)
  chan *cp = get_chan_info(tag);         // get buffer for tag (to communicate with thread)
  log_trace("%s for tag=<%d,%d,%d>: ntries=%d interval=%d (%d.%09d) ns", __func__, tag->mux, tag->sec, tag->typ, ntries, RX_POLL_INTERVAL_NSEC, request.tv_sec, request.tv_nsec);
//  time_trace("XDC_Rx1 Wait for tag=<%d,%d,%d> (test %d times every %d ns)", tag->mux, tag->sec, tag->typ, ntries, RX_POLL_INTERVAL_NSEC);
  while ((ntries--) > 0)  {
    if (nonblock_recv(adu, tag, t) > 0)  return 0;
//    log_trace("LOOP timeout %s: tag=<%d,%d,%d>: remaining tries = %d ", __func__, tag->mux, tag->sec, tag->typ, ntries);
    nanosleep(&request, NULL);
  }
  log_trace("%s timeout for tag=<%d,%d,%d>", __func__, tag->mux, tag->sec, tag->typ);
  return -1;
}

/* Receive ADU from HAL - retry until a valid ADU */
void xdc_blocking_recv(void *socket, void *adu, gaps_tag *tag) {
  log_trace("Start of %s", __func__);
  while (xdc_recv(socket, adu, tag) < 0);
}
