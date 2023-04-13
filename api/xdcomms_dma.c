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

codec_map    cmap[DATA_TYP_MAX];       /* maps data type to its data encode + decode functions */
rx_tag_info  rx_info[GAPS_TAG_MAX];       /* array of buffers to store Rx packet info per tag */
tx_tag_info *tx_tag_info_root = NULL;  /* linked list to save number of retires per tag */

pthread_mutex_t txlock;
pthread_mutex_t rxlock;

/**********************************************************************/
/* Codec map table to store encoding and decoding function pointers   */
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
/* Tag Compression / Decompression                                    */
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
/* rx_tag_info                                                        */
/**********************************************************************/
void rx_tag_info_print(rx_tag_info *t) {
  log_trace("rx_tag_info: ctag=%08x new=%d lock=%d buf_ptr=%x ret=%d", t->ctag, t->newd, t->lock, t->p_ptr, t->retries);
}

/* Return pointer to Rx packet buffer for specified tag */
rx_tag_info *get_rx_info(gaps_tag *tag) {
  static int once=1;
  uint32_t ctag;
  int i, t_in_ms;
  char *t_env;

  ctag_encode(&ctag, tag);       /* Use encoded ctag as  */
  /* a) Initilize all possible tag packet buffers (after locking from other application threads) */
  pthread_mutex_lock(&rxlock);
  if(once==1) {
    once = 0;
    for(i=0; i < GAPS_TAG_MAX; i++) {
      rx_info[i].ctag = 0;
      rx_info[i].newd = 0;
      t_in_ms = ((t_env = getenv("TIMEOUT_MS")) == NULL) ? RX_POLL_TIMEOUT_MSEC_DEFAULT : atoi(t_env);
      rx_info[i].retries = (t_in_ms * NSEC_IN_MSEC)/RX_POLL_INTERVAL_NSEC;
// fprintf(stderr, "default t_in_ms = %d (retries=%d) i=%d\n", t_in_ms, rx_info[i].retries, RX_POLL_INTERVAL_NSEC);
      if (pthread_mutex_init(&(rx_info[i].lock), NULL) != 0) {
        pthread_mutex_unlock(&rxlock);
        log_fatal("rx_tag_info mutex init has failed failed");
        exit(EXIT_FAILURE);
      }
    }
  }
  /* b) Find info for this tag (stored in rx_info erray) */
  for(i=0; i < GAPS_TAG_MAX; i++) { /* Break on finding tag or empty whichever is first */
    if (rx_info[i].ctag == ctag) break; /* found existing slot for tag */
    if (rx_info[i].ctag == 0) {          /* found empty slot (before tag) */
      rx_info[i].ctag = ctag;
      break;
    }
  }
  log_trace("%s: rx_info entry %d for ctag=%x (once=%d)", __func__, i, ctag, once);
  if (i >= GAPS_TAG_MAX) {
    pthread_mutex_unlock(&rxlock);
    log_fatal("TagBuf table is full (GAPS_TAG_MAX=%d)\n", i);
    exit(EXIT_FAILURE);
  }
  /* c) Unlock and return rx_info pointer */
  rx_tag_info_print(&rx_info[i]);
  pthread_mutex_unlock(&rxlock);
  return &rx_info[i];
}

/* Return the number of retries for the specified input tag (from the value stored
 * in the tx_tag_info linked-list. If not set, t will calculate the number of
 * retries from one of three possible timeout values (specified in milli-seconds).
 * In order of precedence (highest first) they are the:
 *    a) Input parameter (t_in_ms) specified in a xdc_sub_socket_non_blocking() call
 *       (this is the only way to specify a different value for each flow).
 *    b) Environment variable (TIMEOUT_MS) speciied when starting app (see get_rx_info()).
 *    c) Default (RX_POLL_TIMEOUT_MSEC_DEFAULT) from xdcomms.h (see get_rx_info())
 */
int get_retries(gaps_tag *tag, int t_in_ms) {
  rx_tag_info *t = get_rx_info(tag);
  if (t_in_ms > 0) {
    t->retries = (t_in_ms * NSEC_IN_MSEC)/RX_POLL_INTERVAL_NSEC;;     // Set value
    fprintf(stderr, "Set number of RX retries = %d every %d ns (for ctag=%08x)\n", t->retries, RX_POLL_INTERVAL_NSEC, t->ctag);
  }
  return (t->retries);
}

/**********************************************************************/
/* BW Packet processing                                               */
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
/* DMA-based open, send, and receive functions                        */
/**********************************************************************/
/* Open channel and save virtual address of buffer pointer */
int dma_open_channel(chan *c, char **channel_name, int channel_count, int buffer_count) {
  int i;

  log_trace("%s: open DMA channel", __func__);
  if(channel_count != 1) {
      log_fatal("xdcomms_dma handles only one DMA channel currently");
      exit(EXIT_FAILURE);
  }

  for (i = 0; i < channel_count; i++) {
    log_trace("DMA IOCTL mode", __func__);
    char dev_chan[64] = "/dev/";
    if (strlen(channel_name[i]) >= 60) {
      log_fatal("Channel name must be less than 60 chars: %s", channel_name[i]);
      exit(EXIT_FAILURE);
    }
    strcat(dev_chan, channel_name[i]);
    c[i].fd = open(dev_chan, O_RDWR);
    if (c[i].fd < 1) {
      log_fatal("Unable to open DMA proxy device file: %s", channel_name[i]);
      exit(EXIT_FAILURE);
    }
    c[i].buf_ptr = (struct channel_buffer *)mmap(NULL, sizeof(struct channel_buffer) * buffer_count,
                    PROT_READ | PROT_WRITE, MAP_SHARED, c[i].fd, 0);
    if (c[i].buf_ptr == MAP_FAILED) {
      log_fatal("Failed to mmap tx channel\n");
      exit(EXIT_FAILURE);
    }
    log_trace("Opened channel (id=%d): %s (buf_ptr=%p, fd=%d)", i, channel_name[i], c[i].buf_ptr, c[i].fd);
  }
  return (0); /* success */
}

/* Perform DMA ioctl operations to tx or rx data */
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
int send_channel_buffer(chan *c, size_t packet_len, int buffer_id) {
  log_trace("Start of %s: Ready to Write packet (len=%d) to fd=%d (id=%d) ", __func__, packet_len, c->fd, buffer_id);
  c->buf_ptr[buffer_id].length = packet_len;
  if (packet_len <= sizeof(bw)) log_buf_trace("TX_PKT", (uint8_t *) &(c->buf_ptr[buffer_id]), packet_len);
  return dma_start_to_finish(c->fd, &buffer_id, &(c->buf_ptr[buffer_id]));
}

void get_tx_dev_name_and_type(char *dev_type, char *tx_channel_name) {
  char        *tx;

  dev_type = getenv("TYPEDEV");
  if (strcmp(dev_type, "shm") == 0) {
    strcpy(dev_type, "mem");
    strcpy(tx_channel_name, ((tx = getenv("DMATXDEV")) == NULL) ? "mem" : tx);
  }
  else {
    strcpy(dev_type, "dma");
    strcpy(tx_channel_name, ((tx = getenv("DMATXDEV")) == NULL) ? "dma_proxy_tx" : tx);
  }
  log_trace("Tx Device type=%s name=%s", dev_type, tx_channel_name);
  exit(22);
}

/* Asynchronously send ADU to DMA driver in 'bw' packet */
void asyn_send(void *adu, gaps_tag *tag) {
  static int   once=1;                                   /* Open tx_channels only once */
  bw          *p;                                        /* Packet pointer */
  const int    i=0;                                      /* Assume single DMA channel */
  size_t       packet_len, adu_len;                      /* Note: encoder calculates length */
  static chan  tx_channels[1];                           /* Use only a single channel */
  const int    buffer_id=0;                              /* Use only a single buffer */
  char         tx_channel_name[1][24], dev_type[4];
    
  log_trace("Start of %s", __func__);
  // Open channel if needed
  if (once == 1) {
    pthread_mutex_lock(&txlock);
    get_tx_dev_name_and_type(dev_type, tx_channel_name[0]);
    if (strcmp(dev_type, "dma") == 0) dma_open_channel(tx_channels, tx_channel_name, 1, TX_BUFFER_COUNT);
    else {
      log_fatal("Unsupported device type %s\n", dev_type);
      exit(-1);
    }
    once = 0;
  }

  // encode packet into TX buffer and send */
  p = (bw *) &(tx_channels[i].buf_ptr[buffer_id]);          /* DMA channel buffer holds created packet */
  time_trace("XDC_Tx1 ready to encode for tag=<%d,%d,%d>", tag->mux, tag->sec, tag->typ);
  bw_gaps_data_encode(p, &packet_len, adu, &adu_len, tag);  /* Put packet into channel buffer */
  time_trace("XDC_Tx2 ready to send data for tag=<%d,%d,%d> len=%ld", tag->mux, tag->sec, tag->typ, packet_len);
  send_channel_buffer(&tx_channels[i], packet_len, buffer_id);
  time_trace("XDC_Tx3 sent data for tag=<%d,%d,%d> len=%ld", tag->mux, tag->sec, tag->typ, packet_len);
  log_debug("XDCOMMS tx packet tag=<%d,%d,%d> len=%ld", tag->mux, tag->sec, tag->typ, packet_len);
//  log_trace("%s: Buffer id = %d packet pointer=%p", buffer_id, p);
//  time_trace("Tx packet end: tag=<%d,%d,%d> pkt-len=%d", tag->mux, tag->sec, tag->typ, packet_len);
  pthread_mutex_unlock(&txlock);
}

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
      t = get_rx_info(&tag);
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

/* Receive packet from DMA driver, storing data and length in ADU */
int dma_recv(void *adu, gaps_tag *tag, rx_tag_info *t) {
  bw *p;
  size_t packet_len=0;  /* initialize to check at return */
  size_t adu_len=0;

  pthread_mutex_lock(&(t->lock));
  log_trace("%s: Check for received packet on tag=<%d,%d,%d>", __func__, tag->mux, tag->sec, tag->typ);
  rx_tag_info_print(t);
  if (t->newd != 0) {         // get packet from buffer if available)
    p = t->p_ptr;             // Point to device rx buffer
    time_trace("XDC_Rx2 start decode for tag=<%d,%d,%d>", tag->mux, tag->sec, tag->typ);
    bw_gaps_data_decode(p, 0, adu, &adu_len, tag);   /* Put packet into ADU */
    packet_len = bw_get_packet_length(p, adu_len);
    log_trace("XDCOMMS reads from DMA channel (buf_ptr=%p) len=(b=%d p=%d)", p, adu_len, packet_len);
    if (packet_len <= sizeof(bw)) log_buf_trace("RX_PKT", (uint8_t *) p, packet_len);
    t->newd = 0;                      // unmark newdata
    time_trace("XDC_Rx3 packet copied to ADU: tag=<%d,%d,%d> pkt-len=%d adu-len=%d", tag->mux, tag->sec, tag->typ, packet_len, adu_len);
    log_debug("XDCOMMS rx packet tag=<%d,%d,%d> len=%d", tag->mux, tag->sec, tag->typ, packet_len);
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
                                                      
  rcvr_thread_start();                       // Start rx thread for all tags (on first xdc_recv() call)
  rx_tag_info *t = get_rx_info(tag);         // get buffer for tag (to communicate with thread)
  log_trace("%s for tag=<%d,%d,%d>: ntries=%d interval=%d (%d.%09d) ns", __func__, tag->mux, tag->sec, tag->typ, ntries, RX_POLL_INTERVAL_NSEC, request.tv_sec, request.tv_nsec);
//  time_trace("XDC_Rx1 Wait for tag=<%d,%d,%d> (test %d times every %d ns)", tag->mux, tag->sec, tag->typ, ntries, RX_POLL_INTERVAL_NSEC);
  while ((ntries--) > 0)  {
    if (dma_recv(adu, tag, t) > 0)  return 0;
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

