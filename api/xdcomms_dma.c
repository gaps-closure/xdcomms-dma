/*
 * Cross Domain (XD) Communication API between Applicaitons and GAP XD Guards
 * v0.2, August 2022
 *
 * This version of xdcomms supports Cross Domain communication by directly
 * (without HAL) communicating with the GE MIND DMA Device Driver (DDD). The
 * DDD itself is a proxy for kernel space DMA control to the DMA Engine (using
 * the XILINX AXI DMA / MCDMA driver). The module is based on the MIND user-
 * space test APP (dma_proxy_test.c).
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

codec_map  cmap[DATA_TYP_MAX];    /* maps data type to its data encode + decode functions */

pthread_mutex_t txlock;
pthread_mutex_t rxlock;

void put_rx_packet(gaps_tag *tag);
struct channel_buffer *get_rx_packet(gaps_tag *tag);
dmamap *dma_map_root = NULL;

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

/* Encode compressed teg (tag -> ctag) */
void bw_ctag_encode(uint32_t *ctag, gaps_tag *tag) {
  uint32_t ctag_h;
  ctag_h = ((CTAG_MOD * (
                         ( CTAG_MOD * ((tag->mux) % CTAG_MOD)) +
                                  ((tag->sec)  % CTAG_MOD))
             ) + ((tag->typ) % CTAG_MOD));
  *ctag = htonl(ctag_h);
}

/* Decode compressed teg (ctag -> tag) */
void bw_ctag_decode(uint32_t *ctag, gaps_tag *tag) {
  uint32_t ctag_h = ntohl(*ctag);
  tag->mux = (ctag_h & 0xff0000) >> 16 ;
  tag->sec = (ctag_h &   0xff00) >> 8;
  tag->typ = (ctag_h &     0xff);
}

/* Create packet (serialize data and add header) */
void bw_gaps_data_encode(bw *p, size_t *p_len, uint8_t *buff_in, size_t *buff_len, gaps_tag *tag) {
  codec_map  *cm = cmap_find(tag->typ);
  
  xdc_log_level(-1);            /* set logging level to default (if not set) */
  /* a) serialize data into packet */
  cm->encode (p->data, buff_in, buff_len);
  log_buf_trace("API <- raw app data:", buff_in, *buff_len);
  log_buf_trace("    -> encoded data:", p->data, *buff_len);
  /* b) Create CLOSURE packet header */
  bw_ctag_encode(&(p->message_tag_ID), tag);
  bw_len_encode(&(p->data_len), *buff_len);
  p->crc16 = htons(bw_crc_calc(p));
  /* c) Return packet length */
  *p_len = bw_get_packet_length(p, *buff_len);
}

/* Decode data from packet */
void bw_gaps_data_decode(bw *p, size_t p_len, uint8_t *buff_out, size_t *len_out, gaps_tag *tag) {
  codec_map  *cm = cmap_find(tag->typ);
  
  xdc_log_level(-1);            /* set logging level to default (if not set) */
  bw_ctag_decode(&(p->message_tag_ID), tag);
  bw_len_decode(len_out, p->data_len);
  cm->decode (buff_out, p->data, &p_len);
  log_buf_trace("API -> raw app data:", p->data,  *len_out);
  log_buf_trace("    <- decoded data:", buff_out, *len_out);
}

/**********************************************************************/
/* DMA-based open, send, and receive functions                        */
/**********************************************************************/
/* Open channel and save virtual address of buffer pointer */
int dma_open_channel(chan *c, const char **channel_name, int channel_count, int buffer_count) {
  int i;
  char* ll;

  if ((ll = getenv("XDCLOGLEVEL")) != NULL) xdc_log_level(atoi(ll));
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
void dma_start_to_finish(int fd, int *buffer_id_ptr, struct channel_buffer *cbuf_ptr) {
  log_trace("START_XFER (fd=%d, id=%d buf_ptr=%p unset-status=%d)", fd, *buffer_id_ptr, cbuf_ptr, cbuf_ptr->status);
  ioctl(fd, START_XFER,  buffer_id_ptr);
  ioctl(fd, FINISH_XFER, buffer_id_ptr);
  if (cbuf_ptr->status != PROXY_NO_ERROR) {
    log_warn("Proxy transfer error (fd=%d, id=%d): status=%d (BUSY=1, TIMEOUT=2, ERROR=3)", fd, *buffer_id_ptr, cbuf_ptr->status);
  }
  log_trace("FINISH_XFER");
}

/* Send Packet to DMA driver */
void send_channel_buffer(chan *c, size_t packet_len, int buffer_id) {
  log_trace("Start of %s: Ready to Write packet (len=%d) to fd=%d (id=%d) ", __func__, packet_len, c->fd, buffer_id);
  c->buf_ptr[buffer_id].length = packet_len;
  if (packet_len < 543) log_buf_trace("API sends Packet", (uint8_t *) &(c->buf_ptr[buffer_id]), packet_len);
  dma_start_to_finish(c->fd, &buffer_id, &(c->buf_ptr[buffer_id]));
}

/* Send ADU to DMA driver in 'bw' packet */
void dma_send(void *adu, gaps_tag *tag) {
  const int    i=0;                                      /* Assume single DMA channel */
  size_t       packet_len, adu_len;                      /* Note; encoder calculates lengiha */
  static int   once=1;                                   /* Open tx_channels only once */
  static chan  tx_channels[TX_CHANNEL_COUNT];            /* DMA channels */
  const int    buffer_id=0;                              /* Use only a single buffer */
  const char  *tx_channel_names[] = {"dma_proxy_tx"};
    
  log_trace("Start of %s", __func__);

  /* Encode */
  char fmt[] = "bw";
  bw  *p;                                                   /* Packet pointer */
  p = (bw *) &(tx_channels[i].buf_ptr[buffer_id]);  /* DMA channel buffer holds created packet */
  bw_gaps_data_encode(p, &packet_len, adu, &adu_len, tag);  /* Put packet into channel buffer */
  
  /* Open channel if needed, then send Packet */
  pthread_mutex_lock(&txlock);
  if (once == 1) {
    once = 0;
    dma_open_channel(tx_channels, tx_channel_names, TX_CHANNEL_COUNT, TX_BUFFER_COUNT);
  }
  send_channel_buffer(&tx_channels[i], packet_len, buffer_id);
  log_debug("XDCOMMS tx packet fmt=%s len=%ld, id=%d, buf_ptr=%p tag=<%d,%d,%d>", fmt, packet_len, buffer_id, p, tag.mux, tag.sec, tag.typ);
  pthread_mutex_unlock(&txlock);
}

/* Receive packet from DMA driver */
void receive_channel_buffer(chan *c, size_t *packet_len, int buffer_id) {
  log_trace("THREAD %s: Ready to read packet from fd=%d (unset len=%d, unset id=%d) ", __func__, c->fd, c->buf_ptr[buffer_id].length, buffer_id);
  dma_start_to_finish(c->fd, &buffer_id, &(c->buf_ptr[buffer_id]));
  *packet_len = c->buf_ptr[buffer_id].length;
}

/* Receive packets via DMA in a loop */
void *rcvr_thread_function(thread_args *vargs) {
  const int buffer_id = 0;
  size_t    packet_len = 0;
  gaps_tag  tag;
  bw       *p;

  log_trace("THREAD %s: fd=%d (ptr: a=%p, b=%p, c=%p", __func__, vargs->c->fd, vargs, vargs->c->buf_ptr, vargs->c);
  while (1) {
    receive_channel_buffer(vargs->c, &packet_len, vargs->buffer_id);  
    p = (bw *) &(vargs->c->buf_ptr[vargs->buffer_id]);
    bw_ctag_decode(&(p->message_tag_ID), &tag);
    log_debug("THREAD rx packet len=%ld, id=%d, tag=<%d,%d,%d>", packet_len, buffer_id, tag.mux, tag.sec, tag.typ);
    put_rx_packet(&tag);
  }
}

/* Start a receiver thread */
void rcvr_thread_start(void) {
  static int   once=1;
  static chan  rx_channels[RX_CHANNEL_COUNT];  /* DMA channels */
  const char  *rx_channel_names[] = { "dma_proxy_rx"};
  static thread_args rxargs[RX_BUFFER_COUNT];
  pthread_t tid[RX_BUFFER_COUNT];
  int nthreads = 1;

  /* nthreads = RX_BUFFER_COUNT; */

  /* Open rx channel and receive threads (only once) */
  pthread_mutex_lock(&rxlock);
  if (once==1) {
    once = 0;
    dma_open_channel(rx_channels, rx_channel_names, RX_CHANNEL_COUNT, RX_BUFFER_COUNT);
    for (int i=0; i < nthreads; i++) {
      rxargs[i].c = rx_channels;
      rxargs[i].buffer_id = i;
      if (pthread_create(&tid[i], NULL, (void *) rcvr_thread_function, (void *)&rxargs[i]) != 0) {
        log_fatal("Failed to create rx thread");
        exit(EXIT_FAILURE);
      }
    }
  }
  pthread_mutex_unlock(&rxlock);
}

/* Receive 'bw' packet from DMA driver, storing data and length in ADU */
int dma_recv(void *adu, gaps_tag *tag) {
  size_t packet_len=0, adu_len=0;        /* packet length is read from dma buffer */
  struct channel_buffer *cbuf_ptr;

  log_trace("Start of %s", __func__);
  rcvr_thread_start();

  log_debug("%s: Waiting for received packet on tag=<%d,%d,%d>", __func__, tag->mux, tag->sec, tag->typ);
  cbuf_ptr = get_rx_packet(tag);

  if (cbuf_ptr == NULL) return (-1);

  /* Decode packet */
  packet_len = cbuf_ptr->length;
  char fmt[] = "bw";
  bw  *p = (bw *) cbuf_ptr;                                 /* Packet pointer in DMA channel buffer */
  bw_gaps_data_decode(p, packet_len, adu, &adu_len, tag);   /* Put packet into ADU */
  size_t len_out;
  bw_len_decode(&len_out, p->data_len);
  log_debug("XDCOMMS reads  (format=%s) from DMA channel (buf_ptr=%p) len=%d dlen=%d", fmt, p, packet_len, len_out);
  if (packet_len < 543) log_buf_trace("API recv packet", (uint8_t *) p, packet_len);
  return (packet_len);
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
  static int do_once = 1;
  if (new_level == -1) new_level = LOG_INFO; 
  if ((new_level >= LOG_TRACE) && (new_level <= LOG_FATAL)) {
    if (do_once == 1) {
      log_set_quiet(0);
      log_set_level(new_level);
      log_trace("User sets API log level: %d", new_level);
      do_once = 0;
    }
  }
  else {
    log_warn("Cannot change API to log level %d (min=%d max=%d)\n", __func__, new_level, LOG_TRACE, LOG_FATAL);
  }
}

/* Load Codec Table with ADU encode and decode functions */
/* XXX: must be called at least once so locks are inited */
void xdc_register(codec_func_ptr encode, codec_func_ptr decode, int typ) {
  int i;
  static int  do_once = 1;

  if (do_once == 1) {
    do_once = 0;
    xdc_log_level(-1);                                  /* set log level to default if not set */
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
  if (i >= DATA_TYP_MAX) log_fatal("CMAP table is full (DATA_TYP_MAX=%d)\n", i);
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
void *xdc_sub_socket_non_blocking(gaps_tag tag, int timeout) { return NULL; } /* XXX: ought to use this timeout */
void xdc_asyn_send(void *socket, void *adu, gaps_tag *tag) { dma_send(adu, tag); }
int  xdc_recv(void *socket, void *adu, gaps_tag *tag) { return dma_recv(adu, tag); }

/* Receive ADU from HAL - retry until a valid ADU */
void xdc_blocking_recv(void *socket, void *adu, gaps_tag *tag) {
  log_trace("Start of %s", __func__);
  while (xdc_recv(socket, adu, tag) < 0);
}

/* ######################### DANGER ZONE BEGINS ####################### */
void put_rx_packet(gaps_tag *tag) {
  /* 
   * get the BW compressed tag 
   * get a lock of buffer pool, allocating one for tag if has none, release lock
   * get the buffer for tag 
   * get a lock on buffer, put the packet in buffer, mark newdata, release lock
   */
}

struct channel_buffer *get_rx_packet(gaps_tag *tag) {
  /* 
   * get the BW compressed tag 
   * get a lock of buffer pool, allocating one for tag if has none, release lock
   * get the buffer for tag 
   * get a lock on buffer, if buffer has newdata, get the packet in buffer, unmark newdata, release lock
   * if no newdata, retry until timeout 
   * if still no newdata, return NULL
   */
  return NULL;
}

#ifdef BORROW_IF_NEEDED_THEN_DELETE
/**********************************************************************/
/* Linked list with packet pointers to DMA rx buffers for all registered tags   */
/**********************************************************************/
dmamap *dma_map_add(gaps_tag *tag_in) {
  int     i;
  static int once=1;
  static pthread_mutex_t dma_lock;
  dmamap *new_dm = (dmamap *) malloc(sizeof(dmamap));
  
  /* 1) Initialize the dma_lock*/
  if (once == 1) {
    once = 0;
    if (pthread_mutex_init(&dma_lock, NULL) != 0) {
      log_fatal("mutex init has failed failed");
      exit(EXIT_FAILURE);
    }
  }
  
  /* 2) initialize new_dm dmamap node */
  tag_cp (&(new_dm->tag), tag_in);
  new_dm->index_r = 0;
  new_dm->index_w = 0;
  new_dm->next    = NULL;
  for (i=0; i<MAX_BUFS_PER_TAG; i++) {
    new_dm->cbuf_ptr[i] = NULL;
  }

  /* 3) Add  new node to start of linked list */
  pthread_mutex_lock(&dma_lock);
  new_dm->next = dma_map_root;
  dma_map_root = new_dm;
  pthread_mutex_unlock(&dma_lock);
  return (new_dm);
}

dmamap *dma_map_search(gaps_tag *tag_in) {
  dmamap *dm = NULL;

  /* Not locking dma_map_root: if root changes because of head insert, then new map will not be found in this seach */
  for(dm = dma_map_root; dm != NULL; dm = dm->next) {
    if ( (dm->tag.mux == tag_in->mux)
      && (dm->tag.sec == tag_in->sec)
      && (dm->tag.typ == tag_in->typ)
       ) {
      return (dm);
    }
  }
  return NULL;
}

dmamap *dma_map_search_and_add(gaps_tag *tag_in) {
  dmamap *dm;
  dm = dma_map_search(tag_in);
  if (dm == NULL) {
    log_debug("Could not find tag <%d, %d, %d> adding into dma_map", tag_in->mux, tag_in->sec, tag_in->typ);
    dm = dma_map_add(tag_in);
  }
  return (dm);
}

/* Wiat for new packet for tag associated with dm (up to timeout_count seconds) */
/* MUST USE NAON SLEEP and elapsed time from start */
struct channel_buffer *dma_buffer_ready_wait(dmamap *dm, long nanosec, int ntries) {
  struct timespec request = {0, nanosec};
  
  while ((ntries--) > 0)  {
    pthread_mutex_lock(&(dm->lock));
    if ((dm->index_w) != (dm->index_r)) {   /* If read, write indexes match, then buffer is empty */
      struct channel_buffer * ret=dm->cbuf_ptr[dm->index_r];
      pthread_mutex_unlock(&(dm->lock));
      return (ret);
    }
    pthread_mutex_unlock(&(dm->lock));
    nanosleep(&request, NULL);
  }
  return (NULL);
}

void *rcvr_thread_function1(thread_args *vargs) {
  int       buffer_id = 0;
  size_t    packet_len = 0;
  gaps_tag  tag;
  dmamap   *dm;     /* DMA-map corresponding to the tag of message just received */
  bw       *p;

  log_trace("THREAD %s: fd=%d (ptr: a=%p, b=%p, c=%p", __func__, vargs->c->fd, vargs, vargs->c->buf_ptr, vargs->c);
  while (1) {
    receive_channel_buffer(vargs->c, &packet_len, buffer_id);  /* Only one thread uses dma_proxy_rx, no need to lock */
    p = (bw *) &(vargs->c->buf_ptr[buffer_id]);        /* Packet pointer in DMA channel buffer */
    bw_ctag_decode(&(p->message_tag_ID), &tag);
    log_debug("THREAD rx packet len=%ld, id=%d, tag=<%d,%d,%d>", packet_len, buffer_id, tag.mux, tag.sec, tag.typ);
    
    if ( (dm = dma_map_search(&tag)) == NULL) {
      log_warn("THREAD could not find tag=<%d,%d,%d> in DMA-map, so ignoring", tag.mux, tag.sec, tag.typ);
    }
    else{
      pthread_mutex_lock(&(dm->lock));
      dm->cbuf_ptr[dm->index_w] = &(vargs->c->buf_ptr[buffer_id]);
      /* XXX ought to check if buffer will wrap around and log warning */
      dm->index_w = ((dm->index_w) + 1) % MAX_BUFS_PER_TAG;
      pthread_mutex_unlock(&(dm->lock));
      log_debug("THREAD adds packet pointer to Rx-ptr-list of tag=<%d,%d,%d>", tag.mux, tag.sec, tag.typ);
    }
    log_trace("THREAD rx done with message (len=%d id=%d) - loop for next ", packet_len, buffer_id);
  }
}

/* Receive 'bw' packet from DMA driver, storing data and length in ADU */
int dma_recv1(void *adu, gaps_tag *tag) {
  size_t                 packet_len=0, adu_len=0;        /* packet length is read from dma buffer */
  static int             once=1;
  static chan            rx_channels[TX_CHANNEL_COUNT];  /* DMA channels */
  dmamap                *dm;      /* Ptr to DMA-map associated with the tag for this receive */
  struct channel_buffer *cbuf_ptr;
  const char  *rx_channel_names[] = { "dma_proxy_rx", /* add unique channel names here */ };

  /* 1) find and create (if not already created) a DMA map for this tag */
  log_trace("Start of %s", __func__);
  dm = dma_map_search_and_add(tag);

  /* 2) Initialize channel and receive thread (only once) */
  if (once != 0) {
    /* open channel and save virtual address of buffer pointer in channel_buffer */
    once = dma_open_channel(rx_channels, rx_channel_names, RX_CHANNEL_COUNT, RX_BUFFER_COUNT);  /* return 0 on success */
    rcvr_thread_start(rx_channels);
  }
  
  /* 3) Wait for packet */
  log_debug("%s: Waiting for received packet on tag=<%d,%d,%d>", __func__, tag->mux, tag->sec, tag->typ);
  /* Checks up to 15 times for packet (in tag's dmamap queue) with 1ms delay between tries */
  cbuf_ptr = dma_buffer_ready_wait(dm, 1000000, 15);
  if (cbuf_ptr == NULL) return (-1);
  
  packet_len = cbuf_ptr->length;

  /* 4) Decode packet */
  char fmt[] = "bw";
  bw  *p = (bw *) cbuf_ptr;                                 /* Packet pointer in DMA channel buffer */
  bw_gaps_data_decode(p, packet_len, adu, &adu_len, tag);   /* Put packet into ADU */
  size_t len_out;
  bw_len_decode(&len_out, p->data_len);
  log_debug("XDCOMMS reads  (format=%s) from DMA channel %s (buf_ptr=%p) len=%d dlen=%d", fmt, rx_channel_names[0], p, packet_len, len_out);
  if (packet_len < 543) log_buf_trace("API recv packet", (uint8_t *) p, packet_len);

  /* Everything in cbuf pointer is processed, so release it back to the pool */
  pthread_mutex_lock(&(dm->lock));
  dm->index_r = ((dm->index_r) + 1) % MAX_BUFS_PER_TAG;
  pthread_mutex_unlock(&(dm->lock));
  return (packet_len);
}

#endif
