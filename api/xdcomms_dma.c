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
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <stdint.h>
#include <signal.h>
#include <string.h>
#include <sched.h>
#include <time.h>
#include <errno.h>
#include <sys/param.h>

#include "xdcomms.h"
#include "dma-proxy.h"

codec_map  cmap[DATA_TYP_MAX];    /* maps data type to its data encode + decode functions */
static thread_args args;

/**********************************************************************/
/* A) Set API Logging to a new level */
/**********************************************************************/
void xdc_log_level(int new_level) {
  static int do_once = 1;
  
//  fprintf(stderr, "%s: once=%d new_level=%d\n", __func__, do_once, new_level);
  // set to default if User has not already set
  if (new_level == -1) {
    if (do_once == 1) {
      log_set_quiet(0);               /* not quiet */
//      log_set_level(LOG_INFO);        /* default level */
      log_set_level(LOG_DEBUG);       /* for initial testing */
    }
    return;
  }
  if ((new_level >= LOG_TRACE) && (new_level <= LOG_FATAL)) {
    log_set_quiet(0);
    log_set_level(new_level);
    log_trace("User sets API log level: %d", new_level);
    do_once = 0;
  }
  else {
    log_warn("Cannot change API to log level %d (min=%d max=%d)\n", __func__, new_level, LOG_TRACE, LOG_FATAL);
  }
}

#ifdef BW_PACKET
/**********************************************************************/
/* B) BW Packet processing */
/**********************************************************************/
/* Print Packet */
void bw_print(bw *p) {
  fprintf(stderr, "%s: ", __func__);
  fprintf(stderr, "ctag=%u ", ntohl(p->message_tag_ID));
  fprintf(stderr, "crc=%02x ", ntohs(p->crc16));
  log_buf_trace("Data", (uint8_t *) p->data, ntohs(p->data_len));
  fprintf(stderr, "\n");
}

/* calculate packet's crc */
uint16_t bw_crc_calc(bw *pkt) {
//  uint16_t crc;
//  long *p=(long *) pkt;
//  crc = crc16((uint8_t *) pkt, sizeof(pkt->message_tag_ID) + sizeof (pkt->data_len));
//  log_debug("pkt = %x%x, len = %d crc=%d", *p, *(p+1), sizeof(pkt->message_tag_ID) + sizeof (pkt->data_len), crc);
  return (crc16((uint8_t *) pkt, sizeof(pkt->message_tag_ID) + sizeof (pkt->data_len)));
}

/* get size of packet (= header length + data length) */
int bw_get_packet_length(bw *pkt, size_t data_len) {
  return (sizeof(pkt->message_tag_ID) + sizeof(pkt->data_len) + sizeof(pkt->crc16) + data_len);
}

/* Convert tag to local host format (TODO, Use DFDL schema) */
void bw_len_encode (uint16_t *out, size_t len) {
  *out = ntohs((uint16_t) len);
}

/* Convert tag to local host format (TODO, Use DFDL schema) */
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
  tag->mux = ctag_h & 0xff0000;
  tag->sec = ctag_h &   0xff00;
  tag->typ = ctag_h &     0xff;
}

#endif

/**********************************************************************/
/* B) Legacy Tag processing */
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

/* Serialize tag onto wire (TODO, Use DFDL schema) */
void tag_encode (gaps_tag *tag_out, gaps_tag *tag_in) {
  tag_out->mux = htonl(tag_in->mux);
  tag_out->sec = htonl(tag_in->sec);
  tag_out->typ = htonl(tag_in->typ);
}

/* Convert tag to local host format (TODO, Use DFDL schema) */
void tag_decode (gaps_tag *tag_out, gaps_tag *tag_in) {
  tag_out->mux = ntohl(tag_in->mux);
  tag_out->sec = ntohl(tag_in->sec);
  tag_out->typ = ntohl(tag_in->typ);
}

/* Convert tag to local host format (TODO, Use DFDL schema) */
void len_encode (uint32_t *out, size_t len) {
  *out = ntohl((uint32_t) len);
}

/* Convert tag to local host format (TODO, Use DFDL schema) */
void len_decode (size_t *out, uint32_t in) {
  *out = (uint32_t) htonl(in);
}

/**********************************************************************/
/* C) Legacy CMAP table to store encoding and decoding function pointers */
/**********************************************************************/
/*
 * Print Codec Table entry
 */
void cmap_print_one(codec_map *cm) {
  fprintf(stderr, "[typ=%d ", cm->data_type);
  fprintf(stderr, "e=%p ",    cm->encode);
  fprintf(stderr, "d=%p] ",    cm->decode);
//  fprintf(stderr, "[valid=%d] ",   cm->valid);
}

/*
 * Print entire Codec Table
 */
void cmap_print(void) {
  codec_map  *cm;
  
  fprintf(stderr, "%s: ", __func__);
  for(cm = cmap; cm->valid != 0; cm++) cmap_print_one(cm);
  fprintf(stderr, "\n");
}

/*
 * Find Codec Table entry for a given data type
 */
codec_map *cmap_find(int data_type) {
  codec_map  *cm;
  
//  fprintf(stderr, "%s: typ=%d\n", __func__, data_type); cmap_print();
  for(cm = cmap; cm->valid != 0; cm++) {
    if (cm->data_type == data_type) return (cm);
  }
  log_warn("Could not find registered data typ = %d\n", data_type);
  return (NULL);
}

/*
 * Initialize Codec Table
 */
void cmap_init(void) {
  int         i;
  static int  do_once = 1;
  
  if (do_once == 1) {
    for (i=0; i < DATA_TYP_MAX; i++) cmap[i].valid=0;
    do_once = 0;
  }
}

/*
 * Load Codec Table with ADU encode and decode functions
 */
void xdc_register(codec_func_ptr encode, codec_func_ptr decode, int typ) {
  int i;

  xdc_log_level(-1);            /* set logging level to default (if not set) */
  cmap_init();
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
// cmap_print();
}

#ifdef BW_PACKET
/**********************************************************************/
/* Dbw) Legacy Encocde/decode data into/from a HAL packet */
/***************************************bw******************************/
/*
 * Create packet (serialize data and add header)
 */
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

/*
 * Decode data from packet
 */
void bw_gaps_data_decode(bw *p, size_t p_len, uint8_t *buff_out, size_t *len_out, gaps_tag *tag) {
  codec_map  *cm = cmap_find(tag->typ);
  
  xdc_log_level(-1);            /* set logging level to default (if not set) */
  bw_ctag_decode(&(p->message_tag_ID), tag);
  bw_len_decode(len_out, p->data_len);
  cm->decode (buff_out, p->data, &p_len);
  log_buf_trace("API -> raw app data:", p->data,  *len_out);
  log_buf_trace("    <- decoded data:", buff_out, *len_out);
}
#else
/**********************************************************************/
/* D) Legacy Encocde/decode data into/from a HAL packet */
/**********************************************************************/
/*
 * Create packet (serialize data and add header)
 */
void gaps_data_encode(sdh_ha_v1 *p, size_t *p_len, uint8_t *buff_in, size_t *buff_len, gaps_tag *tag) {
  codec_map  *cm = cmap_find(tag->typ);
  
  xdc_log_level(-1);            /* set logging level to default (if not set) */

  /* a) serialize data into packet */
  cm->encode (p->data, buff_in, buff_len);
  log_buf_trace("API <- raw app data:", buff_in, *buff_len);
  log_buf_trace("    -> encoded data:", p->data, *buff_len);

  /* b) Create CLOSURE packet header */
  tag_encode(&(p->tag), tag);

// tag_print(tag, stderr);
// fprintf(stderr, "%s: mux = %d = %d\n", __func__, tag->mux, *((uint32_t *) p));

  len_encode(&(p->data_len), *buff_len);
  /* TODO: preplace last two with  sizeof(*p) - ADU_SIZE_MAX_C  */
  *p_len = (*buff_len) + sizeof(p->tag) + sizeof(p->data_len);
  // TODO - return value to indicate an error
}

/*
 * Decode data from packet
 */
void gaps_data_decode(sdh_ha_v1 *p, size_t p_len, uint8_t *buff_out, size_t *len_out, gaps_tag *tag) {
  codec_map  *cm = cmap_find(tag->typ);
  
  xdc_log_level(-1);            /* set logging level to default (if not set) */
  /* a) deserialize data from packet (TODO: remove NBO ha tag, len) */
  tag_decode(tag, &(p->tag));
  len_decode(len_out, p->data_len);
//  fprintf(stderr, "%s\n", __func__); cmap_print();
  cm->decode (buff_out, p->data, &p_len);
//  log_trace("%s: plen=%d, %x  %x typ=%d, p_ptr=%p", __func__, p_len, *len_out, p->data_len, tag->typ, p);
  log_buf_trace("API -> raw app data:", p->data,  *len_out);
  log_buf_trace("    <- decoded data:", buff_out, *len_out);
  // TODO - return value to indicate an error
}

// XXX: Additional Functions TBD
//  typ = xdc_generate(spec);  /* creates encode and decode functions and typ, then use register to load them into the table */
// Also xdc_provision function(s)

void gaps_data_decode3(sdh_ha_v1 *p, size_t p_len, uint8_t *buff_out, size_t *len_out, gaps_tag *tag) {
  *buff_out=2;
}
#endif

  
/* We now use strerror instead of zmq_strerror */
void exit_with_zmq_error(const char* where) {
  log_fatal("HAL API exits after %s error %d: %s\n", where, errno, strerror(errno));
  exit(-1);
}

/******** FUNCTIONS GUTTED AS NOT NEEDED FOR DMA **********/
void set_address(char *xdc_addr, char *addr_in, const char *addr_default, int *do_once) { }
char *xdc_set_in(char *addr_in) { return NULL; }
char *xdc_set_out(char *addr_in) { return NULL; }
void *xdc_ctx(void) { return NULL; }
void *xdc_pub_socket(void) { return NULL; }
void *xdc_sub_socket_non_blocking(gaps_tag tag, int timeout) { return NULL; }
void *xdc_sub_socket(gaps_tag tag) { return NULL; }

/**********************************************************************/
/* X) Tx/Rx Threads and DMA Open/Map device  */
/**********************************************************************/
/*
 * Open channel and save virtual address of buffer pointer
 */
int open_channel(chan *c, const char **channel_name, int channel_count, int buffer_count) {
  int i;
  
#ifdef SHARED_MEMORY_MODE
  log_trace("Shared Memory mode", __func__);
#else
  log_trace("DMA IOCTL mode", __func__);
#endif
  for (i = 0; i < channel_count; i++) {
    c[i].fd = open(channel_name[i], O_RDWR);
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
  return (0);
}

/*
 * Set low pthread attribe priority
 */
pthread_attr_t *low_thread_priority(void) {
  static int              once=1;
  static pthread_attr_t   tattr_tx;
  pthread_attr_t         *attr_ptr;
  struct sched_param      param;
  int                     newprio = 20;   /* low priority */
  
  attr_ptr = &tattr_tx;
  if (once == 1) {
    /* Get  default attributes and scheduling param */
    pthread_attr_init (attr_ptr);
    pthread_attr_getschedparam (attr_ptr, &param);
    /* Set transmit priority to the lowest */
    param.sched_priority = newprio;
    pthread_attr_setschedparam (attr_ptr, &param);
    once = 0;
  }
  return (attr_ptr);
}

#ifndef SHARED_MEMORY_MODE
/*
 * Perform DMA ioctl operations to tx or rx data
 */
void dma_start_to_finish(int fd, int *buffer_id_ptr, struct channel_buffer *channel_buffer_ptr) {
  ioctl(fd, START_XFER,  buffer_id_ptr);
  ioctl(fd, FINISH_XFER, buffer_id_ptr);
  if (channel_buffer_ptr->status != PROXY_NO_ERROR) {
    log_warn("Proxy tx transfer error");
  }
}
#endif

/*
 * Transmit thread sends packet already stored in channel structure
 * (As ioctl calls are blocking so that multiple threads are required)
 */

void *tx_thread(thread_args *vargs) {
//  log_trace("%s: ptr=%p fd=%d id=%d len=%d (arg_ptr=%p)", __func__,  vargs->buf_ptr, vargs->fd, vargs->buffer_id, vargs->buf_ptr->length, vargs);
#ifndef SHARED_MEMORY_MODE
    dma_start_to_finish(vargs->fd, &(vargs->buffer_id), vargs->buf_ptr);
#endif
  log_trace("%s: Exut Tx thread (Finished TX of buffer (id=%d)", __func__, vargs->buffer_id);
  return (NULL);
}

/*
 * Send Packet to DMA driver in a new thread
 */
void send_channel_buffer(chan *c, size_t packet_len, void *thread_func, int buffer_count) {
  static int buffer_id=0;
  
  c->buf_ptr[buffer_id].length = packet_len;
  args.buf_ptr   = &(c->buf_ptr[buffer_id]);
  args.fd        = c->fd;
  args.buffer_id = buffer_id;
  log_buf_trace("API sends Packet", (uint8_t *) args.buf_ptr, packet_len);
  log_trace("%s: ptr=%p fd=%d id=%d len=%d (arg_ptr=%p)", __func__, args.buf_ptr, args.fd, args.buffer_id, packet_len, &args);
 
  if (pthread_create(&(c->tid), low_thread_priority(), thread_func, (void *)&args) != 0) {
    log_fatal("Failed to create tx thread");
    exit(EXIT_FAILURE);
  }
//  pthread_join(c->tid, NULL); /* Wait until thread is finished */
  /* Flip to next buffer, treating them as a circular list */
  buffer_id += BUFFER_INCREMENT;
  buffer_id %= buffer_count;
}

/*
 * Receive packet from DMA driver
 */
void receive_channel_buffer(chan *c, size_t *packet_len) {

  log_trace("Start of %s: Ready to Read data from fd=%d (len=%d)", __func__, c->fd, c->buf_ptr[0].length);
//  pthread_create(&(rx_channel->tid), NULL, (void *) rx_thread, (void *)rx_channel);
#ifdef SHARED_MEMORY_MODE
  while (c->buf_ptr[0].length < 1 ) {
    sleep(1);
    log_trace("%s len=%d, data[0]=%x ptr=(%p-%p = %x)", __func__, c->buf_ptr[0].length, c->buf_ptr[0].buffer[0], c->buf_ptr[0].buffer, &(c->buf_ptr[0].length), c->buf_ptr[0].buffer - (&(c->buf_ptr[0].length)));
  }
#else
  int buffer_id=0;
  dma_start_to_finish(c->fd, &buffer_id, c->buf_ptr);
#endif
  *packet_len = c->buf_ptr[0].length;
}

/**********************************************************************/
/* G) Legacy ZMQ Communication Send and Receive */
/**********************************************************************/
/*
 * Send ADU to DMA driver in an sdh_ha_v1 packet
 */
void xdc_asyn_send(void *socket, void *adu, gaps_tag *tag) {
  int         i=0;                                      /* Initially assume 1 DMA channel */
  static int  once=1;
  size_t      packet_len, adu_len;                      /* Note; encoder calculates lengiha */
  chan        tx_channels[TX_CHANNEL_COUNT];            /* DMA channels */
#ifdef SHARED_MEMORY_MODE
  const char *tx_channel_names[] = { "green_to_orange_channel", /* unique channel name(s) here */ };
#else
  const char *tx_channel_names[] = { "dma_proxy_tx", /* add unique channel names here */ };
#endif
    
  log_trace("Start of %s", __func__);
  /* open channel and save virtual address of buffer pointer (in tx_channels) */
  if (once == 1) once = open_channel(tx_channels, tx_channel_names, TX_CHANNEL_COUNT, TX_BUFFER_COUNT);
#ifdef BW_PACKET
  char fmt[] = "bw";
  bw  *p;                                                   /* Packet pointer */
  p = (bw *) tx_channels[i].buf_ptr;                        /* DMA channel buffer holds created packet */
  bw_gaps_data_encode(p, &packet_len, adu, &adu_len, tag);  /* Put packet into channel buffer */
#else
  char fmt[] = "ha";
  sdh_ha_v1  *p;                                            /* Packet pointer */
  p = (sdh_ha_v1 *) tx_channels[i].buf_ptr;                 /* DMA channel buffer holds created packet */
  gaps_data_encode(p, &packet_len, adu, &adu_len, tag);     /* Put packet into channel buffer */
#endif
  send_channel_buffer(&tx_channels[i], packet_len, (void *)tx_thread, TX_BUFFER_COUNT);  /* Send packet */
  log_debug("XDCOMMS writes (format=%s) into DMA channel %s (id=%d, buf_ptr=%p) len=%d", fmt, tx_channel_names[i], i, p, packet_len);

}

/*
 * Receive ADU from HAL (HAL is ZMQ publisher) from a sdh_ha_v1 packet
 * Returs size of packet received (timeout/error if < 0)
 */
int xdc_recv(void *socket, void *adu, gaps_tag *tag) {
  int         i=0;                                      /* Initially assume 1 DMA channel */
  static int  once=1;
  size_t      packet_len=0, adu_len=0;                      /* Note; encoder calculates lengiha */
  chan        rx_channels[TX_CHANNEL_COUNT];            /* DMA channels */
#ifdef SHARED_MEMORY_MODE
  const char *rx_channel_names[] = { "orange_to_green_channel", /* add unique channel names here */ };
#else
  const char *rx_channel_names[] = { "dma_proxy_rx", /* add unique channel names here */ };
#endif

  log_trace("Start of %s", __func__);
  /* open channel and save virtual address of buffer pointer (in channel structure) */
  if (once == 1) once = open_channel(rx_channels, rx_channel_names, RX_CHANNEL_COUNT, RX_BUFFER_COUNT);
//  log_trace("%s: once-%d buf_ptr=%p", __func__, once, &(rx_channels[i].buf_ptr[0]));
  receive_channel_buffer(&(rx_channels[i]), &packet_len);  /* Wait for packet in channel buffer */
#ifdef BW_PACKET
  char fmt[] = "bw";
  bw  *p;                                                   /* Packet pointer */
  p = (bw *) &(rx_channels[i].buf_ptr[0]);                  /* DMA channel buffer with packet */
  bw_gaps_data_decode(p, packet_len, adu, &adu_len, tag);   /* Put packet into ADU */
#else
  char fmt[] = "ha";
  sdh_ha_v1  *p;                                            /* Packet pointer */
  p = (sdh_ha_v1 *) &(rx_channels[i].buf_ptr[0]);           /* DMA channel buffer with packet */
  gaps_data_decode(p, packet_len, adu, &adu_len, tag);      /* Put packet into ADU */
#endif

  log_debug("XDCOMMS reads  (format=%s) from DMA channel %s (id=%d, buf_ptr=%p) len=%d", fmt, rx_channel_names[i], i, p, packet_len);
  log_buf_trace("API recv packet", (uint8_t *) p, packet_len);
  return (packet_len);
}

/*
 * Receive ADU from HAL (HAL is the ZMQ publisher) - Blocks until it gets a valid adu
 */
void xdc_blocking_recv(void *socket, void *adu, gaps_tag *tag) {
  while (xdc_recv(socket, adu, tag) < 0);
}

