/*
 * Cross Domain (XD) Communication API between Applicaitons and GAP XD Guards
 * v0.2, August 2022
 *
 * This version of xdcomms supports Cross Domain communication by directly
 * (without HAL) communicating with the GE MIND DMA Device Driver (DDD). The
 * DDD itself is a proxy for kernel space DMA control to the DMA Engine (using
 * the XILINX AXI DMA / MCDMA driver). The module is based on the MIND user-
 * space test APP (dma_proxy_test.c).
 *
 * STEPS:
 *  1) INSTALL the DDD kernel module (automatic on ZCU102 board or QEMU emulation)
 *         modprobe dma-proxy.ko
 *         ls /dev/dma*  (/dev/dma_proxy_rx  /dev/dma_proxy_tx)
 *     DDD supports multiple driver instances (with multiple IPs).
 *
 *  2) OPEN the tx and rx channels (created by the DDD)
 *         tx_channels[i].fd open(tx_channel_name, O_RDWR);
 *         rx_channels[i].fd open(rx_channel_name, O_RDWR);
 *
 *  3) COMMS: The DDD provides a zero copy scheme by passing the mmap'ed address
 *     of the kernel allocated memory region (channel buffer). This moduule eses
 *     the DDD provided Ioctl functions to start (non-blocking) and finish
 *      (blocking) DMA transfer:
 *         ioctl(channel_ptr->fd, FINISH_XFER, &buffer_id);
 *         ioctl(channel_ptr->fd,  START_XFER, &buffer_id);
 *     DDD defines the number (N) of channel buffers: currently, TX_BUFFER_COUNT=1
 *     RX_BUFFER_COUNT=32. Thus, the range of buffer_id's is greater for RX (0-31)
 *     than TX (0).
 *
 *  4) COMPILE the XDCOMMS library and aApplicaiton:
 *       amcauley@liono:~/gaps/build/hal/api$ make -f Makefile_xdc_dma
 *       amcauley@liono:~/gaps/build/hal/test$ make clean; make; ./app_req_rep -v -l0
 *
 *  5) TEST WITHOUT DDD: this program can use shared memory for communication if we
 *     #define SHARED_MEMORY_MODE, then allocate two shared memory files:
 *         dd if=/dev/zero of=green_to_orange_channel bs=4k count=1000
 *     a) As DMA communication use the transmit and receive channels in all enclaves
 *        we first run transmitter into green_to_orange_channel:
 *       amcauley@liono:~/gaps/build/hal/test$ ./app_req_rep -v -l0
 * 20:05:35 DEBUG xdcomms_dma.c.c:354: HAL API address set to: ipc:///tmp/halsubgreen
 * 20:05:35 DEBUG xdcomms_dma.c.c:354: HAL API address set to: ipc:///tmp/halpubgreen
 * Enclave 1 has tags: pub=[mux=01 sec=01 typ=01]  sub=[mux=02 sec=02 typ=01]
 * 20:05:35 DEBUG xdcomms_dma.c.c:237: API registered new data typ = 1 (index=0)
 * 20:05:35 DEBUG xdcomms_dma.c.c:237: API registered new data typ = 3 (index=1)
 * 20:05:35 DEBUG xdcomms_dma.c.c:237: API registered new data typ = 19088743 (index=2)
 * 20:05:35 DEBUG xdcomms_dma.c.c:237: API registered new data typ = 13 (index=3)
 * 20:05:35 DEBUG xdcomms_dma.c.c:237: API registered new data typ = 113 (index=4)
 * 20:05:35 TRACE xdcomms_dma.c.c:403: API creates a new ZMQ context (p=0x5650c8593380)
 * 20:05:35 TRACE xdcomms_dma.c.c:419: API connects (spck=0x5650c85979d0 t=1) to ipc:///tmp/halpubgreen
 * 20:05:36 TRACE xdcomms_dma.c.c:459: API connects to ipc:///tmp/halsubgreen (sock=0x5650c85a1ae0 timeout(ms)=-1 match Len=12)
 * app tx: position (len=40): -74.574489, 40.695545, 102.000000, 1, 0, 0, 0, 0
 * 20:05:36 TRACE xdcomms_dma.c.c:644: Start of xdc_asyn_send
 * 20:05:36 TRACE xdcomms_dma.c.c:571: Start of open_channel
 * 20:05:36 TRACE xdcomms_dma.c.c:586: Openned channel 0: green_to_orange_channel (ptr=0x7fe8f4425000, fd=10)
 *    TRACE API <- raw app data: (len=40) 5ABA826D C4A452C0 BBF2599E 07594440 00000000 00805940 01000000 00000000 00000000 00000000
 *    TRACE     -> encoded data: (len=40) 5ABA826D C4A452C0 BBF2599E 07594440 00000000 00805940 00000001 00000000 00000000 00000000
 *    TRACE API sends Packet (len=56) 00000001 00000001 00000001 00000028 5ABA826D C4A452C0 BBF2599E 07594440 00000000 00805940 00000001 00000000 00000000 00000000
 *
 *       amcauley@liono:~/gaps/build/hal/test$ hexdump green_to_orange_channel
 *          0000000 0000 0100 0000 0100 0000 0100 0000 2800
 *          0000010 ba5a 6d82 a4c4 c052 f2bb 9e59 5907 4044
 *          0000020 0000 0000 8000 4059 0000 0100 0000 0000
 *
 *     b) Copy green to oragnge: so receiver (enclave #2, -e 2) sees a packet in
 *        its receive buffer. Note: packet length is 56 bytes (0x38 in line 0020000)
 *        amcauley@liono:~/gaps/build/hal/test$ cp green_to_orange_channel orange_to_green_channel
 *        amcauley@liono:~/gaps/build/hal/test$ hexdump orange_to_green_channel
 *           0000000 0000 0100 0000 0100 0000 0100 0000 2800
 *           0000010 ba5a 6d82 a4c4 c052 f2bb 9e59 5907 4044
 *           0000020 0000 0000 8000 4059 0000 0100 0000 0000
 *           0000030 0000 0000 0000 0000 0000 0000 0000 0000
 *
 *           0020000 0000 0000 0038 0000 0000 0000 0000 0000
 *           0020010 0000 0000 0000 0000 0000 0000 0000 0000
 *
 *           03e8000
 *
 *     c) RECEIVE
 *        amcauley@liono:~/gaps/build/hal/test$ make clean; make; ./app_req_rep -v -l0 -e 2
 *            21:08:35 TRACE xdcomms_dma.c.c:547: rx_thread: Ready to Read data from fd=10 (len=56)
 *            21:08:35 TRACE xdcomms_dma.c.c:550: rx_thread
 *                 TRACE API receives Packet (len=56) 00000001 00000001 00000001 00000028 5ABA826D C4A452C0 BBF2599E 07594440 00000000 00805940 00000001 00000000 00000000 00000000
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
#include <sched.h>
#include <time.h>
#include <errno.h>
#include <sys/param.h>

#include <zmq.h>
#include "xdcomms.h"
#include "dma-proxy.h"

#define TX_CHANNEL_COUNT 1
#define RX_CHANNEL_COUNT 1

typedef struct channel {
  struct channel_buffer *buf_ptr;
  int fd;
  pthread_t tid;
} chan;

typedef struct _thread_args {
  struct channel_buffer *buf_ptr;
  int                    fd;
  int                    buffer_id;
} thread_args;

codec_map  cmap[DATA_TYP_MAX];    /* maps data type to its data encode + decode functions */

/**********************************************************************/
/* A) Set API Logging to a new level */
/**********************************************************************/
void xdc_log_level(int new_level) {
  static int do_once = 1;
  
  // set to default if User has not already set
  if (new_level == -1) {
    if (do_once == 1) {
      log_set_quiet(0);               /* not quiet */
      log_set_level(LOG_INFO);        /* default level */
//      log_set_level(LOG_TRACE);       /* test */
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
/* Bm) Non-Legacy Tag processing (all just aliases) */
/**********************************************************************/
void my_tag_write (gaps_tag *tag, uint32_t mux, uint32_t sec, uint32_t typ) {
  tag_write (tag, mux, sec, typ);
}
void my_tag_encode (gaps_tag *tag_out, gaps_tag *tag_in) {
  tag_encode (tag_out, tag_in);
}
void my_tag_decode (gaps_tag *tag_out, gaps_tag *tag_in) {
  tag_decode (tag_out, tag_in);
}
void my_len_encode (uint32_t *out, size_t len) {
  len_encode (out, len);
}
void my_len_decode (size_t *out, uint32_t in) {
  len_decode (out, in);
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

/**********************************************************************/
/* Cm) Non-Legacy CMAP table to store encoding and decoding function pointers */
/**********************************************************************/
/* Non-legacy tag data_type used as direct index, so must be less than MY_DATA_TYP_MAX */
void my_type_check(uint32_t typ, codec_map *cmap) {
    if ( (typ >= MY_DATA_TYP_MAX) || (cmap[typ].valid==0) ) {
        exit (1);
    }
}

/* tag data_type used as direct index (Legacy uses first available as index and stored data type) */
void my_xdc_register(codec_func_ptr encode, codec_func_ptr decode, int typ, codec_map *cmap) {
    cmap[typ].valid=1;
    cmap[typ].encode=encode;
    cmap[typ].decode=decode;
}

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
  
  
/**********************************************************************/
/* Dm) Non-Legacy Encocde/decode data into/from a HAL packet */
/**********************************************************************/
void my_gaps_data_encode(sdh_ha_v1 *p, size_t *p_len, uint8_t *buff_in, size_t *len_out, gaps_tag *tag, codec_map *cmap) {
    uint32_t typ = tag->typ;
    my_type_check(typ, cmap);
    cmap[typ].encode (p->data, buff_in, len_out);
    log_buf_trace("my API <- raw app data:", buff_in, *len_out);
    log_buf_trace("       -> encoded data:", p->data, *len_out);
    my_tag_encode(&(p->tag), tag);
    log_trace("A");
    my_len_encode(&(p->data_len), *len_out);
    log_trace("B");
    *p_len = (*len_out) + sizeof(p->tag) + sizeof(p->data_len);
    log_trace("my_gaps_data_encode: typ=%d p_len=%d", typ, p_len);
}
void my_gaps_data_decode(sdh_ha_v1 *p, size_t p_len, uint8_t *buff_out, size_t *len_out, gaps_tag *tag, codec_map *cmap) {
    uint32_t typ = tag->typ;
    my_type_check(typ, cmap);
    my_tag_decode(tag, &(p->tag));
    my_len_decode(len_out, p->data_len);
    cmap[typ].decode (buff_out, p->data, &p_len);
    }

/**********************************************************************/
/* E) Set and Get APP-HAL API Addresses */
/**********************************************************************/
/*
 * Set static address (xdc_addr) if addr_in != NULL
 *   and initialize to addr_default when do_once == 1
 */
void set_address(char *xdc_addr, char *addr_in, const char *addr_default, int *do_once) {
    if (*do_once == 1) {
      xdc_log_level(-1);            /* set logging level to default (if not set) */
      if (strlen(addr_default) >= 255) {
        log_fatal("API IPC address default is too long: %s", addr_default);
        exit(1);
      }
      *do_once = 0;
      strcpy(xdc_addr, addr_default);
    }
    if (addr_in != NULL) {
      if (strlen(addr_in) >= 255) {
        log_warn("Input too long (%d), not changing: %s", strlen(addr_in), addr_in);
      } else {
        strcpy(xdc_addr, addr_in);
        log_debug("HAL API address set to: %s", xdc_addr);
      }
    }
}

/*
 * Set and Get IPC Addresses where APP client subscribes to HAL publisher
 * TODO: change name to xdc_addr_sub
 */
char *xdc_set_in(char *addr_in) {
  static int do_once = 1;
  static char xdc_addr[256];
  set_address(xdc_addr, addr_in, IPC_ADDR_DEFAULT_HALSUB, &do_once);
//  fprintf(stderr, "%s = %s\n", __func__, xdc_addr);
  return xdc_addr;
}

/*
 * Set and Get A IPC Addresses where APP publishes to HAL subscriber
 * TODO: change name to xdc_addr_pub
 */
char *xdc_set_out(char *addr_in) {
  static int do_once = 1;
  static char xdc_addr[256];
  set_address(xdc_addr, addr_in, IPC_ADDR_DEFAULT_HALPUB, &do_once);
// fprintf(stderr, "%s = %s\n", __func__, xdc_addr);
  return xdc_addr;
}

/**********************************************************************/
/* F) Legacy ZMQ-based Communication Setup */
/**********************************************************************/
/*
 * Exit with ZMQ error message
 */
void exit_with_zmq_error(const char* where) {
  log_fatal("HAL API exits after %s error %d: %s\n", where, errno, zmq_strerror(errno));
  exit(-1);
}

/*
 * Get zmq context (and create if not already created) - once per process
 */
void *xdc_ctx(void) {
  static void *ctx = NULL;
  if (ctx == NULL) {
    xdc_log_level(-1);            /* set logging level to default (if not set) */
    ctx = zmq_ctx_new();
    if(ctx == NULL) exit_with_zmq_error("zmq_ctx_new");
    log_trace("API creates a new ZMQ context (p=%p)", ctx);
  }
  return ctx;
}

/*
 * Open ZMQ Publisher socket, connecting to HAL subscriber listening at addreess set by xdc_set_out:
 */
void *xdc_pub_socket(void) {
    int      err;
    void    *socket;

    socket = zmq_socket(xdc_ctx(), ZMQ_PUB);
    if (socket == NULL) exit_with_zmq_error("zmq_socket");
    err = zmq_connect(socket, xdc_set_out(NULL));
    if (err) exit_with_zmq_error("zmq_connect");
    log_trace("API connects (spck=%p t=%d) to %s", socket, ZMQ_PUB, xdc_set_out(NULL));
    /*
     * HAL subscriber binds to address (usually publisher would bind).
     * The APP-API cannot send immediately after a connect, as there
     * is a few msec before ZMQ creates outgoing pipe (so sleep 1)
     */
    sleep (1);
    return socket;
}

/*
 * Open non-blocking ZMQ Subscriber socket, with timeout specified in milliseconds
 * (HAL is a ZMQ Publisher listening on the address configured by xdc_set_in)
 */
void *xdc_sub_socket_non_blocking(gaps_tag tag, int timeout) {
    int      err, len;
    void    *socket;
    gaps_tag tag4filter;
    void    *filter;

    socket = zmq_socket(xdc_ctx(), ZMQ_SUB);
    if (socket == NULL) exit_with_zmq_error("zmq_socket");
  
    if (timeout>=0) {
        err = zmq_setsockopt(socket, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));
        assert(err == 0);
    }

    err = zmq_connect(socket, xdc_set_in(NULL));
    if (err) exit_with_zmq_error("zmq_connect");
    
    if ((tag.mux) != 0) {
        len = RX_FILTER_LEN;
        tag_encode(&tag4filter, &tag);
        filter = (void *) &tag4filter;
    } else {
        len = 0;
        filter = (void *) "";
    }
      
    log_trace("API connects to %s (sock=%p timeout(ms)=%d match Len=%d)", xdc_set_in(NULL), socket, timeout, len);
    
    err = zmq_setsockopt(socket, ZMQ_SUBSCRIBE, filter, len);
    assert(err == 0);
    return socket;
}

/*
 * Open ZMQ Subscriber socket, connecting to HAL publisher listening at addreess set by xdc_set_in:
 */
void *xdc_sub_socket(gaps_tag tag) {
  return (xdc_sub_socket_non_blocking(tag, -1));      /* Recv call blocks forever */
//  return (xdc_sub_socket_non_blocking(tag, 10000));       /* Receive call block for 10 seconds */
}

/**********************************************************************/
/* Fm) Non-Legacy ZMQ-based Communication Setup (URI per thread) */
/**********************************************************************/
void *my_xdc_pub_socket(void *ctx, const char *outuri) {
    int err;
    void *socket;
    socket = zmq_socket(ctx, ZMQ_PUB);
    err = zmq_connect(socket, outuri);
    assert(err == 0);
    return socket;
}

void *my_xdc_sub_socket(gaps_tag tag, void *ctx, const char *inuri) {
    int err;
    gaps_tag tag4filter;
    void *socket;
    socket = zmq_socket(ctx, ZMQ_SUB);
    err = zmq_connect(socket, inuri);
    my_tag_encode(&tag4filter, &tag);
    err = zmq_setsockopt(socket, ZMQ_SUBSCRIBE, (void *) &tag4filter, RX_FILTER_LEN);
    assert(err == 0);
    return socket;
}

void *my_xdc_sub_socket_non_blocking(gaps_tag tag, void *ctx, int timeout, const char *inuri) {
    int  err, len;
    void    *socket;
    gaps_tag tag4filter;
    void    *filter;
    socket = zmq_socket(ctx, ZMQ_SUB);
    if (timeout>=0) {
        err = zmq_setsockopt(socket, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));
        assert(err == 0);
    }
    err = zmq_connect(socket, inuri);
    if ((tag.mux) != 0) {
        len = RX_FILTER_LEN;
        my_tag_encode(&tag4filter, &tag);
        filter = (void *) &tag4filter;
    } else {
        len = 0;
        filter = (void *) "";
    }
    err = zmq_setsockopt(socket, ZMQ_SUBSCRIBE, filter, len);
    assert(err == 0);
    return socket;
}

/**********************************************************************/
/* X) Tx/Rx Threads and DMA Open/Map device  */
/**********************************************************************/
/*
 * Open channel and save virtual address of buffer pointer
 */
int open_channel(chan *c, const char **channel_name, int channel_count, int buffer_count) {
  int i;
  
  log_trace("Start of %s", __func__);
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
    log_trace("Openned channel %d: %s (ptr=%p, fd=%d)", i, channel_name[i], c[i].buf_ptr, c[i].fd);
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
/*
 * Perform DMA ioctl operations to tx or rx data
 */
#ifndef SHARED_MEMORY_MODE
void *dma_start_to_finish(int fd, int *buffer_id_ptr, struct channel_buffer *channel_buffer_ptr) {
  ioctl(fd, START_XFER,  buffer_id_ptr);
  ioctl(fd, FINISH_XFER, buffer_id_ptr);
  if (channel_buffer_ptr->status != PROXY_NO_ERROR) {
    log_warn("Proxy tx transfer error");
  }
  log_trace("%s: Completed data transfer of buffer id=%d (fd=%d)", __func__, *buffer_id_ptr, fd);
  return(NULL);
}
#endif

/*
 * Transmit thread sends packet already stored in channel structure
 * (As ioctl calls are blocking so that multiple threads are required)
 */

void *tx_thread(thread_args *vargs) {
  log_trace("%s: ptr=%p fd=%d id=%d len=%d", __func__, vargs->buf_ptr, vargs->fd, vargs->buffer_id, vargs->buf_ptr->length);
#ifndef SHARED_MEMORY_MODE
    dma_start_to_finish(vargs->fd, &(vargs->buffer_id), vargs->buf_ptr);
#endif
  return (NULL);
}

/*
 * Send Pack to DMA driver in a new thread
 */
void dma_channel_buffer(chan *tx_channel, size_t packet_len, void *thread_func, int buffer_count) {
  static int buffer_id=0;
  thread_args  args;
  
  tx_channel->buf_ptr[buffer_id].length = packet_len;
  args.buf_ptr   = &(tx_channel->buf_ptr[buffer_id]);
  args.fd        = tx_channel->fd;
  args.buffer_id = buffer_id;
  log_buf_trace("API sends Packet", (uint8_t *) args.buf_ptr, packet_len);
  log_trace("%s: ptr=%p fd=%d id=%d", __func__, args.buf_ptr, args.fd, args.buffer_id);

  pthread_create(&(tx_channel->tid), low_thread_priority(), thread_func, (void *)&args);
  
  /* Flip to next buffer, treating them as a circular list */
  buffer_id += BUFFER_INCREMENT;
  buffer_id %= buffer_count;
}

/*
 * Send Pack to DMA driver in a new thread
 */
void receive_channel_buffer(chan *c, size_t *packet_len) {

  log_trace("Start of %s: Ready to Read data from fd=%d (len=%d)", __func__, c->fd, c->buf_ptr[0].length);
//  pthread_create(&(rx_channel->tid), NULL, (void *) rx_thread, (void *)rx_channel);
#ifdef SHARED_MEMORY_MODE
  while (c->buf_ptr[0].length < 1 ) {
    sleep(1);
    log_trace("%s len=%d, data=%x ptr=(%p-%p=%x)", __func__, c->buf_ptr[0].length, c->buf_ptr[0].buffer[0], c->buf_ptr[0].buffer, &(c->buf_ptr[0].length), c->buf_ptr[0].buffer - (&(c->buf_ptr[0].length)));
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
  sdh_ha_v1  *p;                                        /* Packet pointer */
  chan        tx_channels[TX_CHANNEL_COUNT];            /* DMA channels */
#ifdef SHARED_MEMORY_MODE
  const char *tx_channel_names[] = { "green_to_orange_channel", /* unique channel name(s) here */ };
#else
  const char *tx_channel_names[] = { "dma_proxy_tx", /* add unique channel names here */ };
#endif
  
#ifdef SHARED_MEMORY_MODE
  log_trace("%s: SHARED", __func__);
#else
  log_trace("%s: IOCTL", __func__);
#endif
  
  log_trace("Start of %s", __func__);
  /* open channel and save virtual address of buffer pointer (in tx_channels) */
  if (once == 1) once = open_channel(tx_channels, tx_channel_names, TX_CHANNEL_COUNT, TX_BUFFER_COUNT);
  p = (sdh_ha_v1 *) tx_channels[i].buf_ptr;              /* DMA channel buffer holds created packet */
  gaps_data_encode(p, &packet_len, adu, &adu_len, tag);  /* Put packet into channel buffer */
  dma_channel_buffer(&tx_channels[i], packet_len, (void *)tx_thread, TX_BUFFER_COUNT);  /* Send packet */
}

/*
 * Receive ADU from HAL (HAL is ZMQ publisher) from a sdh_ha_v1 packet
 * Returs size of packet received (timeout/error if < 0)
 */
int xdc_recv(void *socket, void *adu, gaps_tag *tag) {
  int         i=0;                                      /* Initially assume 1 DMA channel */
  static int  once=1;
  size_t      packet_len=0, adu_len=0;                      /* Note; encoder calculates lengiha */
  sdh_ha_v1  *p;                                        /* Packet pointer */
  chan        rx_channels[TX_CHANNEL_COUNT];            /* DMA channels */
#ifdef SHARED_MEMORY_MODE
  const char *rx_channel_names[] = { "orange_to_green_channel", /* add unique channel names here */ };
#else
  const char *rx_channel_names[] = { "dma_proxy_rx", /* add unique channel names here */ };
#endif


  log_trace("Start of %s", __func__);
  /* open channel and save virtual address of buffer pointer (in channel structure) */
  if (once == 1) once =
  open_channel(rx_channels, rx_channel_names, RX_CHANNEL_COUNT, RX_BUFFER_COUNT);
  log_trace("%s: once-%d buf_ptr=%d", __func__, once, &(rx_channels[i].buf_ptr[0]));
  receive_channel_buffer(&(rx_channels[i]), &packet_len);  /* Wait for packet in channel buffer */
  p = (sdh_ha_v1 *) &(rx_channels[i].buf_ptr[0]);          /* DMA channel buffer with packet */
  log_trace("%s: ptr=%p = %p", __func__, p, rx_channels[i].buf_ptr);
  log_buf_trace("API recv packet", (uint8_t *) p, packet_len);
  gaps_data_decode(p, packet_len, adu, &adu_len, tag);  /* Put packet into ADU */
//  gaps_data_encode(p, &packet_len, adu, &adu_len, tag);  /* Put packet into ADU */
  exit(33);

  return (packet_len);
}

/*
 * Receive ADU from HAL (HAL is the ZMQ publisher) - Blocks until it gets a valid adu
 */
void xdc_blocking_recv(void *socket, void *adu, gaps_tag *tag) {
  while (xdc_recv(socket, adu, tag) < 0);
}

/**********************************************************************/
/* Gm) Non-Legacy ZMQ Communication Send and Receive */
/**********************************************************************/
void my_xdc_asyn_send(void *socket, void *adu, gaps_tag *tag, codec_map *cmap) {
    sdh_ha_v1    packet, *p=&packet;
    size_t       packet_len;
    size_t adu_len;         /* Size of ADU is calculated by encoder */
    log_trace("my API send encodes ");
    my_gaps_data_encode(p, &packet_len, adu, &adu_len, tag, cmap);
    log_buf_trace("my API sends Packet", (uint8_t *) p, packet_len);
    exit (33);

    int bytes = zmq_send (socket, (void *) p, packet_len, 0);
    if (bytes <= 0) fprintf(stderr, "send error %s %d ", zmq_strerror(errno), bytes);
}

void my_xdc_blocking_recv(void *socket, void *adu, gaps_tag *tag, codec_map *cmap) {
    sdh_ha_v1 packet;
    void *p = &packet;
    log_trace("my API waiting to recv packet");
    int size = zmq_recv(socket, p, sizeof(sdh_ha_v1), 0);
    size_t adu_len;
    my_gaps_data_decode(p, size, adu, &adu_len, tag, cmap);
}

int my_xdc_recv(void *socket, void *adu, gaps_tag *tag, codec_map *cmap) {
    sdh_ha_v1 packet;
    void *p = &packet;
    int size = zmq_recv(socket, p, sizeof(sdh_ha_v1), 0);
    size_t adu_len;
    my_gaps_data_decode(p, size, adu, &adu_len, tag, cmap);
    return size;
}

