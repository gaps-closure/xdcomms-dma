#ifndef XDC_HEADER_FILE
#define XDC_HEADER_FILE

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <assert.h>
#include "log.h"
#include "dma-proxy.h"
#include "crc.h"

#define DATA_TYP_MAX                      50
#define GAPS_TAG_MAX                      50
#define CTAG_MOD                         256
#define PKT_G1_ADU_SIZE_MAX            65528  // Max packet size with 16-bit data_len = 2^16 - 8 (see bw header)
#define ADU_SIZE_MAX_C               1000000     /* 1 MB - Increased for ILIP payload mode*/

// Buffer allocation to threads. NB: RX_THREADS * RX_BUFFS_PER_THREAD <= RX_BUFFER_COUNT
#define RX_THREADS                         1  // Total number of receiver threads
#define RX_BUFFS_PER_THREAD  RX_BUFFER_COUNT  // Use all DMA rx buffers in one rx channel thread

// How often (interval) and how long (timout) to check for rx packet (check 'newd' flag in Per-tag buffer).
//   Lower interval means lower delay, higher means less overhead
//   Timout in milliseconds = RX_POLL_TIMEOUT_MSEC_DEFAULT * num_retries / NSEC_IN_MSEC
//     - Default timeout value can be overridden by envionmental variable TIMEOUT_MS
//    -  User can override deafult timout value per tag in xdc_sub_socket_non_blocking() call
#define NSEC_IN_SEC               1000000000  // 10^9
#define NSEC_IN_MSEC                 1000000  // 10^6
#define RX_POLL_INTERVAL_NSEC        2000000  // Poll Interval in nanpseconds e.g. 1000000 = checks every 1ms
#define RX_POLL_TIMEOUT_MSEC_DEFAULT      40  // Default Total Poll time in milliseconds
/* Per-tag Rx buffer stores retries (based on timeout) per tag value */

/* MIND packet format */
typedef struct _sdh_bw {
  uint32_t  message_tag_ID;             /* Compressed Application Mux, Sec, Typ */
  uint16_t  data_len;                   /* Length (in bytes) */
  uint16_t  crc16;                      /* Error detection field */
  uint8_t   data[PKT_G1_ADU_SIZE_MAX];  /* Application data unit */
} bw;
/* MIND DMA channel structure */
typedef struct channel {
  struct channel_buffer *buf_ptr;
  int fd;
} chan;

/* CLOSURE tag structure */
typedef struct _tag {
  uint32_t    mux;      /* APP ID */
  uint32_t    sec;      /* Security tag */
  uint32_t    typ;      /* data type */
} gaps_tag;
/* CLOSURE Per-tag Rx information (stroed as linked list) */
typedef struct _tx_tag_info {
  gaps_tag              tag;
  int                   retries; // number of rx retries (passed using xdc_sub_socket_non_blocking())
  struct _tx_tag_info  *next;    // linked list *
} tx_tag_info;
/* CLOSURE Per-tag Rx information (stored as array list) */
typedef struct _rx_tag_info {
  uint32_t              ctag;    // Compressed tag (unique index)
  pthread_mutex_t       lock;    // Ensure thread does not write while xdcomms reads
  char                  newd;    // set to indicate received new packet (reset after reading)
  bw                   *p_ptr;   // XYZ2 Replace with void *
  int                   retries; // number of rx retries (every RX_POLL_INTERVAL_NSEC)
                                 //  - value can be passed using xdc_sub_socket_non_blocking())
} rx_tag_info;

/* Table of codec per data types (Max of DATA_TYP_MAX types) */
typedef void (*codec_func_ptr)(void *, void *, size_t *);
typedef struct _codec_map {
  int             valid;
  uint32_t        data_type;
  codec_func_ptr  encode;
  codec_func_ptr  decode;
} codec_map;

/* Receiver thread arguments */
typedef struct _thread_args {
  chan    *c;
  int      buffer_id_start;
} thread_args;

extern void tag_print     (gaps_tag *, FILE *);
extern void tag_write     (gaps_tag *, uint32_t,   uint32_t,   uint32_t);
extern void tag_read      (gaps_tag *, uint32_t *, uint32_t *, uint32_t *);
extern void tag_cp        (gaps_tag *tag_out, gaps_tag *tag_in);
extern void tag_encode    (gaps_tag *, gaps_tag *);
extern void tag_decode    (gaps_tag *, gaps_tag *);
extern void len_encode    (uint32_t *, size_t);
extern void len_decode    (size_t *, uint32_t);
extern void xdc_log_level (int new_level);
extern void xdc_register(codec_func_ptr encoder, codec_func_ptr decoder, int type);

extern char *xdc_set_in (char *addr); 
extern char *xdc_set_out(char *addr);
extern void *xdc_ctx(void);
extern void *xdc_pub_socket(void);
extern void *xdc_sub_socket_non_blocking(gaps_tag tag, int timeout);
extern void *xdc_sub_socket(gaps_tag tag);

extern void xdc_asyn_send(void *socket, void *adu, gaps_tag *tag);
extern void xdc_blocking_recv(void *socket, void *adu, gaps_tag *tag);
extern int  xdc_recv(void *socket, void *adu, gaps_tag *tag);

#endif /* XDC_HEADER_FILE */
