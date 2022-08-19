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

#define DATA_TYP_MAX        200
#define CTAG_MOD            256
#define PKT_G1_ADU_SIZE_MAX 65528            /* Max size with 16-bit data_laen = 2^16 - 8 (see bw header) */

#define TX_CHANNEL_COUNT    1
#define RX_CHANNEL_COUNT    1
#define MAX_BUFS_PER_TAG    RX_BUFFER_COUNT  /* RX_BUFFER_COUNT from dma-proxy.h */

/* Closure tag structure */
typedef struct _tag {
  uint32_t    mux;      /* APP ID */
  uint32_t    sec;      /* Security tag */
  uint32_t    typ;      /* data type */
} gaps_tag;

/* Table of codec per data types (Max of DATA_TYP_MAX types) */
typedef void (*codec_func_ptr)(void *, void *, size_t *);
typedef struct _codec_map {
  int             valid;
  uint32_t        data_type;
  codec_func_ptr  encode;
  codec_func_ptr  decode;
} codec_map;

/* GAPS MIND Packet Format */
typedef struct _sdh_bw {
  uint32_t  message_tag_ID;             /* Compressed Application Mux, Sec, Typ */
  uint16_t  data_len;                   /* Length (in bytes) */
  uint16_t  crc16;                      /* Error detection field */
  uint8_t   data[PKT_G1_ADU_SIZE_MAX];  /* Application data unit */
} bw;

/* DMA structures */
typedef struct channel {
  struct channel_buffer *buf_ptr;
  int fd;
} chan;

/* node storing packet pointers to DMA rx buffer for a tag (in circular linked list) */
typedef struct _dmamap {
  gaps_tag               tag;
  pthread_mutex_t        lock;
  int                    index_r;
  int                    index_w;
  struct channel_buffer *cbuf_ptr[MAX_BUFS_PER_TAG];
  struct _dmamap        *next;
} dmamap;

typedef struct _thread_args {
  chan    *c;
  int      buffer_id;
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
