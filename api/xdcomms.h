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

#define ADU_SIZE_MAX_C  1000000     /* 1 MB - Increased for ILIP payload mode*/
#define RX_FILTER_LEN   12
#define DATA_TYP_MAX    200
/* Non Legacy versions */
#define MY_DATA_TYP_MAX 200

#define TX_CHANNEL_COUNT  1
#define RX_CHANNEL_COUNT  1
#define MAX_BUFS_PER_TAG  RX_BUFFER_COUNT
/**********************************************************************/
/* LIB Structures */
/*********t************************************************************/
/* Closure tag structure */
typedef struct _tag {
  uint32_t    mux;      /* APP ID */
  uint32_t    sec;      /* Security tag */
  uint32_t    typ;      /* data type */
} gaps_tag;

/* CLOSURE packet (for comms between application and HAL) */
typedef struct _sdh_ha_v1 {
  gaps_tag  tag;
  uint32_t  data_len;               /* 0 = no immediate data */
  uint8_t   data[ADU_SIZE_MAX_C];   /* Immediate data */
} sdh_ha_v1;

/* Table of codec per data types (Max of DATA_TYP_MAX types) */
typedef void (*codec_func_ptr)(void *, void *, size_t *);
typedef struct _codec_map {
  int             valid;
  uint32_t        data_type;
  codec_func_ptr  encode;
  codec_func_ptr  decode;
} codec_map;
//extern codec_map  cmap[DATA_TYP_MAX];   /* declare for global use (where?) */


/* DMA structures */
typedef struct channel {
  struct channel_buffer *buf_ptr;
  int fd;
  pthread_t tid;
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
//  struct channel_buffer *buf_ptr;
  chan    *c;
  dmamap  *dm;
//  int     fd;
//  int                    buffer_id;
} thread_args;

/* If using BW_v1 packet format */
#define BW_PACKET         /* choose between ha (not defined) and bw packet formats */

#ifdef BW_PACKET
#include "crc.h"
#define CTAG_MOD               256
#define PKT_G1_ADU_SIZE_MAX  65528      /* Max size with 16-bit data_laen = 2^16 - 8 (see bw header) */
/* BW Compressed Mode packet */
typedef struct _sdh_bw {
  uint32_t  message_tag_ID;             /* Compressed Application Mux, Sec, Typ */
  uint16_t  data_len;                   /* Length (in bytes) */
  uint16_t  crc16;                      /* Error detection field */
  uint8_t   data[PKT_G1_ADU_SIZE_MAX];  /* Application data unit */
} bw;
#endif

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
