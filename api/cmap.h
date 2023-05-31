#ifndef CMAP_HEADER_FILE
#define CMAP_HEADER_FILE

#include <stdio.h>
#include <stdint.h>
#include "log.h"
#include "crc.h"

#define DATA_TYP_MAX  50

/* CLOSURE tag structure */
typedef struct _tag {
  uint32_t         mux;      /* APP ID */
  uint32_t         sec;      /* Security tag */
  uint32_t         typ;      /* data type */
} gaps_tag;

/* Table of codec per data types (Max of DATA_TYP_MAX types) */
typedef void (*codec_func_ptr)(void *, void *, size_t *);
typedef struct _codec_map {
  int             valid;
  uint32_t        data_type;
  codec_func_ptr  encode;
  codec_func_ptr  decode;
} codec_map;

codec_map       cmap[DATA_TYP_MAX];           // maps data type to its data encode + decode functions

void cmap_encode(uint8_t *data, uint8_t *buff_in, size_t *buff_len, gaps_tag *tag);
void cmap_decode(uint8_t *data, size_t data_len, uint8_t *buff_out, gaps_tag *tag);
  
#endif /* CMAP_HEADER_FILE */
