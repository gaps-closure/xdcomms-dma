// Codec map table to encode and decode function pointers
#include "cmap.h"

void cmap_print_one(codec_map *cm) {
  fprintf(stderr, "[typ=%d ", cm->data_type);
  fprintf(stderr, "e=%p ",    cm->encode);
  fprintf(stderr, "d=%p] ",   cm->decode);
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
