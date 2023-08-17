#ifndef XDC_HEADER_FILE
#define XDC_HEADER_FILE

#include <stddef.h>
//#include <stdio.h>
//#include <json-c/json.h>
#include "cmap.h"

#define GAPS_TAG_MAX                      32   // MAX mnumber of Tags for this node
#define CTAG_MOD                         256
#define ADU_SIZE_MAX_C               1000000   // 1 MB - Increased for ILIP payload mode*/
#define MAX_DEV_NAME_LEN                  64
#define MAX_DEV_COUNT                      4

// How often (interval) and how long (timout) to check for rx  (check 'newd'  in pkt_info buffer).
//   Lower interval means lower delay, higher means less overhead
//   Timout in milliseconds = RX_POLL_TIMEOUT_MSEC_DEFAULT * num_retries / NSEC_IN_MSEC
//     - Default timeout value can be overridden by envionmental variable TIMEOUT_MS
//    -  User can override deafult timout value per tag in xdc_sub_socket_non_blocking() call
#define NSEC_IN_SEC               1000000000  // 10^9
#define NSEC_IN_MSEC                 1000000  // 10^6
#define RX_POLL_INTERVAL_NSEC        1000000  // Poll Interval in nanpseconds e.g. 1000000 = checks every 1ms
#define RX_POLL_TIMEOUT_MSEC_DEFAULT      40  // Default Total Poll time in milliseconds

#define FATAL do { fprintf(stderr, "Error at line %d, file %s (%d) [%s]\n", \
    __LINE__, __FILE__, errno, strerror(errno)); exit(1); } while(0)

#define DEV_DIR_IN  0
#define DEV_DIR_OUT 1

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
