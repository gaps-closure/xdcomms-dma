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
#include "crc.h"


#define DATA_TYP_MAX                      50
#define GAPS_TAG_MAX                      50
#define CTAG_MOD                         256
#define PKT_G1_ADU_SIZE_MAX            65528  // Max packet size with 16-bit data_len = 2^16 - 8 (see bw header)
#define ADU_SIZE_MAX_C               1000000     /* 1 MB - Increased for ILIP payload mode*/
#define MAX_DEV_NAME_LEN                  64
#define MAX_DEV_COUNT                      4

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
#define RX_POLL_INTERVAL_NSEC        1000000  // Poll Interval in nanpseconds e.g. 1000000 = checks every 1ms
#define RX_POLL_TIMEOUT_MSEC_DEFAULT      40  // Default Total Poll time in milliseconds

#define FATAL do { fprintf(stderr, "Error at line %d, file %s (%d) [%s]\n", \
    __LINE__, __FILE__, errno, strerror(errno)); exit(1); } while(0)

#define DEV_DIR_IN  0
#define DEV_DIR_OUT 1

/* Per-tag Rx buffer stores retries (based on timeout) per tag value */

/* Table of codec per data types (Max of DATA_TYP_MAX types) */
typedef void (*codec_func_ptr)(void *, void *, size_t *);
typedef struct _codec_map {
  int             valid;
  uint32_t        data_type;
  codec_func_ptr  encode;
  codec_func_ptr  decode;
} codec_map;

/* CLOSURE tag structure */
typedef struct _tag {
  uint32_t         mux;      /* APP ID */
  uint32_t         sec;      /* Security tag */
  uint32_t         typ;      /* data type */
} gaps_tag;

// channel configuration (with device abstraction)
typedef struct channel {
  uint32_t         ctag;           // Compressed tag (unique index) - used to search for channel
  char             dir;            // Receive (from network) or Transmit (to network): 'r' or 't'
  char             dev_type[4];    // device type: e.g., shm (ESCAPE) or dma (MIND)
  char             dev_name[64];   // Device name: e.g., /dev/mem or /dev/sue_dominous
  int              fd;             // Device file descriptor (set when device openned)
  int              mmap_prot;      // Mmap protection field (e.g., Read and/or write)
  int              mmap_flags;     // mmap'ed flags (e.g., SHARED)
  unsigned long    mmap_phys_addr; // mmap'ed physical address
  unsigned long    mmap_len;       // mmap'ed memory length
  void            *mmap_virt_addr; // Mmaped virtual address of packet buffer structure
                                   // (channel_buffer in DMA device, shm_channel in SHM device)
  unsigned long    addr_offset;    // Offset from mmap_virt_addr to channel info
  pthread_mutex_t  lock;           // Ensure RX thread does not write while xdcomms reads
  char             newd;           // RX thread received new packet (xdcomms resets after reading)
  int              retries;        // number of RX polls (every RX_POLL_INTERVAL_NSEC) before timeout
  int              count;          // number of RX polls (every RX_POLL_INTERVAL_NSEC) before timeout
} chan;

/* RX thread arguments */
typedef struct _thread_args {
  chan            *cp;               // Channel RX thread is looking for
  void            *buffer_id_start; // Device buffer index
} thread_args;

void *rcvr_thread_function(thread_args *vargs);

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
