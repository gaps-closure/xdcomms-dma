#ifndef _SHM_H_
#define _SHM_H_

#include "xdcomms.h"

#define SHM_MMAP_HOST   // Uncomment to use Host memory and comment-out to use ESCAPE (FPGA) memory
#ifdef  SHM_MMAP_HOST
#define SHM_MMAP_ADDR             0x0UL            // Host System selects mmap physical memory address
#define SHM_MMAP_LEN              0x100000UL       // 1 MB (limit on Unix)
//#define SHM_MMAP_LEN              0x80000UL       // 0.5 MB (limit on Unix)
#define SHM_PKT_COUNT             2                // packet buffers per channel (>1). Limit to fit in SHM_MMAP_LEN

#else
#define SHM_MMAP_ADDR             0x2080000000UL   // mmap ESCAPE physical memory address @ 130 GB
#define SHM_MMAP_LEN              0x10000000UL     // 256 MB
#define SHM_PKT_COUNT             2                // number of packet buffers/channel (>1). Larger = higher max tput)
#endif                 // SHM_MMAP_HOST

#define DEFAULT_NS_GUARD_TIME_AW  500  // Timing param to sync Tx write and Rx read
#define DEFAULT_NS_GUARD_TIME_BW  100  // Timing param prevent reading changing data
#define MMAP_PAGE_MASK            (sysconf(_SC_PAGE_SIZE) - 1)    // Normally 4K - 1

/* Static SHM Channel Information (created when channel is initialized at TX) */
typedef struct _cinfo {
  unsigned long ms_guard_time_aw;    // After write min data duration (5000 ms)
  unsigned long ms_guard_time_bw;    // Before write min guard time (1000 ms)
  time_t        unix_seconds;        // Tx start time (seconds since 1970)
  uint32_t      ctag;
  int           pkt_index_max;
} cinfo;

/* SHM packet header (changed for each packet) */
typedef struct _pinfo {
  unsigned long data_length;       // Packet length
  unsigned long transaction_ID;    // Packet transaction ID
} pinfo;

/* SHM data (0x8000 = 32KB, 0x9C40 = 40KB) */
typedef struct _pdata {
#ifdef SHM_MMAP_HOST
  uint8_t data[(0x8c00  / sizeof(uint8_t))];  // 35KB (36KB fails though there is room????)
#else
  uint8_t data[(0x10000 / sizeof(uint8_t))];  // 64KB
#endif //SHM_MMAP_HOST
} pdata;

/* SHM channel (one per TAG) */
typedef struct _shm_channel {
  cinfo    cinfo;
  uint16_t crc16;                 // cinfo error detection field
  int      pkt_index_last;        // Index to last valid packet
  int      pkt_index_next;        // Index to next packet to be written
  pinfo    pinfo[SHM_PKT_COUNT];  // packet info
  pdata    pdata[SHM_PKT_COUNT];  // Packet data
} shm_channel __attribute__ ((aligned (1024)));		/*  byte alignment */

  
#endif /* _SHM_H_ */
