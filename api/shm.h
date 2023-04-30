#ifndef _SHM_H_
#define _SHM_H_

#include "xdcomms.h"

#define SHM_MMAP_ADDR_HOST     0x0UL            // Host System selects mmap physical memory address
#define SHM_MMAP_LEN_HOST      0x80000UL        // 0.5 MB
#define SHM_MMAP_ADDR_ESCAPE   0x2080000000UL   // mmap physical memory address @ 130 GB
#define SHM_MMAP_LEN_ESCAPE    0x10000000UL     // 256 MB
#define MMAP_PAGE_MASK     (sysconf(_SC_PAGE_SIZE) - 1)    // Normally 4K - 1

#define PKT_INDEX_MAX             2     // number of packet buffers in channel
#define DEFAULT_MS_GUARD_TIME_AW  5000  // Timing param to sync Tx write and Rx read
#define DEFAULT_MS_GUARD_TIME_BW  1000  // Timing param to sync Tx write and Rx read
//#define DEFAULT_MS_POLL_TIME      1000  // Timing param to sync Tx write and Rx read

/* Static SHM Channel Information (created when channel is initialized at TX) */
typedef struct _cinfo {
  uint32_t      ctag;
  int           pkt_index_max;
  unsigned long ms_guard_time_aw;    // After write min data duration (5000 ms)
  unsigned long ms_guard_time_bw;    // Before write min guard time (1000 ms)
  time_t        unix_seconds;        // Transmitter start time (seconds since 1970)
  uint16_t      crc16;               // cinfo error detection field
//  unsigned long ms_poll_time;      // How often to check SHM (1000 ms)
} cinfo;

/* SHM packet header (changed for each packet) */
typedef struct _pinfo {
  unsigned long data_length;       // Packet length
  unsigned long transaction_ID;    // Packet transaction ID
} pinfo;

/* SHM data */
typedef struct _pdata {
  //  unsigned long data[PKT_G1_ADU_SIZE_MAX];
  unsigned long data[0x1000];
} pdata;

/* SHM channel (one per TAG) */
typedef struct _shm_channel {
  cinfo  cinfo;
  int    pkt_index_last;            // Index to last valid packet
  int    pkt_index_next;            // Index to next packet to be written
  pinfo  pinfo[PKT_INDEX_MAX];
  pdata  pdata[PKT_INDEX_MAX];
} shm_channel __attribute__ ((aligned (1024)));		/*  byte alignment */

#endif /* _SHM_H_ */
