#ifndef _SHM_H_
#define _SHM_H_

#define MMAP_LEN_HOST      0x80000UL        // 0.5 MB
#define MMAP_LEN_ESCAPE    0x10000000UL     // 256 MB
#define MMAP_ADDR_ESCAPE   0x2080000000UL   // mmap physical memory address @ 130 GB
#define MMAP_ADDR_HOST     0x0UL            // System selectsmmap physical memory address
#define MMAP_PAGE_MASK     (sysconf(_SC_PAGE_SIZE) - 1)    // Normally 4K - 1

#define SHM_MAX_INDEX             16    // number of packet buffers in channel
#define DEFAULT_MS_GUARD_TIME_AW  5000  // Timing param to sync Tx write and Rx read
#define DEFAULT_MS_GUARD_TIME_BW  1000  // Timing param to sync Tx write and Rx read
#define DEFAULT_MS_POLL_TIME      1000  // Timing param to sync Tx write and Rx read

/* Static SHM channel structure (created when channel is initialized) */
typedef struct _cinfo {
  uint32_t      ctag;
  unsigned long ms_guard_time_aw;  // After write min data duration (5000 ms)
  unsigned long ms_guard_time_bw;  // Before write min guard time (1000 ms)
  unsigned long ms_poll_time;      // How often to check SHM (1000 ms)
} cinfo;

/* SHM packet header (changed for each packet) */
typedef struct _pinfo {
  unsigned long data_length;       // Packet length
  unsigned long transaction_ID;    // Packet transaction ID
} pinfo;

/* SHM data */
typedef struct _pdata {
  unsigned long data[PKT_G1_ADU_SIZE_MAX];
} pdata;

/* SHM channel (one per TAG) */
typedef struct shm_channel {
  cha_info  cinfo;
  pkt_info  pinfo[SHM_CHANNEL_PKT_COUNT];
  pkt_data  pdata[SHM_CHANNEL_PKT_COUNT];
  int       next_pkt_index;                 // Index to next packet to be written
} shm_channel __attribute__ ((aligned (1024)));		/*  byte alignment */

#endif /* _SHM_H_ */
