#ifndef VCHAN_HEADER_FILE
#define VCHAN_HEADER_FILE

#include <stdio.h>      // size_t
#include <stdint.h>     // uint8_t
#include <arpa/inet.h>  // ntohl
#include "dma-proxy.h"
#include "shm.h"

#define STR_SIZE          64
#define MAX_PKTS_PER_CHAN 32   // Max pkts in Device for one (SHM) / all (DMA) chans

// Fixed mmap configuration (channel_buffer in DMA device, shm_channel in SHM device)
typedef struct _memmap {
  int              prot;      // Mmap protection field (e.g., Read and/or write)
  int              flags;     // mmap'ed flags (e.g., SHARED)
  unsigned long    phys_addr; // mmap'ed physical address
  unsigned long    len;       // mmap'ed memory length
  void            *virt_addr; // Mmaped virtual address of device buffer struct (for 1st SHM and all DMA channels)
//  unsigned long    offset;    // Offset from mmap_virt_addr to channel info
} memmap;

// Dynamic (per packet) Received Packet information
typedef struct _pkt_info {
  char             newd;      // RX thread sets on new packet, xdcomms resets after reading
//  gaps_tag  tag;       // Received tag
  size_t           data_len;  // length of data
  uint8_t         *data;      // data buffer
  int              tid;       // transaction ID
} pkt_info;

// Virutal Channel configuration (with device abstraction)
typedef struct channel {
  uint32_t         ctag;                   // Compressed tag (unique index) - used to search for channel
  char             dir;                    // Receive (from network) or Transmit (to network): 'r' or 't'
  char             dev_type[4];            // device type: e.g., shm (ESCAPE) or dma (MIND)
  char             dev_name[STR_SIZE];     // Device name: e.g., /dev/mem or /dev/sue_dominous
  int              fd;                     // Device file descriptor (set when device openned)
  int              retries;                // Number of RX polls (every RX_POLL_INTERVAL_NSEC) pre timeout
  time_t           unix_seconds;           // When process was started
  unsigned long    wait4new_client;        // 1 = RX server checks if client started later (0 = no check)
  pthread_mutex_t  lock;                   // Ensure RX thread does not write while xdcomms reads
  memmap           mm;                     // Mmap configuration
  int              pkt_buf_index;          // Buf index between Rx Thread nad RX virtual channel
  int              pkt_buf_count;          // Number of packets in a channel: e.g., SHM=2, DMAt=1, DMAr=16
  pkt_info         rx[MAX_PKTS_PER_CHAN];  // RX packet info from RX thread (needs mutex lock)
  shm_channel     *shm_addr;               // Ptr to mmap'ed SHM struct: virtual addr + offset
} chan;

void chan_print(chan *cp, char *enclave_name);

#endif  // VCHAN_HEADER_FILE
