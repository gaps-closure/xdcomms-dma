#ifndef VCHAN_HEADER_FILE
#define VCHAN_HEADER_FILE

#include <stdio.h>      // size_t
#include <stdint.h>     // uint8_t
#include <arpa/inet.h>  // ntohl
#include <string.h>     // strcmp...
#include <pthread.h>

#include "dma-proxy.h"
#include "shm.h"
#include "file_info.h"

#define STR_SIZE          64
#define MAX_PKTS_PER_CHAN 32   // Max pkts in Device for one (SHM) / all (DMA) chans

// RX thread arguments when starting thread
typedef struct _thread_args {
  void           *cp;               // Virtual Channel for RX thread is looking for
  int             buffer_id_start;  // Device buffer index
} thread_args;

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
  uint32_t         ctag;      // Received tag used for DMA (1 RX thread) not for SHM (1 RX thread/tag)
  size_t           data_len;  // length of data
  uint8_t         *data;      // data buffer
  int              tid;       // transaction ID
} pkt_info;

// Virutal Channel configuration (with device abstraction)
typedef struct virtual_channel {
  uint32_t         ctag;                   // Compressed tag (unique index) - used to search for channel
  char             dir;                    // Receive (from network) or Transmit (to network): 'r' or 't'
  char             dev_type[8];            // device type: e.g., shm (ESCAPE) or dma (MIND)
  char             dev_name[STR_SIZE];     // Device name: e.g., /dev/mem, /dev/sue_dominous or /tmp/xdc
  int              fd;                     // Device file descriptor (set when device openned)
  int              retries;                // Number of RX polls (every RX_POLL_INTERVAL_NSEC) pre timeout
  time_t           unix_seconds;           // When process was started
  unsigned long    wait4new_client;        // 1 = RX server checks if client started later (0 = no check)
  // RX Virtual Packet buffers between thread (reading from device) and nonblock_recv (sending to user)
  pkt_info         rvpb[MAX_PKTS_PER_CHAN];  // RX packet info from RX thread (needs mutex lock)
  int              rvpb_count;             // Number of Receive Virtual Packet buffers: e.g., SHM=2, DMAt=1, DMAr=16
  int              rvpb_index_thrd;        // Thread's cyrrent Receive Virtual Packet buffer index
  int              rvpb_index_recv;        // Nonblock_recv cyrrent Receive Virtual Packet buffer index
  // RX THREAD(S)
  pthread_mutex_t  lock;                   // Ensure RX thread does not write while xdcomms reads
  thread_args      thd_args;               // arguements passed rx thread for this tag
  pthread_t        thread_id;              // thread id for the rx thread
  // INFO FOR SPECIFIC CHANNELS
  file_channel    *file_info;              // Ptr to file info
  shm_channel     *shm_addr;               // Ptr to mmap'ed SHM struct: virtual addr + offset
  memmap           mm;                     // Mmap configuration
} vchan;

typedef struct _virtual_chan_list {
  vchan  *cp;
  char   dev_name[STR_SIZE];
  int    count;
} vchan_list;

void vchan_print(vchan *cp, char *enclave_name);

#endif  // VCHAN_HEADER_FILE
