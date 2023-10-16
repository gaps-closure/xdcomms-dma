/*
 * Cross Domain Communication (xdcomms) Library among CLOSURE
 * partitioned applications using GAP's Cross Domain Guards, It
 * maintains the same API as the Hardware Abstraction Layer (HAL).
 *
 *
 * v0.5 October 2023 Added support for file-based communication,
 * including interfacing with an X-ARBITOR gateway.
 *
 * v0.4 August 2023: Virtual Hardware device layer defines abstract
 * channels added to support multiple communication technologies,
 * insread of only DMA channels (see v0.3), so renamed from
 * 'xdcomms-dma.c' to 'xdcomms.c'. In addition to DMA, it now supports
 * Shared Memoru (SHM) channels, by copying data to specified regions
 * of mmap'ed host memory. The mmap'ed addresses can be in the host's
 * own memory or regions mapp'ed from the Intel ESCAPE FPGA board.
 * One-way channel definitions (including Tags) are now directly read
 * from the CLOSURE generated JSON configuration file (xdcomms.ini).
 * Selection of device configuration is done through new Environment
 * variables specified when running the partitioned application.
 *
 * v0.3 October 2022: Supprted direct transfers between Applications
 * and the MIND-DMA CDG via the MIND proxy DMA driver using IOCTL
 * commands. The driver uses kernel space DMA control to the XILINX AXI
 * DMA/MCDMA driver on the GE MIND ZCU102 FPGA board. For testing, it
 * can also communicate without a XDG using a Pseudo driver emulation
 */

#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>
#include <string.h>
#include <sched.h>
#include <errno.h>
#include <sys/param.h>
#include <assert.h>

#include <stdlib.h>
#include <unistd.h>
#include <assert.h>

#include <sys/inotify.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "xdcomms.h"
#include "vchan.h"
#include "../tiny-json/tiny-json.h"


#define DATA_TYP_MAX        50
#define JSON_OBJECT_SIZE    10000
#define PRINT_STATE_LEVEL   2                 // Reduce level to help debug (min=0)
//#define OPEN_WITH_NO_O_SYNC                   // Replaces slow open-O_SYNC with msync DOES NOT WORK
#define PRINT_US_TRACE                        // print Performance traces when defined

codec_map       xdc_cmap[DATA_TYP_MAX];       // maps data type to its data encode + decode functions
char            enclave_name[STR_SIZE] = "";  // enclave name (e.g., green)
vchan           vchan_info[GAPS_TAG_MAX];     // buffer array to store local virtual channel info per tag
pthread_mutex_t vchan_create;

void rcvr_thread_start(vchan *cp);
vchan *get_cp_from_ctag(uint32_t ctag, char dir, int json_index);

/**********************************************************************/
/* A) Tag Compression / Decompression                                    */
/**********************************************************************/
/* Compress teg (tag -> ctag) from a 3 member stuct to a uint32_t*/
void ctag_encode(uint32_t *ctag, gaps_tag *tag) {
  uint32_t ctag_h;
  ctag_h = ((CTAG_MOD * (
                         ( CTAG_MOD * ((tag->mux) % CTAG_MOD)) +
                                  ((tag->sec)  % CTAG_MOD))
             ) + ((tag->typ) % CTAG_MOD));
  *ctag = htonl(ctag_h);
}

/* Decompress teg (ctag -> tag) */
void ctag_decode(gaps_tag *tag, uint32_t *ctag) {
  uint32_t ctag_h = ntohl(*ctag);
  tag->mux = (ctag_h & 0xff0000) >> 16 ;
  tag->sec = (ctag_h &   0xff00) >> 8;
  tag->typ = (ctag_h &     0xff);
}

/**********************************************************************/
/* B) Read/write BW Packet                                            */
/**********************************************************************/
void bw_print(bw *p) {
  fprintf(stderr, "%s: ", __func__);
  fprintf(stderr, "ctag=%u ", ntohl(p->message_tag_ID));
  fprintf(stderr, "crc=%02x ", ntohs(p->crc16));
  log_buf_trace("Data", (uint8_t *) p->data, ntohs(p->data_len));
  fprintf(stderr, "\n");
}

uint16_t bw_crc_calc(bw *pkt) {
  return (crc16((uint8_t *) pkt, sizeof(pkt->message_tag_ID) + sizeof (pkt->data_len)));
}

/* Get size of packet (= header length + data length) */
int bw_get_packet_length(bw *pkt, size_t data_len) {
  return (sizeof(pkt->message_tag_ID) + sizeof(pkt->data_len) + sizeof(pkt->crc16) + data_len);
}

/* Convert tag to local host format */
void bw_len_encode (uint16_t *out, size_t len) {
  *out = ntohs((uint16_t) len);
}

/* Convert tag to local host format */
void bw_len_decode (size_t *out, uint16_t len) {
  *out = ntohs(len);
}

/* b) Create CLOSURE packet header */
void bw_gaps_header_encode(bw *p, size_t *p_len, uint8_t *buff_in, size_t *buff_len, gaps_tag *tag) {
  ctag_encode(&(p->message_tag_ID), tag);
  bw_len_encode(&(p->data_len), *buff_len);
  p->crc16 = htons(bw_crc_calc(p));
  *p_len = bw_get_packet_length(p, *buff_len);
}

// Copy packet from device buffer to virtual channel buffer (idx=vb_index)
void bw_write_into_vpb(vchan *cp, bw *p) {
  int vb_index = cp->rvpb_index_thrd;
  pthread_mutex_lock(&(cp->lock));
  bw_len_decode(&(cp->rvpb[vb_index].data_len), p->data_len);
  cp->rvpb[vb_index].data = (uint8_t *) p->data;
  cp->rvpb[vb_index].ctag = ntohl(p->message_tag_ID);
  cp->rvpb[vb_index].newd = 1;
  log_trace("THREAD-4 Copy rx packet (ctag=%08x len=%d) into rx Virtual Buffer (vb_index=%d)", cp->rvpb[vb_index].ctag, cp->rvpb[vb_index].data_len, vb_index);
  cp->rvpb_index_thrd = (vb_index + 1) % cp->rvpb_count;     // Increement vp-buffer index
  pthread_mutex_unlock(&(cp->lock));
}

void bw_process_rx_packet_if_good(bw *p) {
  vchan *cp = get_cp_from_ctag(p->message_tag_ID, 'r', -1);      // -1 = find context only if flow is already setup
  if ((cp == NULL) || ((p->data_len) < 1)) log_trace("Thread-4x Ignore bad rx packet: len=%d ctag=%08x cp=%p", p->data_len, ntohl(p->message_tag_ID), cp);
  else bw_write_into_vpb(cp, p);
}

/**********************************************************************/
/* D1) DMA device: Open/Configure                                     */
/**********************************************************************/
/* Open DMA channel. USes and fills-in cp (with fd, mmap-va, mmap-len) */
void dma_open_channel(vchan *cp) {
  // a) Get buffer count
  int buffer_count = DMA_PKT_COUNT_TX;
  if ((cp->dir) == 'r') buffer_count = DMA_PKT_COUNT_RX;
  // b) Open device
  if ((cp->fd = open(cp->dev_name, O_RDWR)) < 1) FATAL;
  // c) mmpp device
  cp->mm.len = sizeof(struct channel_buffer) * buffer_count;
  cp->mm.virt_addr = mmap(NULL, cp->mm.len, cp->mm.prot, cp->mm.flags, cp->fd, cp->mm.phys_addr);
  if (cp->mm.virt_addr == MAP_FAILED) FATAL;
  log_debug("Opened and mmap'ed DMA channel %s: addr=(v=0x%lx p=0x%lx) len=0x%x fd=%d", cp->dev_name, cp->mm.virt_addr, cp->mm.phys_addr, cp->mm.len, cp->fd);
  log_debug("Chan_Buff=0x%lx Bytes Channels/dev={Tx=%d Rx=%d} Max Bytes/PKT=0x%lx (Packets/channel=%d)", BUFFER_SIZE, DMA_PKT_COUNT_TX, DMA_PKT_COUNT_RX, sizeof(bw), BUFFER_SIZE / sizeof(bw));
}

/**********************************************************************/
/* S2) DMA device: Read/write                                         */
/**********************************************************************/
/* Use DMA ioctl operations to tx or rx data */
int dma_start_to_finish(int fd, int *buffer_id_ptr, struct channel_buffer *cbuf_ptr) {
//  log_trace("START_XFER (fd=%d, id=%d buf_ptr=%p unset-status=%d)", fd, *buffer_id_ptr, cbuf_ptr, cbuf_ptr->status);
//  time_trace("DMA Proxy transfer 1 (fd=%d, id=%d)", fd, *buffer_id_ptr);
  ioctl(fd, START_XFER,  buffer_id_ptr);
//  log_trace("DMA Proxy Started (fd=%d, id=%d) len=0x%lx", fd, *buffer_id_ptr, cbuf_ptr->length);
  ioctl(fd, FINISH_XFER, buffer_id_ptr);
//  time_trace("DMA Proxy transfer 3 (fd=%d, id=%d): status=%d", fd, *buffer_id_ptr, cbuf_ptr->status);
  return (cbuf_ptr->status);
}

void dma_send(vchan *cp, void *adu, gaps_tag *tag) {
  struct channel_buffer  *dma_tx_chan = (struct channel_buffer *) cp->mm.virt_addr;
  bw        *p;               // Packet pointer
  int       rv, buffer_id=0;  // Use only a single buffer
  size_t    adu_len;          // The CLOSURE generated encoder calculates length */
  size_t    packet_len;
  
  p = (bw *) &(dma_tx_chan->buffer);      // DMA packet buffer pointer, where we put the packet */
#ifdef PRINT_US_TRACE
  time_trace("XDC_Tx1 ready to encode for ctag=%08x into %p", ntohl(cp->ctag), p);
#endif
  cmap_encode(p->data, adu, &adu_len, tag, xdc_cmap);         // Put packet data into DMA buffer
  bw_gaps_header_encode(p, &packet_len, adu, &adu_len, tag);  // Put packet header into DMA buffer
  dma_tx_chan->length = packet_len;                           // Tell DMA buffer packet length (data + header)
  log_trace("Send packet on ctag=%08x fd=%d buf_id=%d of len: adu=%d packet=%d Bytes", ntohl(cp->ctag), cp->fd, buffer_id, ntohs(p->data_len), packet_len);
  if (packet_len <= sizeof(bw)) log_buf_trace("TX_PKT", (uint8_t *) &(dma_tx_chan->buffer), packet_len);
  rv = dma_start_to_finish(cp->fd, &buffer_id, dma_tx_chan);
#ifdef PRINT_US_TRACE
  time_trace("XDC_Tx2 DMA Sent ctag=%08x len=%ld rv=%d", ntohl(cp->ctag), packet_len, rv);
#endif
  log_debug("XDCOMMS tx packet tag=<%d,%d,%d> len=%ld rv=%d", tag->mux, tag->sec, tag->typ, packet_len, rv);
}
  
// Check received packet in DMA channel buffer (index = dma_cb_index)
void dma_check_rx_packet(vchan *cp, struct channel_buffer *dma_cb_ptr, int dma_cb_index) {
  bw *p = (bw *) &(dma_cb_ptr[dma_cb_index].buffer);    // NB: DMA buffer must be larger than bW
  log_debug("THREAD-3 rx DMA packet: ctag=0x%08x data-len=%d index=%d status=%d rv=0", ntohl(p->message_tag_ID), ntohs(p->data_len), dma_cb_index, dma_cb_ptr[dma_cb_index].status);
  bw_process_rx_packet_if_good(p);
}

// Wait for data from DMA channel (index = dma_cb_index)
// Inc index for each data returned: good (PROXY_NO_ERROR) or bad (PROXY_ERROR or PROXY_BUSY)
void dma_rcvr(vchan *cp) {
  static int             dma_cb_index=0;      // DMA channel buffer index
  int                    rv;
  struct channel_buffer *dma_cb_ptr = (struct channel_buffer *) cp->mm.virt_addr;  /* start of channel buffer */
  struct channel_buffer *cbuf_ptr   = &(dma_cb_ptr[dma_cb_index]);

  // A) Wait for Packet from DMA
  cbuf_ptr->length = sizeof(bw);            /* Receive up to make length Max size */
  log_debug("THREAD-2 waiting for any tag on %s device %s: fd=%d max_len=%d cb_index=%d", cp->dev_type, cp->dev_name, cp->fd, cbuf_ptr->length, dma_cb_index);
#ifdef PRINT_US_TRACE
  time_trace("XDC_Rx1 DMA Waiting on %d cb_index=%d", cp->dev_name, dma_cb_index);
#endif
  while ((rv = dma_start_to_finish(cp->fd, &dma_cb_index, cbuf_ptr)) == PROXY_TIMEOUT) { ; }
#ifdef PRINT_US_TRACE
  time_trace("XDC_Rx2 DMA returned rv=%d, status=%d len=0x%lx", rv, cbuf_ptr->status, cbuf_ptr->length);
#endif
  log_trace("DMA rx returned rv=%d, status=%d (NO_ERR=0, BUSY=1, TIMEOUT=2, ERR=3) id=%d len=0x%lx) ", rv, cbuf_ptr->status, dma_cb_index, cbuf_ptr->length);
  if (rv == PROXY_NO_ERROR) dma_check_rx_packet(cp, dma_cb_ptr, dma_cb_index);
  dma_cb_index = (dma_cb_index + 1) % DMA_PKT_COUNT_RX;
}

/**********************************************************************/
/* E1) SHM device: Open/Configure                                     */
/**********************************************************************/
// Open SHM channel given Physical address and length. Returns fd, mmap-va and mmap-len
void shm_open_channel(vchan *cp) {
  void          *pa_virt_addr;
  unsigned long  pa_phys_addr, pa_mmap_len;       /* page aligned physical address (offset) */
  int            shm_open_flags=O_RDWR;
  
  // a) Open device (Default is to SYNC per write (much slower, but no manual map sync required)
#ifndef OPEN_WITH_NO_O_SYNC   // double-negative means open with SYNC which is the default
  shm_open_flags |= O_SYNC;
//  shm_open_flags |= O_DSYNC;     // O_SYNC, but does (or does not?) care if file metadata is written to disk
//  shm_open_flags |= O_DIRECT;      // Works the same as O_SYNC
#endif
  if ((cp->fd = open(cp->dev_name, shm_open_flags)) == -1) FATAL;

  // b) mmpp device: reduce address to be a multiple of page size and add the diff to length
  pa_phys_addr       = cp->mm.phys_addr & ~MMAP_PAGE_MASK;
  pa_mmap_len        = cp->mm.len + cp->mm.phys_addr - pa_phys_addr;
//  log_trace("SHM len = 0x%x = 0x%x", cp->mm.len, pa_mmap_len);
  pa_virt_addr       = mmap(0, pa_mmap_len, cp->mm.prot, cp->mm.flags, cp->fd, pa_phys_addr);
  if (pa_virt_addr == (void *) MAP_FAILED) FATAL;   // MAP_FAILED = -1
  cp->mm.virt_addr   = pa_virt_addr + cp->mm.phys_addr - pa_phys_addr;   // add offset to page aligned addr
  log_debug("Opened + mmap'ed SHM channel %s: fd=%d, v_addr=0x%lx p_addr=0x%lx len=0x%x: Open flags=%x (RDWR=%x, SYNC=%x) Mmap flags0x%x (SHARED=%x)", cp->dev_name, cp->fd, cp->mm.virt_addr, pa_phys_addr, pa_mmap_len, shm_open_flags, O_RDWR, O_SYNC, cp->mm.flags, MAP_SHARED);
}

void shm_info_print(shm_channel *cip) {
  int            i;
  unsigned long  len_bytes;
  
  fprintf(stderr, "  shm info %08x (%p): last=%d next=%d (max=%d ga=%ld gb=%ld ut=0x%lx crc=0x%04x)\n", ntohl(cip->cinfo.ctag), cip, cip->pkt_index_last, cip->pkt_index_next, cip->cinfo.pkt_index_max, cip->cinfo.ms_guard_time_aw, cip->cinfo.ms_guard_time_bw, cip->cinfo.unix_seconds, cip->crc16);
  for (i=0; i<SHM_PKT_COUNT; i++) {
    len_bytes = cip->pinfo[i].data_length;
    fprintf(stderr, "  %d (%p) tid=0x%lx len=%ld: ", i, cip->pdata[i].data, cip->pinfo[i].transaction_ID, len_bytes);
    buf_print_hex(cip->pdata[i].data, len_bytes);
  }
}

uint16_t get_crc(vchan *cp) {
  return(crc16((uint8_t *) &(cp->shm_addr->cinfo), sizeof(cinfo)));
}

// After openning SHM device, initialize SHM configuration
void shm_init_config_one(vchan *cp) {
  int           i;
  shm_channel  *shm_ptr = cp->shm_addr;
  cinfo        *cip = &(shm_ptr->cinfo);

  log_trace("%s:  cp=%p va=%p va_offset=%lx", __func__, cp, cp->mm.virt_addr, shm_ptr);
//  log_trace("shm_channel size t=%lx c=%lx i=%lx d=%lx = %lx %lx", sizeof(time_t), sizeof(cinfo), sizeof(pinfo), sizeof(pdata), sizeof(shm_channel), SHM_PKT_COUNT*(sizeof(pinfo) + sizeof(pdata)));
  log_trace("Up to %ld Channels (mmap=0x%lx / Chan=0x%lx) PKTS/Chan=%d Bytes/PKT=0x%lx", (cp->mm.len) / sizeof(shm_channel), cp->mm.len, sizeof(shm_channel), SHM_PKT_COUNT, sizeof(pdata));
  shm_ptr->pkt_index_next = 0;
  // CHeck that it is possible to write into the SHM structure
  if (shm_ptr->pkt_index_next !=0) {log_fatal("SHM write fails for ctag=0x%08x", cp->ctag); FATAL;}
  shm_ptr->pkt_index_last = -1;

  cip->ctag               = cp->ctag;
  cip->pkt_index_max      = SHM_PKT_COUNT;
  cip->ms_guard_time_aw   = DEFAULT_NS_GUARD_TIME_AW;
  cip->ms_guard_time_bw   = DEFAULT_NS_GUARD_TIME_BW;
  cip->unix_seconds       = time(NULL);
  shm_ptr->crc16          = get_crc(cp);
//  log_trace("%s %08x %c Pnters: va=%p vc=%p ci=%p vd=%p vn=%p", __func__, ntohl(cp->ctag), cp->dir, shm_ptr, cip, &(shm_ptr->pinfo), &(shm_ptr->pdata), &(shm_ptr->pkt_index_next));
  
  for (i=0; i<SHM_PKT_COUNT; i++) {
    shm_ptr->pinfo[i].data_length    = 0;
    shm_ptr->pinfo[i].transaction_ID = 0;
  }
#if 1 >= PRINT_STATE_LEVEL
  shm_info_print(shm_ptr);
#endif  // PRINT_STATE
}

/**********************************************************************/
/* E2) SHM device; Copy into/from                                     */
/**********************************************************************/
// Dumb memory copy using 8-byte words
//void naive_memcpy(unsigned long *d, const unsigned long *s, unsigned long len_in_words) {
//  for (int i = 0; i < len_in_words; i++) *d++ = *s++;
//}

#ifdef OPEN_WITH_NO_O_SYNC
// int msync(void addr, size_t length, int flags);
void shm_sync(void *addr, unsigned long len, int flags) {
  int rv = msync(addr, len, flags);
  log_info("%s rv = %d addr=%p len=%ld flags=%x", __func__, rv, addr, len, flags);
  if (rv < 0) {
     perror("msync failed");
     exit(0);
  }
}

// void __builtin___clear_cache(void *begin, void *end);
// void shm_sync2(void *addr) {
//   __builtin___clear_cache(addr, addr + sizeof(shm_channel));
// }

// int cacheflush(char *addr, int nbytes, int cache);
// #include <asm/cachectl.h>   // cacheflush
// needs sudo ln -s /usr/src/linux-hwe-5.15-headers-5.15.0-76/arch/arc/include/uapi/asm/ /usr/include/asm
// void shm_sync3(void *addr) {
//   int rv = cacheflush((char *) addr, sizeof(shm_channel), DCACHE);
//  log_trace("%s rv = %d", __func__, rv);
//  if (rv < 0) {
//    perror("msync failed");
//    exit(0);
//  }
// }

//  dmac_map_area( cp->shm_addr->pdata[write_index].data, adu_len, DMA_TO_DEVICE);
#endif

// Modify last_ptr
void delete_oldest_pkt(int *last_ptr, int write_index) {
  struct timespec ts;                 // Guard time

  if (*last_ptr == write_index) {
    ts.tv_sec  = 0;
    ts.tv_nsec = DEFAULT_NS_GUARD_TIME_BW;
    log_trace("%s: guard time = %d ns)", __func__, DEFAULT_NS_GUARD_TIME_BW);
    *last_ptr = ((*last_ptr) + 1) % FILE_COUNT;
    nanosleep(&ts, NULL);       // Give time so other enclave can finish reading before writing into last
  }
  if (*last_ptr < 0) *last_ptr = 0;   // If the first written packet
}

void shm_send(vchan *cp, void *adu, gaps_tag *tag) {
  int            *last_ptr    = &(cp->shm_addr->pkt_index_last);
  int            *next_ptr    = &(cp->shm_addr->pkt_index_next);
  int             write_index = *next_ptr;
  size_t          adu_len     = 0;    // encoder calculates length */
  
  log_trace("%s TX index: next=%d last=%d (guard_bw=%d)", __func__, *next_ptr, *last_ptr, DEFAULT_NS_GUARD_TIME_BW);
  delete_oldest_pkt(last_ptr, write_index);
  // B) Encode Data into SHM
#ifdef PRINT_US_TRACE
  time_trace("TX1 %08x (index=%d)", ntohl(cp->ctag), write_index);
#endif
  cmap_encode(cp->shm_addr->pdata[write_index].data, adu, &adu_len, tag, xdc_cmap);
//  XXX TODO: incoprate naive_memcpy into cmap_encode/decode
//  naive_memcpy(cp->shm_addr->pdata[pkt_index_nxt].data, adu, adu_len);  // TX adds new data
  cp->shm_addr->pinfo[write_index].data_length = adu_len;
  *next_ptr = (write_index + 1) % SHM_PKT_COUNT;
#if 1 >= PRINT_STATE_LEVEL
  shm_info_print(cp->shm_addr);
#endif  // PRINT_STATE

  // C) Sync data (if not open /dev/mem with 'slow' O_SYNC)
#ifdef OPEN_WITH_NO_O_SYNC
//  shm_sync((void *) (cp->shm_addr), sizeof(shm_channel), MS_ASYNC);
  shm_sync(cp->mm.virt_addr, cp->mm.len, MS_ASYNC);
#endif
#ifdef PRINT_US_TRACE
  time_trace("TX2 %08x (len=%d)", ntohl(cp->ctag), adu_len);
#endif
}

// Check that SHM TX has matches RX tag and crc. Also, (optionally) has newer timestamp
int wait_if_old(vchan *cp) {
  uint32_t rx_tag = cp->ctag;
  uint16_t rx_crc = get_crc(cp);
  time_t   rx_tim = cp->unix_seconds;
  
  uint32_t tx_tag = cp->shm_addr->cinfo.ctag;
  uint16_t tx_crc = cp->shm_addr->crc16;
  time_t   tx_tim = cp->shm_addr->cinfo.unix_seconds;
  
//  log_trace("tags: SHM=0x%08x Loc=0x%08x", rx_tag, tx_tag);
  if (tx_tag != rx_tag)  return(-1);  // wait for good data
//  log_trace("CRCs: SHM=0x%04x Loc=0x%04x", tx_crc, rx_crc);
  if (tx_crc != rx_crc ) return(-1);  // wait for good data
//  log_trace("time: SHM=0x%04x Loc=0x%04x (nnew=%d)", tx_tim, rx_tim, cp->wait4new_client);
  if      ((cp->wait4new_client) == 0) log_trace("Not wait for new client");
  else if (rx_tim <= tx_tim) log_trace("New client tags: TX=0x%08x RX=0x%08x", rx_tag, tx_tag);
  else return(-1);                    // wait for new client
  return(0);
}

// Copy from SHM  buffer (index = vb_index) to virtual channel buffer (index = vb_index)
void shm_rcvr(vchan *cp) {
  static int once = 1;
  int vb_index = cp->rvpb_index_thrd;

  if (once == 1) {
    while (wait_if_old(cp)) { ; }
    once = 0;
  }
  log_trace("THREAD-2 waiting for tag=0x%08x on %s device %s: index=%d of %d (last=%d)", ntohl(cp->ctag), cp->dev_type, cp->dev_name, vb_index, cp->rvpb_count, cp->shm_addr->pkt_index_next);
  while (vb_index == (cp->shm_addr->pkt_index_next)) { ; }
  pthread_mutex_lock(&(cp->lock));
  log_trace("THREAD-3 rx SHM packet: ctag=0x%08x len=%d id=%d", ntohl(cp->ctag), cp->shm_addr->pinfo[vb_index].data_length, vb_index);
#if 1 >= PRINT_STATE_LEVEL
  shm_info_print(cp->shm_addr);
#endif  // PRINT_STATE
  cp->rvpb[vb_index].data_len = cp->shm_addr->pinfo[vb_index].data_length;
  cp->rvpb[vb_index].data     = (uint8_t *) (cp->shm_addr->pdata[vb_index].data);
  cp->rvpb[vb_index].newd     = 1;
//  fprintf(stderr, "PPP %p = %08x ", cp->shm_addr->pdata[vb_index].data, ntohl(cp->shm_addr->pdata[vb_index].data[4]));
  cp->rvpb_index_thrd = (vb_index + 1) % cp->rvpb_count;     // Increement vp-buffer index
  log_trace("THREAD-4 Copy rx packet (ctag=%08x len=%d) into rx Virtual Buffer (vb_index=%d)", ntohl(cp->ctag), cp->rvpb[vb_index].data_len, vb_index);
  pthread_mutex_unlock(&(cp->lock));
}

/**********************************************************************/
/* F1) FILE XDC: Open/Create                                          */
/**********************************************************************/
// Create directory for XDC files
void create_empty_file_dir(vchan *cp) {
  char cmd[200] = "mkdir -p -m 0777 ";  // add directory with recursive flag (so cannot use mkdir)
  strcat(cmd, cp->dev_name);    // Diractory path
//  strcat(cmd, " 0666");         // permissions
  system(cmd);
  if ((cp->dir) == 't') {       // Transmitter clears directory
    strcpy(cmd, "rm -rf ");
    strcat(cmd, cp->dev_name);
    strcat(cmd, "/*");
    system(cmd);
  }
}

// Open SHM channel given Physical address and length. Returns fd, mmap-va and mmap-len
void file_open_channel(vchan *cp) {
  int  rv;

  cp->file_info = (file_channel *) malloc(sizeof(file_channel));
  cp->file_info->pkt_index_next = 0;
  cp->file_info->pkt_index_last = -1;
//  log_trace("%s cp=%p n=%s", __func__, cp, cp->dev_name);
//  vchan_print(cp, enclave_name);
    
  // Create INOTIFY instance and add directory into watch list
  create_empty_file_dir(cp);
  cp->fd = inotify_init();
  if ( cp->fd < 0 ) perror( "inotify_init" );
  rv = inotify_add_watch(cp->fd, cp->dev_name, IN_CLOSE_WRITE);
  if ( rv < 0 ) perror( "inotify_add_watch" );
  
  // XXX: Should save rv in cp, so can close properly
//  inotify_rm_watch( cp->fd, cp->rv );
//  close( cp->fd );
//  log_trace("Opened file-based channel %s: fd=%d", cp->dev_name, cp->fd);
}

/**********************************************************************/
/* F2) FILE XDC: Read/Write                                           */
/**********************************************************************/
// File exists and executable
bool file_exists(const char *filename) {
  struct stat buffer;
  return (stat(filename, &buffer) == 0 && buffer.st_mode & S_IXUSR) ? true : false;
}

void file_run_send_script(vchan *cp, const char *filename) {
  char cmd[500] = XARBITOR_SEND_SCRIPT_FILENAME;
  strcat(cmd, " -h ");
  strcat(cmd, cp->file_info->xarb_IP);
  strcat(cmd, " -g ");
  strcat(cmd, cp->file_info->xarb_port);
  strcat(cmd, XARBITOR_SEND_SCRIPT_FIXED_ARGS);
  strcat(cmd, " -d ");
  strcat(cmd, filename);
//  log_trace("Send Script = %s", cmd);
  system(cmd);
}

// Write packet into file
void file_write(vchan *cp, bw *p, size_t packet_len, int write_index) {
  char       filename[128];
  char       str[32];
  size_t     written_len;
  FILE      *fp;

  strcpy(filename, cp->dev_name);
  sprintf(str, "/%x_", htonl(cp->ctag));
  strcat(filename, str);
  sprintf(str, "%d", write_index);
  strcat(filename, str);
  strcat(filename, FILENAME_EXTENSION);
  
  log_trace("Writing packet (len=%d) into file: %s", packet_len, filename);
  fp = fopen(filename, "wb");
  if (fp == NULL) {
    log_fatal("fopen() failed\n");
    exit(-1);
  }
  written_len = fwrite (p, sizeof(char), packet_len, fp);
  if (written_len != packet_len) {
    log_fatal("fwrite() failed: wrote only %zu out of %zu packet bytes.\n",
               written_len, packet_len);
    exit(-1);
  }
  fclose(fp);

  if (file_exists(XARBITOR_SEND_SCRIPT_FILENAME))  file_run_send_script(cp, filename);
  else log_warn("XARBITOR_SEND_SCRIPT %s does not exist", XARBITOR_SEND_SCRIPT_FILENAME);
}

// Tx packet
void file_send(vchan *cp, void *adu, gaps_tag *tag) {
  int            *last_ptr    = &(cp->file_info->pkt_index_last);
  int            *next_ptr    = &(cp->file_info->pkt_index_next);
  int             write_index = *next_ptr;
  size_t          adu_len     = 0;    // encoder calculates length */
  bw             *p;                  // Packet pointer
  size_t          packet_len;
  
  log_trace("%s: ctag=%08x fd=%d (TX index next=%d last=%d)", __func__, ntohl(cp->ctag), cp->fd, *next_ptr, *last_ptr);
  delete_oldest_pkt(last_ptr, write_index);
  p = (bw *) &(cp->file_info->pkt_buffer);    // FILE packet buffer pointer, where we put packet */
  cmap_encode(p->data, adu, &adu_len, tag, xdc_cmap);         // Put packet data into DMA buffer
  bw_gaps_header_encode(p, &packet_len, adu, &adu_len, tag);  // Put packet header into DMA buffer
  log_trace("Send packet ctag=%08x len: adu=%d packet=%d Bytes", ntohl(cp->ctag), ntohs(p->data_len), packet_len);
  if (packet_len <= sizeof(bw)) log_buf_trace("TX_PKT", (uint8_t *) &(cp->file_info->pkt_buffer), packet_len);
  file_write(cp, p, packet_len, write_index);
  log_debug("XDCOMMS tx packet tag=<%d,%d,%d> len=%ld", tag->mux, tag->sec, tag->typ, packet_len);
  *next_ptr = (write_index + 1) % FILE_COUNT;
}

// Open file (if event file name is correct), else return NULL
FILE *file_event_get_matching_filename(char *filename, vchan *cp, struct inotify_event *event) {
  char *filename_extension_ptr;
  
  log_trace("New file = %s", event->name);
  filename_extension_ptr = strstr(event->name, FILENAME_EXTENSION);
  if (filename_extension_ptr != NULL) {
    log_trace("filename %s starts at %p extension at %p len=%ld ext-diff= %ld - %ld", event->name, event->name, filename_extension_ptr, strlen(event->name), strlen(filename_extension_ptr), strlen(FILENAME_EXTENSION));
    if ( strlen(filename_extension_ptr) == strlen(FILENAME_EXTENSION) ) {
      strcpy(filename, cp->dev_name);
      strcat(filename, "/");
      strcat(filename, event->name);
//      log_trace("Matching New file has Full filename=%s", filename);
      return (fopen(filename, "rb"));
    }
  }
  return (NULL);
}

// Process list of changed files in event list one by one
void process_file_event_list(vchan *cp, char *buffer, int length) {
  int                   i = 0;
  size_t                packet_len;
  FILE                 *efp;                 // event file pointer
  bw                   *p;                  // Packet pointer
  struct inotify_event *event;
  char                  filename[FILENAME_MAX_BYTES];
  
  while ( i < length ) {
    event = (struct inotify_event *) &buffer[i];
    if (event->len) {
      log_trace("Event: current_ptr=%d name_len=%d + fixed_len=%d = %d (list_len=%d)", i, event->len, EVENT_SIZE, (event->len), (event->len) + EVENT_SIZE, length);
      if (event->mask & IN_CLOSE_WRITE) {      //  Move between dir:  || (event->mask & IN_MOVED_TO) ) {
        efp = file_event_get_matching_filename(filename, cp, event);
        if (efp == NULL) log_debug("THREAD-3x rx file=%s: ignoring due to bad filename", event->name);
        else {                    // write FILE into packet buffer */
          p = (bw *) &(cp->file_info->pkt_buffer);
          packet_len = fread(p, sizeof(char), FILE_MAX_BYTES, efp);
          fclose(efp);
          log_debug("THREAD-3b rx file=%s: Processing packet len=%ld bytes ctag=0x%08x)", filename, packet_len, ntohl(p->message_tag_ID));
          bw_process_rx_packet_if_good(p);
        }
      }
    }
    i += EVENT_SIZE + event->len;
  }
}

void file_rcvr(vchan *cp) {
  int length;
  char buffer[EVENT_BUF_LEN];
  
  log_debug("THREAD-2 waiting for any %s in directory %s (using inotify)", cp->dev_type, cp->dev_name);
  length = read(cp->fd, buffer, EVENT_BUF_LEN);   // Blocking inotify read
  log_trace("THREAD-3a: New file(s) detected by inotify (len=%d)", length);
  if (length < 0) perror( "read" );
  process_file_event_list(cp, buffer, length);
}

/**********************************************************************/
/* O) Virtual Device: Open                                            */
/**********************************************************************/
// Open channel device (based on name and type) and return its channel structure
void open_device(vchan *cp) {
  log_trace("%s of type=%s name=%s", __func__, cp->dev_type, cp->dev_name);
  if      (strcmp(cp->dev_type, "dma")  == 0) dma_open_channel(cp);
  else if (strcmp(cp->dev_type, "shm")  == 0) shm_open_channel(cp);
  else if (strcmp(cp->dev_type, "file") == 0) file_open_channel(cp);
  else {log_warn("Unknown type=%s (name=%s)", cp->dev_type, cp->dev_name); FATAL;}
}

// If new device, then open it (and remember it in local list)
//   Assumes new tag
void dev_open_if_new(vchan *cp) {
  static vchan_list  clist[MAX_DEV_COUNT];
  static int        once=1;
  int               i;
  
  if (once==1) {
    // a) initialize list
    for (i=0; i<MAX_DEV_COUNT; i++) {
      clist[i].cp = NULL;
      strcpy(clist[i].dev_name, "");
      clist[i].count = 0;
    }
    once = 0;
  }
  
  for(i=0; i<MAX_DEV_COUNT; i++) {
    // b) See if device is not open
    if (clist[i].count == 0) {   // Not set, so put new device name into list
      strcpy(clist[i].dev_name, cp->dev_name);
      clist[i].cp = cp;
      (clist[i].count)++;
      open_device(cp);           // Open new device
      log_trace("%s: Opened new device %s dir=%c ctag=0x%08x i=%d", __func__, cp->dev_name, cp->dir, ntohl(cp->ctag), i);
      return;  // new device
    }
    // b2) See if device is already open
    if (strcmp(cp->dev_name, clist[i].cp->dev_name) == 0) {    // Already set, so open info
      cp->fd           = clist[i].cp->fd;
      cp->file_info    = clist[i].cp->file_info;
      cp->mm.virt_addr = clist[i].cp->mm.virt_addr;
      (clist[i].count)++;
      log_trace("%s: %s device shared %d times for i=%d", __func__, cp->dev_name, clist[i].count, i);
      return;
    }
  }
  log_trace("%s: Too many devices (MAX=%d) to create device %s", __func__, MAX_DEV_COUNT, cp->dev_name);
  FATAL;    // Only here if list cannot store all devices (> MAX_DEV_COUNT)
}

// Initialize channel information for all (GAPS_TAG_MAX) possible tags
//   Set retries from one of three possible timeout values (in msecs).
//   In order of precedence (highest first) they are the:
//    a) Input parameter specified in a xdc_sub_socket_non_blocking() call
//       (this is the only way to specify a different value for each flow).
//    b) Environment variable (TIMEOUT_MS) speciied when starting app
//    c) Default (RX_POLL_TIMEOUT_MSEC_DEFAULT) from xdcomms.h
void vchan_init_all_once(void) {
  static int once=1;
  int        i, index_buf, t_in_ms;
  char      *t_env = getenv("TIMEOUT_MS");

  if (once==1) {
    t_in_ms = (t_env == NULL) ? RX_POLL_TIMEOUT_MSEC_DEFAULT : atoi(t_env);
    if (pthread_mutex_init(&vchan_create, NULL) != 0)   FATAL;
    for(i=0; i < GAPS_TAG_MAX; i++) {
      vchan_info[i].ctag            = 0;
      vchan_info[i].mm.prot         = PROT_READ | PROT_WRITE;
      vchan_info[i].mm.flags        = MAP_SHARED;
      vchan_info[i].retries         = (t_in_ms * NSEC_IN_MSEC)/RX_POLL_INTERVAL_NSEC;
      vchan_info[i].unix_seconds    = time(NULL);
      vchan_info[i].wait4new_client = 0;    // Do not check time transmitter started
      for (index_buf=0; index_buf<MAX_PKTS_PER_CHAN; index_buf++) {
        vchan_info[i].rvpb[index_buf].newd = 0;
        vchan_info[i].rvpb[index_buf].data_len = 0;
        vchan_info[i].rvpb[index_buf].data = NULL;
      }
      if (pthread_mutex_init(&(vchan_info[i].lock), NULL) != 0)   FATAL;
    }
    once=0;
  }
}

// Get channel device type
void get_dev_type(char *dev_type, char *env_type, char *def_type) {
  (env_type == NULL) ? strcat(dev_type, def_type) : strcat(dev_type, env_type);
}

// Get channel device name (*dev_name) from enivronment or default (for that type)
void get_dev_name(char *dev_name, char *env_name, char *def_name_dma, char *def_name_shm, char *def_name_file, char *dev_type, vchan *cp) {
  char   str[2 + sizeof(uint32_t)];

  if      ((strcmp(dev_type, "dma")) == 0) {
    strcpy(dev_name, "/dev/");        // prefix device name
    (env_name == NULL) ? strcat(dev_name, def_name_dma) : strcat(dev_name, env_name);
  }
  else if ((strcmp(dev_type, "shm")) == 0) {
    strcpy(dev_name, "/dev/");        // prefix device name
    (env_name == NULL) ? strcat(dev_name, def_name_shm) : strcat(dev_name, env_name);
  }
  else if ((strcmp(dev_type, "file")) == 0) {
    (env_name == NULL) ? strcpy(dev_name, def_name_file) : strcpy(dev_name, env_name);
    if (FILE_DIR_SHARE == 0) {
      sprintf(str, "/%x", htonl(cp->ctag));
      strcat(dev_name, str);
    }
  }
  else {
    log_fatal("Unknown device type: %s", dev_type);
    FATAL;
  }
//  log_trace("dev=%s (len=%d)", dev_name, strlen(dev_name));
}

// Get channel device number (*val) from enivronment or default (for that type)
void get_dev_val(unsigned long *val, char *env_val, unsigned long def_val_dma, unsigned long def_val_shm, unsigned long def_val_file, char *dev_type) {
  if (env_val == NULL) {
    if       (strcmp(dev_type, "dma") == 0) *val = def_val_dma;
    else if  (strcmp(dev_type, "shm") == 0) *val = def_val_shm;
    else if  (strcmp(dev_type, "file") == 0) *val = def_val_file;
    else     FATAL;
  }
  else       *val = strtol(env_val, NULL, 16);
}

// Initialize configuration for a new tag based on environment variables
void init_new_chan_from_envi(vchan *cp, uint32_t ctag, char dir) {
  cp->ctag = ctag;
  cp->dir  = dir;

//  log_trace("%s: ctag=0x%08x dir=%c TX=%s RX=%s", __func__, ntohl(ctag), dir, getenv("DEV_NAME_TX"), getenv("DEV_NAME_RX"));
  if (dir == 't') { // TX
    get_dev_type(cp->dev_type,     getenv("DEV_TYPE_TX"), "dma");
    get_dev_name(cp->dev_name,     getenv("DEV_NAME_TX"), "dma_proxy_tx", "mem", FILE_DIR_PATH_DEFAULT, cp->dev_type, cp);
    get_dev_val (&(cp->mm.len),    getenv("DEV_MMAP_LE"), (sizeof(struct channel_buffer) * DMA_PKT_COUNT_TX), SHM_MMAP_LEN, 0, cp->dev_type);
  }
  else {            // RX
    get_dev_type(cp->dev_type,     getenv("DEV_TYPE_RX"), "dma");
    get_dev_name(cp->dev_name,     getenv("DEV_NAME_RX"), "dma_proxy_rx", "mem", FILE_DIR_PATH_DEFAULT, cp->dev_type, cp);
//    log_trace("XXX dev=%s (len=%d)", cp->dev_name, strlen(cp->dev_name));
    get_dev_val (&(cp->mm.len),    getenv("DEV_MMAP_LE"), (sizeof(struct channel_buffer) * DMA_PKT_COUNT_RX), SHM_MMAP_LEN, 0, cp->dev_type);   // XXX dma default is an over estimate
    get_dev_val (&(cp->wait4new_client), getenv("SHM_WAIT4NEW"), 0x0, 0x0, 0x0, cp->dev_type);
  }
  get_dev_val(&(cp->mm.phys_addr), getenv("DEV_MMAP_AD"), DMA_ADDR_HOST, SHM_MMAP_ADDR, 0, cp->dev_type);
}

// Configure new channel, using 'json_index' to locate SHM block (not needed for DMA, file)
void init_new_chan(vchan *cp, uint32_t ctag, char dir, int json_index) {
  static int once=1;

  init_new_chan_from_envi(cp, ctag, dir);   // 1) Configure new tag
  dev_open_if_new(cp);                   // 2) Open device (if not already open)
//      log_trace("%s: Using %s device %s for ctag=0x%08x dir=%c", __func__, cp->dev_type, cp->dev_name, ntohl(cp->ctag), cp->dir);
  cp->rvpb_index_recv = 0;
  cp->rvpb_index_thrd = 0;
  
  if ((strcmp(cp->dev_type, "shm")) == 0) {
    cp->shm_addr   = cp->mm.virt_addr + (json_index * sizeof(shm_channel));
    cp->rvpb_count = SHM_PKT_COUNT;
    if ((cp->dir) == 't') shm_init_config_one(cp);  // 3) Configure SHM structure for new channel
    if ((cp->dir) == 'r') rcvr_thread_start(cp);    // 4) Start rx thread for new receive tag
    log_trace("ctag=0x%08x: VA=%p PA=%p", ctag, cp->shm_addr, (cp->mm.phys_addr & ~MMAP_PAGE_MASK) + (json_index * sizeof(shm_channel)));
  }
  
  else if ((strcmp(cp->dev_type, "dma")) == 0) {
    if ((cp->dir) == 'r') {
      cp->rvpb_count = DMA_PKT_COUNT_RX;
      if (once==1) {
        rcvr_thread_start(cp);      // 4) Start rx thread only once
        once = 0;
      }
    }
    else {
      cp->rvpb_count = DMA_PKT_COUNT_TX;
    }
  }
  
  else if ((strcmp(cp->dev_type, "file")) == 0) {
    char *xarb_IP_env   = getenv("XARB_IP");
    char *xarb_port_env = getenv("XARB_PORT");
    char *xIP           = cp->file_info->xarb_IP;
    char *xPORT         = cp->file_info->xarb_port;
    (xarb_IP_env   == NULL) ? strcpy(xIP,   XARBITOR_IP_DEFAULT)   : strcpy(xIP,   xarb_IP_env);
    (xarb_port_env == NULL) ? strcpy(xPORT, XARBITOR_PORT_DEFAULT) : strcpy(xPORT, xarb_port_env);
    if ((cp->dir) == 'r') {
      cp->rvpb_count = DMA_PKT_COUNT_RX;
      if (FILE_DIR_SHARE == 0) rcvr_thread_start(cp);        // 4) Start rx thread for each new receive tag
      else {
        if (once==1) {
          rcvr_thread_start(cp);      // 4) Start rx thread only once
          once = 0;
        }
      }
    }
  }
  else {log_warn("Unknown type=%s (name=%s)", cp->dev_type, cp->dev_name); FATAL;}
#if 1 >= PRINT_STATE_LEVEL
  vchan_print(cp, enclave_name);
#endif  // LOG_LEVEL_MIN
}

// Search for channel with this ctag
//   If new ctag, then initialize (based on direction and json_index)
vchan *get_cp_from_ctag(uint32_t ctag, char dir, int json_index) {
  int       chan_index;
  vchan    *cp;
  
  /* a) Initilize all channels (after locking from other application threads) */
  pthread_mutex_lock(&vchan_create);
  vchan_init_all_once();
  /* b) Find info for this tag (and possibly initialize*/
  for(chan_index=0; chan_index < GAPS_TAG_MAX; chan_index++) {  // Break on finding tag or empty
    cp = &(vchan_info[chan_index]);
    if (cp->ctag == ctag) break;            // found existing slot for tag
    if (cp->ctag == 0) {                    // found empty slot (before tag)
      if (json_index < 0) cp = NULL;        // Cannot find existing tag in database
      else init_new_chan(cp, ctag, dir, json_index);   // config new channel info, using 'index' to locate SHM block
      break;
    }
  }
  /* c) Unlock and return chan_info pointer */
  if (chan_index >= GAPS_TAG_MAX) FATAL;
//  log_trace("%s chan_index=%d: ctag=0x%08x", __func__, chan_index, ctag);
  pthread_mutex_unlock(&vchan_create);
  return (cp);
}
  
// Return channel pointer to Rx packet buffer for specified tag
vchan *get_chan_info(gaps_tag *tag, char dir, int index) {
  uint32_t  ctag;
  
  ctag_encode(&ctag, tag);                   // Encoded ctag
  return (get_cp_from_ctag(ctag, dir, index));
}


/**********************************************************************/
/* P) Extract JSON Configuration Info (for all devices and TX/RX)     */
/**********************************************************************/
// Configure channel information
//   Ensure a) Both sides use the same index for the same tag
//          b) Group indexes for same direction
//   For example using websrv app:
//      orange j=0 r=0: tag=<1,1,1>   green j=0 t=0: tag=<1,1,1>
//      orange j=1 t=9: tag=<2,2,2>   green j=1 r=9: tag=<2,2,2>
//      orange j=2 r=1: tag=<1,1,3>   green j=2 t=1: tag=<1,1,3>
//      orange j=3 t=8: tag=<2,2,4>   green j=3 r=8: tag=<2,2,4>
//      ...
//      orange j=9 t=5: tag=<2,2,10>   green j=9 r=5: tag=<2,2,10>

//void json_print_node_info(json_t const *node, int name_flag, int value_flag) {
//  json_t const *child=node->u.c.child, *sibling=node->sibling;
//  fprintf(stderr, "JSON node=%p sibl=%p chil=%p\n", node, sibling, child );
//  if (name_flag  == 1) fprintf(stderr, "JSON node name = %s\n", node->name);
//  if (value_flag == 1) fprintf(stderr, "JSON node value = %s\n", node->u.value);
//}

void config_from_jsom(int m, char const *from_name, char const *to_name, gaps_tag tag) {
  static int  j=0, r=0, t=0;
  static char dir_0 = 't';
  
  if ((strcmp(from_name, enclave_name)) == 0) {
    if (j==0) { dir_0 = 't'; t = 0; r = m-1; }
    log_debug("    tag=<%d,%d,%d> from %s (j=%d t=%d)", tag.mux, tag.sec, tag.typ, enclave_name, j, t);
    get_chan_info(&tag, 't', t);
    if (dir_0 == 't') t++;
    else              t--;
  }
  if ((strcmp(to_name, enclave_name)) == 0) {
    if (j==0) { dir_0 = 'r'; r = 0; t = m-1; }
    log_debug("    tag=<%d,%d,%d> to   %s (j=%d r=%d)", tag.mux, tag.sec, tag.typ, enclave_name, j, r);
    get_chan_info(&tag, 'r', r);
    if (dir_0 == 't') r--;
    else              r++;
  }
  j++;
}

// Get value of string from json object that  matches match_str
void json_get_str(json_t const *j_node, char *match_str, char *value) {
  json_t const *j_prop;           // JSON property (e.g., "name")

  if (JSON_OBJ != json_getType(j_node)) {
    log_fatal("j_node is not a json object (%d)", json_getType(j_node));
    exit(-1);
  }
  j_prop = json_getProperty(j_node, match_str);
  if ( !j_prop || JSON_TEXT != json_getType(j_prop) ) {
    puts("Error, string value is not found.");
    exit(-1);
  }
  strcpy(value, json_getValue(j_prop));
//  log_trace("Value=%s", value);
}

// Get value of integer from json object that matches match_str
int json_get_int(json_t const *j_node, char *match_str) {
  json_t const *j_prop;           // JSON property (e.g., "name")

  if (JSON_OBJ != json_getType(j_node)) {
    log_fatal("j_parent is not a json object");
    exit(-1);
  }
  j_prop = json_getProperty(j_node, match_str);
  if ( !j_prop || JSON_INTEGER != json_getType(j_prop) ) {
    puts("Error, integer value is not found.");
    exit(-1);
  }
  return(atoi(json_getValue(j_prop)));
}

// Get length of json array
int json_get_len(json_t const *j_node) {
  int           m=0;
  json_t const *j;
  
//  log_trace("In get_len function");
  if (JSON_ARRAY != json_getType(j_node)) {
    log_fatal("j_node is not a json array (%d)", json_getType(j_node));
    exit(-1);
  }
//  log_trace("Counting number of children");
  for(j = json_getChild(j_node); j != 0; j = json_getSibling(j)) m++;
//  log_trace("Counted number of children = %d", m);
  return m;
}

// Get child (that is a json array) of a json object that matches match_str
json_t const *json_get_j_array(json_t const *j_node, char *match_str) {
  json_t const *j_child;
  
  if (JSON_OBJ != json_getType(j_node)) {
    log_fatal("j_node is not a json object (%d)", json_getType(j_node));
    exit(-1);
  }
//  j_child = j_node->u.c.child;
//  fprintf(stderr, "XX=%s\n", j_child->name);

  j_child = json_getProperty(j_node, match_str);
  if ( !j_child || JSON_ARRAY != json_getType(j_child) ) {
    log_fatal("JSON array not found");
    exit(-1);
  }
  return (j_child);
}

// Copy JSON file xcf into file_as_str
 int json_open_file(char *xcf, char *file_as_str) {
  FILE   *json_fp;
  int     len;
  
  // A) Copy JSON file into buffer
  json_fp = fopen(xcf, "rb");
  assert(json_fp != NULL);
  len = fread(file_as_str, 1, JSON_OBJECT_SIZE, json_fp);
  fclose(json_fp);
  return (len);
}

// Open and parse JSON configuration file (using json-c library)
//   enum: JSON_OBJ, JSON_ARRAY, JSON_TEXT, JSON_BOOLEAN, JSON_INTEGER, JSON_REAL, JSON_NULL
void read_tiny_json_config_file(char *xcf) {
  char          json_file_as_str[JSON_OBJECT_SIZE];
  json_t        mem[JSON_OBJECT_SIZE];  // json node struct 'array' with ptrs to json_file_as_str
  int           json_file_len, helmap_len;
  gaps_tag      tag;
  json_t const *j_root, *j_child, *j_enclaves, *j_envlave_halmaps, *j_halmap_element;
  char          jstr[MAX_DEV_NAME_LEN], jfrom[MAX_DEV_NAME_LEN], jto[MAX_DEV_NAME_LEN];
  
  // A) Get List of Enclaves
  json_file_len = json_open_file(xcf, json_file_as_str);
  log_trace("JSON FILE len = %d", json_file_len);
  j_root = json_create(json_file_as_str, mem, sizeof mem / sizeof *mem);
  assert(j_root);
  j_enclaves = json_get_j_array(j_root, "enclaves");
  // B) Get Each Enclave
  for(j_child = json_getChild(j_enclaves); j_child != 0; j_child = json_getSibling(j_child)) {
    json_get_str(j_child, "enclave", jstr);
//    log_trace("JSON Enclave = %s (I am %s)", jstr, enclave_name);
    // C) Get Each helmap for this node's enclave
    if ((strcmp(enclave_name, jstr)) == 0) {
//      log_trace("FOUND JSON INFO FOR THIS ENCLAVE");
      j_envlave_halmaps = json_getProperty(j_child, "halmaps");
      helmap_len = json_get_len(j_envlave_halmaps);
//      log_trace("helmap_len=%d", helmap_len);
      for (j_halmap_element = json_getChild(j_envlave_halmaps); j_halmap_element != 0; j_halmap_element = json_getSibling(j_halmap_element)) {
        // D) Get Each helmap element parameters
        json_get_str(j_halmap_element, "from", jfrom);
        json_get_str(j_halmap_element, "to",   jto);
        tag.mux = json_get_int(j_halmap_element, "mux");
        tag.sec = json_get_int(j_halmap_element, "sec");
        tag.typ = json_get_int(j_halmap_element, "typ");
//        log_trace( "%s->%s tag=<%d,%d,%d>", jfrom, jto, tag.mux, tag.sec, tag.typ);
        config_from_jsom(helmap_len, jfrom, jto, tag);
      }
    }
  }
}

// initializing configuration using config file
void config_channels(void) {
  char  *e_env = getenv("ENCLAVE");
  char  *e_xcf = getenv("CONFIG_FILE");

  if (strlen(enclave_name) >= 1) return;    // Do only one (already configured channels)
  strcpy(enclave_name, e_env);
  log_debug("Start %s enclave configuration using config file %s", e_env, e_xcf);
  if ((e_env == NULL) || (e_xcf == NULL)) {
    log_fatal("Must specify environment variables 'ENCLAVE' and 'CONFIG_FILE'");
    exit (-1);
  }
  read_tiny_json_config_file(e_xcf);
  log_trace("Finished %s enclave configuration and created receive thread(s) - ?Wait for threads?", e_env);
//  sleep(2);       // ?? ensure receive thread(s) completed before any messages sent ??
}


/**********************************************************************/
/* Q) Virtual Device: Read/write                                      */
/**********************************************************************/
/* Asynchronously send ADU to DMA driver in 'bw' packet */
void asyn_send(void *adu, gaps_tag *tag) {
  vchan  *cp;        // abstract channel struct pointer for any device type

  // a) Open channel once (and get device type, device name and channel struct
  log_trace("Start of %s for tag=<%d,%d,%d>", __func__, tag->mux, tag->sec, tag->typ);
  cp = get_chan_info(tag, 't', -1);
  
  pthread_mutex_lock(&(cp->lock));
  // b) encode packet into TX buffer and send */
  if (strcmp(cp->dev_type, "dma") == 0)  dma_send(cp, adu, tag);
  if (strcmp(cp->dev_type, "shm") == 0)  shm_send(cp, adu, tag);
  if (strcmp(cp->dev_type, "file") == 0) file_send(cp, adu, tag);
  pthread_mutex_unlock(&(cp->lock));
}

// Receive packets via SHM/DMA in a loop (rate controled by FINISH_XFER blocking call)
void *rcvr_thread_function(thread_args *vargs) {
  vchan       *cp = (vchan *) vargs->cp;

  while (1) {
    log_trace("THREAD-1 %s: fd=%d base_id=%d (ctag=0x%08x)", __func__, cp->fd, vargs->buffer_id_start, ntohl(cp->ctag));
#if 0 >= PRINT_STATE_LEVEL
    vchan_print(cp, enclave_name);
#endif  // PRINT_STATE_LEVEL
    if      (strcmp(cp->dev_type, "dma")  == 0) dma_rcvr(cp);
    else if (strcmp(cp->dev_type, "shm")  == 0) shm_rcvr(cp);
    else if (strcmp(cp->dev_type, "file") == 0) file_rcvr(cp);
    else {
      log_fatal("Unsupported device type %s\n", cp->dev_type);
      FATAL;
    }
  }
}

/* Start a receiver thread */
void rcvr_thread_start(vchan *cp) {
  log_trace("%s: dev=%s dir=%c [ctag=0x%08x]", __func__, cp->dev_name, cp->dir, ntohl(cp->ctag));
  cp->thd_args.cp = cp;
  cp->thd_args.buffer_id_start = 0;
  if (pthread_create(&(cp->thread_id), NULL, (void *) rcvr_thread_function, (void *)&(cp->thd_args)) != 0) FATAL;
}


/* Receive packet from driver (via rx thread), storing data and length in ADU */
int nonblock_recv(void *adu, gaps_tag *tag, vchan *cp) {
  int index_buf = cp->rvpb_index_recv;
  int rv = -1;
  
  pthread_mutex_lock(&(cp->lock));
//  log_trace("%s: Check for received packet on tag=<%d,%d,%d> cp=%p ix=%d new%d", __func__, tag->mux, tag->sec, tag->typ, cp, cp->rvpb_index_recv, cp->rvpb[index_buf].newd);
  if (cp->rvpb[index_buf].newd == 1) {                            // get packet from buffer if available)
#if 0 >= PRINT_STATE_LEVEL
    fprintf(stderr, "%s on buff index=%d", __func__, index_buf);
    vchan_print(cp, enclave_name);
#endif
//#ifdef PRINT_US_TRACE
//    time_trace("RX3 %08x (index=%d)", ntohl(cp->ctag), cp->rvpb_index_recv);
//#endif
    cmap_decode(cp->rvpb[index_buf].data, cp->rvpb[index_buf].data_len, adu, tag, xdc_cmap);   /* Put packet into ADU */
//    log_trace("XDCOMMS reads from buff=%p (index=%d): len=%d", cp->rvpb[index_buf].data, index_buf, cp->rvpb[index_buf].data_len);
    cp->rvpb[index_buf].newd = 0;                      // unmark newdata
    log_trace("XDCOMMS rx packet tag=<%d,%d,%d> len=%d", tag->mux, tag->sec, tag->typ, cp->rvpb[index_buf].data_len);
    if (cp->rvpb[index_buf].data_len > 0) rv = cp->rvpb[index_buf].data_len;
    cp->rvpb_index_recv = (cp->rvpb_index_recv + 1) % cp->rvpb_count;
  }
  pthread_mutex_unlock(&(cp->lock));
  return(rv);
}

/**********************************************************************/
/* T) XDCOMMS Utility functions (not sure if all are still needed)    */
/**********************************************************************/
void tag_print (gaps_tag *tag, FILE * fd) {
  fprintf(fd, "[mux=%02u sec=%02u typ=%02u] ", tag->mux, tag->sec, tag->typ);
}

void tag_write (gaps_tag *tag, uint32_t mux, uint32_t sec, uint32_t typ) {
  tag->mux = mux;
  tag->sec = sec;
  tag->typ = typ;
}

void tag_read (gaps_tag *tag, uint32_t *mux, uint32_t *sec, uint32_t *typ) {
  *mux = tag->mux;
  *sec = tag->sec;
  *typ = tag->typ;
}

/* copy tag_in to tag_out */
void tag_cp (gaps_tag *tag_out, gaps_tag *tag_in) {
  tag_out->mux = tag_in->mux;
  tag_out->sec = tag_in->sec;
  tag_out->typ = tag_in->typ;
}

/* Serialize tag onto wire */
void tag_encode (gaps_tag *tag_out, gaps_tag *tag_in) {
  tag_out->mux = htonl(tag_in->mux);
  tag_out->sec = htonl(tag_in->sec);
  tag_out->typ = htonl(tag_in->typ);
}

/* Convert tag to local host format */
void tag_decode (gaps_tag *tag_out, gaps_tag *tag_in) {
  tag_out->mux = ntohl(tag_in->mux);
  tag_out->sec = ntohl(tag_in->sec);
  tag_out->typ = ntohl(tag_in->typ);
}

/* Convert tag to local host format  */
void len_encode (uint32_t *out, size_t len) {
  *out = ntohl((uint32_t) len);
}

/* Convert tag to local host format */
void len_decode (size_t *out, uint32_t in) {
  *out = (uint32_t) htonl(in);
}

/**********************************************************************/
/* U) XDCOMMS API                                                     */
/**********************************************************************/
// These HAL-API functions are gutted - as irrelevant to xdcomms-lib
void set_address(char *xdc_addr, char *addr_in, const char *addr_default, int *do_once) { }
char *xdc_set_in(char *addr_in) { return NULL; }
char *xdc_set_out(char *addr_in) { return NULL; }
void *xdc_ctx(void) { return NULL; }
void *xdc_pub_socket(void) { return NULL; }
void *xdc_sub_socket(gaps_tag tag) { return NULL; }

/* Set API Logging to a new level */
void xdc_log_level(int new_level) {
  char *ll;
  int   lvl;

  /* XXX: warn if new_level does not match XDCLOGLEVEL */
  if ((ll = getenv("XDCLOGLEVEL")) != NULL) { lvl = atoi(ll); } else { lvl = new_level; }

  if ((lvl >= LOG_TRACE) && (lvl <= LOG_FATAL)) {
    log_set_quiet(0);
    log_set_level(lvl);
    log_trace("Set API log level: %d", lvl);
  }
  else {
    log_trace("Cannot change API to log level %d (min=%d max=%d)\n", __func__, lvl, LOG_TRACE, LOG_FATAL);
  }
}

/* Load Codec Table with ADU encode and decode functions */
/* XXX: must be called at least once so locks are inited and log level defaults set  */
void xdc_register(codec_func_ptr encode, codec_func_ptr decode, int typ) {
         int i;
  static int do_once = 1;

//  xdc_log_level(LOG_TRACE);            /* Mostly Quiet (LOG_TRACE is the most verbose) */
  time_log_level(LOG_TRACE);          /* Print time with us duration (for tracing performance)  */

  if (do_once == 1) {
    do_once = 0;
    for (i=0; i < DATA_TYP_MAX; i++) xdc_cmap[i].valid=0;   /* mark all cmap entries invalid */
  }

  for (i=0; i < DATA_TYP_MAX; i++) {
    if (xdc_cmap[i].data_type == typ) break;
    if (xdc_cmap[i].valid == 0) break;
  }
  if (i >= DATA_TYP_MAX) {
    log_fatal("CMAP table is full (DATA_TYP_MAX=%d)\n", i);
    exit(EXIT_FAILURE);
  }
  xdc_cmap[i].data_type = typ;
  xdc_cmap[i].valid     = 1;
  xdc_cmap[i].encode    = encode;
  xdc_cmap[i].decode    = decode;
  log_trace("API registered new data typ = %d (index=%d)", typ, i);
  config_channels();
}

// Only kept to allow setting receiver timeout value
void *xdc_sub_socket_non_blocking(gaps_tag tag, int timeout) {
  log_trace("Start of %s: timeout = %d ms for tag=<%d,%d,%d>", __func__, timeout, tag.mux, tag.sec, tag.typ);
  config_channels();
  vchan *cp = get_chan_info(&tag, 'r', -1);
//  fprintf(stderr, "timeout = %d ms for tag=<%d,%d,%d>\n", timeout, tag.mux, tag.sec, tag.typ);
  if (timeout > 0) cp->retries = (timeout * NSEC_IN_MSEC)/RX_POLL_INTERVAL_NSEC;     // Set value
  log_trace("%s sets RX retries = %d every %d ns (for ctag=%08x)", __func__, cp->retries, RX_POLL_INTERVAL_NSEC, ntohl(cp->ctag));
  return (NULL);
}

void xdc_asyn_send(void *socket, void *adu, gaps_tag *tag) { asyn_send(adu, tag); }

int  xdc_recv(void *socket, void *adu, gaps_tag *tag) {
  vchan           *cp;
  struct timespec  request;
  int              ntries;
  int              x = 0;

  cp              = get_chan_info(tag, 'r', -1);     // get buffer for tag (to communicate with thread)
  request.tv_sec  = RX_POLL_INTERVAL_NSEC/NSEC_IN_SEC;
  request.tv_nsec = RX_POLL_INTERVAL_NSEC % NSEC_IN_SEC;
  ntries          = 1 + (cp->retries);           // number of tries to rx packet
//  log_trace("%s: test %d times every %d (%d.%09d) ns", __func__, ntries, RX_POLL_INTERVAL_NSEC, request.tv_sec, request.tv_nsec);
  while ((ntries--) > 0)  {
//    if (nonblock_recv(adu, tag, cp) > 0)  return 0;
    if ((x=nonblock_recv(adu, tag, cp)) > 0) {
//#ifdef PRINT_US_TRACE
//      time_trace("RX4 %08x (len=%d)", ntohl(cp->ctag), x);
//#endif
      return x;
    }
//    log_trace("LOOP timeout %s: tag=<%d,%d,%d>: remaining tries = %d ", __func__, tag->mux, tag->sec, tag->typ, ntries);
    nanosleep(&request, NULL);
  }
//  log_trace("%s timeout for tag=<%d,%d,%d>", __func__, tag->mux, tag->sec, tag->typ);
  return -1;
}

/* Receive ADU from HAL - retry until a valid ADU */
void xdc_blocking_recv(void *socket, void *adu, gaps_tag *tag) {
  log_trace("Start of %s: tag=<%d,%d,%d>", __func__, tag->mux, tag->sec, tag->typ);
  while (xdc_recv(socket, adu, tag) < 0);
}
