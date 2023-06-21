/*
 * xdcomms (Cross Domain Communication) API Library directly between
 * partitioned applications and a GAP's Cross Domain Guard (CDG).
 *
 *
 * v0.4 June 2023: Hardware abstraction layer defines abstract one-way
 * channels (as a vchan) insread of just DMA channels (so renamed
 * 'xdcomms-dma.c' to 'xdcomms.c'). It now supports:
 *   - MIND-DMA XDG:   Direct Memory Access device (see v0.3)
 *   - ESCAPE-SHM XDG: Reading and writing to a specified region of
 *     shared host mmap'ed memory under the control of the ESCAPE.
 *     FPGA boardFor testing, xdcomms can also communicate without
 *     a XDG using any mmap'ed memory area.
 * Xdcomms configuration is done through:
 *   a) Environment variables that specify device and other parameters
 *   b) A channel configuration file that specifies flows among enclaves
 *   c) Header files 'dma-proxy.h' and 'shm-h'
 *
 * v0.3 OCTOBER 2022: Supprted direct transfers between Applications
 * and the MIND-DMA CDG via the MIND proxy DMA driver using IOCTL
 * commands. The driver uses kernel space DMA control to the XILINX AXI
 * DMA/MCDMA driver on the GE MIND ZCU102 FPGA board. For testing, it
 * can also communicate without a XDG using a Pseudo driver emulation.
 *
 *
 * Example commands using test request-reply application (found in ../test/)
 *   p) Enclave 2 responds to a request from enclave 1 over pseudo DMA channels:
 *     ENCLAVE=orange CONFIG_FILE=xdconf_app_req_rep.json DEV_NAME_RX=sue_donimous_rx1 DEV_NAME_TX=sue_donimous_tx1 ./app_req_rep -v -l 1 -e 2 > ~/log_p.txt 2>&1
 *     ENCLAVE=green CONFIG_FILE=xdconf_app_req_rep.json DEV_NAME_RX=sue_donimous_rx0 DEV_NAME_TX=sue_donimous_tx0 ./app_req_rep -v -l 1 > ~/log_q.txt 2>&1
 *
 *   q) Enclave 2 responds to a request from enclave 1 over host DRAM SHM channels:
 *     sudo ENCLAVE=orange CONFIG_FILE=xdconf_app_req_rep.json DEV_TYPE_RX=shm DEV_TYPE_TX=shm DEV_WAIT_NE=1 ./app_req_rep -v -l 1 -e 2 > ~/log_p.txt 2>&1
 *     sudo ENCLAVE=green CONFIG_FILE=xdconf_app_req_rep.json DEV_TYPE_RX=shm DEV_TYPE_TX=shm ./app_req_rep -v -l 1 > ~/log_q.txt 2>&1
 *
 * Example commands using websrv app. Found in ditectories:
 *   v) ENCLAVE=orange CONFIG_FILE=../xdconf.ini DEV_NAME_RX=sue_donimous_rx0 DEV_NAME_TX=sue_donimous_tx0 DEV_WAIT_NE=1 XDCLOGLEVEL=0 LD_LIBRARY_PATH=~/gaps/xdcomms-dma/api MYADDR=10.109.23.126 CAMADDR=10.109.23.151 ./websrv > ~/log_v.txt 2>&1
 *   web) ENCLAVE=green CONFIG_FILE=../xdconf.ini DEV_NAME_RX=sue_donimous_rx1 DEV_NAME_TX=sue_donimous_tx1 LD_LIBRARY_PATH=~/gaps/xdcomms-dma/api XDCLOGLEVEL=0 ./websrv  > ~/log_w.txt 2>&1
 *
 *   w) sudo ENCLAVE=orange CONFIG_FILE=../xdconf.ini DEV_TYPE_RX=shm DEV_TYPE_TX=shm DEV_WAIT_NE=1 XDCLOGLEVEL=0 LD_LIBRARY_PATH=~/gaps/xdcomms-dma/api MYADDR=10.109.23.126 CAMADDR=10.109.23.151 ./websrv > ~/log_v.txt 2>&1
 *   web) sudo ENCLAVE=green CONFIG_FILE=../xdconf.ini DEV_TYPE_RX=shm DEV_TYPE_TX=shm LD_LIBRARY_PATH=~/gaps/xdcomms-dma/api XDCLOGLEVEL=0 ./websrv  > ~/log_w.txt 2>&1
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


#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <assert.h>

#include "xdcomms.h"
#include "vchan.h"

#define PRINT_STATE_LEVEL  2

void rcvr_thread_start(vchan *cp);

char            enclave_name[STR_SIZE] = "";  // enclave name (e.g., green)
vchan           vchan_info[GAPS_TAG_MAX];      // array of buffers to store local virtual channel info per tag
pthread_mutex_t vchan_create;


// XXX DMA and SHM functions should be put into separate functions, with
// xdcomms including DMA + SHM, and DMA + SHM functions including Codec

/**********************************************************************/
/* D1) Open/Configure DMA device                                      */
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
  log_debug("Chan_Buff=0x%lx Bytes (Bytes/PKT=0x%lx Packets/channel=%d) Channels/dev={Tx=%d Rx=%d}", BUFFER_SIZE, sizeof(bw), BUFFER_SIZE / sizeof(bw), DMA_PKT_COUNT_TX, DMA_PKT_COUNT_RX);
}

/**********************************************************************/
/* D2) Tag Compression / Decompression                                    */
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
/* D3) Read/write DMA device                                           */
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

/* Use DMA ioctl operations to tx or rx data */
int dma_start_to_finish(int fd, int *buffer_id_ptr, struct channel_buffer *cbuf_ptr) {
//  log_trace("START_XFER (fd=%d, id=%d buf_ptr=%p unset-status=%d)", fd, *buffer_id_ptr, cbuf_ptr, cbuf_ptr->status);
//  time_trace("DMA Proxy transfer 1 (fd=%d, id=%d)", fd, *buffer_id_ptr);
  ioctl(fd, START_XFER,  buffer_id_ptr);
//  time_trace("DMA Proxy transfer 2 (fd=%d, id=%d)", fd, *buffer_id_ptr);
  ioctl(fd, FINISH_XFER, buffer_id_ptr);
//  time_trace("DMA Proxy transfer 3 (fd=%d, id=%d): status=%d", fd, *buffer_id_ptr, cbuf_ptr->status);
  if (cbuf_ptr->status != PROXY_NO_ERROR) {
//    log_trace("DMA Proxy transfer error (fd=%d, id=%d): st=%d (BUSY=1, TIMEOUT=2, ERROR=3)", fd, *buffer_id_ptr, cbuf_ptr->status);
    return -1;
  }
  return 0;
}

void dma_send(vchan *cp, void *adu, gaps_tag *tag) {
  struct channel_buffer  *dma_tx_chan = (struct channel_buffer *) cp->mm.virt_addr;
  bw        *p;               // Packet pointer
  int       buffer_id=0;      // Use only a single buffer
  size_t    adu_len;    // encoder calculates length */
  size_t    packet_len;
  
  p = (bw *) &(dma_tx_chan->buffer);      // point to a DMA packet buffer */
  time_trace("XDC_Tx1 ready to encode for ctag=%08x into %p", ntohl(cp->ctag), p);
  cmap_encode(p->data, adu, &adu_len, tag);
  bw_gaps_header_encode(p, &packet_len, adu, &adu_len, tag);  /* Put packet into channel buffer */
  log_debug("len: a=%d p=%d p2=%d", adu_len, packet_len, ntohs(p->data_len));
  if (packet_len <= sizeof(bw)) log_buf_trace("TX_PKT", (uint8_t *) &(dma_tx_chan->buffer), packet_len);
  dma_start_to_finish(cp->fd, &buffer_id, dma_tx_chan);
  log_debug("XDCOMMS tx packet tag=<%d,%d,%d> len=%ld", tag->mux, tag->sec, tag->typ, packet_len);
//  log_trace("%s: Buffer id = %d packet pointer=%p", buffer_id, p);
}

void dma_rcvr(vchan *cp, int index_buf) {
  bw                    *p;
  struct channel_buffer *dma_cb_ptr = (struct channel_buffer *) cp->mm.virt_addr;  /* start of channel buffer */
  static int             dma_cb_index=0;

//  log_trace("%s ctag=%x %s cp=%p va=%p dma=%p ID=%d len=%ld", __func__, ntohl(cp->ctag), cp->dev_name, cp, cp->mm.virt_addr, dma_cb_ptr, index_buf, sizeof(bw));
  dma_cb_ptr[dma_cb_index].length = sizeof(bw);      /* Receive up to make length Max size */
  log_debug("THREAD-2 waiting for %s tag=0x%08x on dev=%s with cb_index=%d, max_len=%d)", cp->dev_type, ntohl(cp->ctag), cp->dev_name, dma_cb_index, dma_cb_ptr[dma_cb_index].length);
  while (dma_start_to_finish(cp->fd, &dma_cb_index, &(dma_cb_ptr[dma_cb_index])) != 0) { ; }
  p = (bw *) &(dma_cb_ptr[dma_cb_index].buffer);    /* XXX: DMA buffer must be larger than size of BW */
  
  unsigned long *data_ptr = (unsigned long *) &(dma_cb_ptr[dma_cb_index].buffer);
  log_trace("%s len=%d data[0] = %p 0x%08x 0x%08x", __func__, ntohs(p->data_len), data_ptr, ntohl(data_ptr[0]), ntohl(data_ptr[1]));
  pthread_mutex_lock(&(cp->lock));
  bw_len_decode(&(cp->rx[index_buf].data_len), p->data_len);
  log_trace("THREAD-3 rx packet cb_index=%d status=%d len=%d", dma_cb_index, dma_cb_ptr[dma_cb_index].status, cp->rx[index_buf].data_len);
  cp->rx[index_buf].data = (uint8_t *) p->data;
  cp->rx[index_buf].newd = 1;
  pthread_mutex_unlock(&(cp->lock));
  dma_cb_index++;
}


/**********************************************************************/
/* S1) Open/Configure SHM device                                      */
/**********************************************************************/
// Open SHM channel given Physical address and length. Returns fd, mmap-va and mmap-len
void shm_open_channel(vchan *cp) {
  void          *pa_virt_addr;
  unsigned long  pa_phys_addr, pa_mmap_len;       /* page aligned physical address (offset) */

  // a) Open device
  if ((cp->fd = open(cp->dev_name, O_RDWR | O_SYNC)) == -1) FATAL;
  // b) mmpp device: reduce address to be a multiple of page size and add the diff to length
  pa_phys_addr       = cp->mm.phys_addr & ~MMAP_PAGE_MASK;
  pa_mmap_len        = cp->mm.len + cp->mm.phys_addr - pa_phys_addr;
//  log_trace("SHM len = 0x%x = 0x%x", cp->mm.len, pa_mmap_len);
  pa_virt_addr       = mmap(0, pa_mmap_len, cp->mm.prot, cp->mm.flags, cp->fd, pa_phys_addr);
  if (pa_virt_addr == (void *) MAP_FAILED) FATAL;   // MAP_FAILED = -1
  cp->mm.virt_addr = pa_virt_addr + cp->mm.phys_addr - pa_phys_addr;   // add offset to page aligned addr
  log_debug("Opened and mmap'ed SHM channel %s (fd=%d): addr=(p=0x%lx v=%0x%lx - %p = len=0x%x ", cp->dev_name, cp->fd, cp->mm.virt_addr, pa_phys_addr, ((unsigned long) cp->mm.virt_addr) + pa_mmap_len - 1, pa_mmap_len);
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
/* S2) SHM read/write functions                                  */
/**********************************************************************/
// Dumb memory copy using 8-byte words
//void naive_memcpy(unsigned long *d, const unsigned long *s, unsigned long len_in_words) {
//  for (int i = 0; i < len_in_words; i++) *d++ = *s++;
//}

void shm_send(vchan *cp, void *adu, gaps_tag *tag) {
  int            *last_ptr    = &(cp->shm_addr->pkt_index_last);
  int            *next_ptr    = &(cp->shm_addr->pkt_index_next);
  int             write_index = *next_ptr;
  size_t          adu_len     = 0;    // encoder calculates length */
  struct timespec ts;                 // Guard time
  
  ts.tv_sec  = 0;
  ts.tv_nsec = DEFAULT_NS_GUARD_TIME_BW;
  log_trace("%s TX index: next = %d last = %d (guard_bw=%d)", __func__, *next_ptr, *last_ptr, DEFAULT_NS_GUARD_TIME_BW);
  // A) Delete oldest message?
  if (*last_ptr == write_index) {
    *last_ptr = ((*last_ptr) + 1) % SHM_PKT_COUNT;
    nanosleep(&ts, NULL);       // Give time to finish reading before writing into last
  }
  if (*last_ptr < 0) *last_ptr = 0;   // If the first written packet
//  time_trace("XDC_Tx1 ready to encode for ctag=%08x (next = %d last = %d)", ntohl(cp->ctag), write_index, *last_ptr);
  
  // B) Encode Data into SHM
  cmap_encode(cp->shm_addr->pdata[write_index].data, adu, &adu_len, tag);
//  XXX TODO: incoprate naive_memcpy into cmap_encode/decode
//  naive_memcpy(cp->shm_addr->pdata[pkt_index_nxt].data, adu, adu_len);  // TX adds new data
  cp->shm_addr->pinfo[write_index].data_length = adu_len;
  *next_ptr = (write_index + 1) % SHM_PKT_COUNT;
//  log_trace("data[19]=%0x", cp->shm_addr->pdata[write_index].data[19]);
  
  time_trace("TX %08x (len=%d index=%d)", ntohl(cp->ctag), adu_len, write_index);
#if 1 >= PRINT_STATE_LEVEL
  shm_info_print(cp->shm_addr);
#endif  // PRINT_STATE
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
  if      ((cp->wait4new_client) == 0) log_trace("Not wait for client");
  else if (rx_tim <= tx_tim) log_trace("New client");
  else return(-1);                    // wait for new client
  return(0);
}

void shm_rcvr(vchan *cp, int index_buf) {
  static int once = 1;
  if (once == 1) {
    while (wait_if_old(cp)) { ; }
    once = 0;
  }
  log_debug("THREAD-2 waiting for %s tag=0x%08x on dev=%s with index=%d (last=%d)", cp->dev_type, ntohl(cp->ctag), cp->dev_name, index_buf, cp->shm_addr->pkt_index_next);
  while (index_buf == (cp->shm_addr->pkt_index_next)) { ; }
//  log_trace("THREAD-3a got packet index=%d (next=%d last=%d) len=%d [tr=0x%lx - tt=0x%lx = 0x%lx]", index_buf, cp->shm_addr->pkt_index_next, cp->shm_addr->pkt_index_last, cp->shm_addr->pinfo[index_buf].data_length, cp->unix_seconds, (cp->unix_seconds) - (cp->shm_addr->cinfo.unix_seconds), cp->shm_addr->cinfo.unix_seconds);
  pthread_mutex_lock(&(cp->lock));
  log_debug("THREAD-3 rx packet ctag=0x%08x len=%d id=%d", ntohl(cp->ctag), cp->shm_addr->pinfo[index_buf].data_length, index_buf);
#if 1 >= PRINT_STATE_LEVEL
  shm_info_print(cp->shm_addr);
#endif  // PRINT_STATE
  cp->rx[index_buf].data_len = cp->shm_addr->pinfo[index_buf].data_length;
  cp->rx[index_buf].data     = (uint8_t *) (cp->shm_addr->pdata[index_buf].data);
  cp->rx[index_buf].newd     = 1;
//  fprintf(stderr, "PPP %p = %08x ", cp->shm_addr->pdata[index_buf].data, ntohl(cp->shm_addr->pdata[index_buf].data[4]));
  pthread_mutex_unlock(&(cp->lock));
}


/**********************************************************************/
/* A) Virtual Device open                                               */
/**********************************************************************/
// Open channel device (based on name and type) and return its channel structure
void open_device(vchan *cp) {
  log_trace("%s of type=%s name=%s", __func__, cp->dev_type, cp->dev_name);
  if      (strcmp(cp->dev_type, "dma") == 0) dma_open_channel(cp);
  else if (strcmp(cp->dev_type, "shm") == 0) shm_open_channel(cp);
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
      log_debug("%s: Opened new device %s dir=%c ctag=0x%08x i=%d", __func__, cp->dev_name, cp->dir, ntohl(cp->ctag), i);
      return;  // new device
    }
    // b2) See if device is already open
    if (strcmp(cp->dev_name, clist[i].cp->dev_name) == 0) {    // Already set, so open info
      cp->fd = clist[i].cp->fd;
      cp->mm.virt_addr = clist[i].cp->mm.virt_addr;
      (clist[i].count)++;
      log_debug("%s: %s device shared %d times for i=%d", __func__, cp->dev_name, clist[i].count, i);
      return;
    }
  }
  FATAL;    // Only here if list cannot store all devices (> MAX_DEV_COUNT)
}


/**********************************************************************/
/* B) Virtual Channel Configuration  (for all devices and TX/RX)      */
/**********************************************************************/
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
        vchan_info[i].rx[index_buf].newd = 0;
        vchan_info[i].rx[index_buf].data_len = 0;
        vchan_info[i].rx[index_buf].data = NULL;
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
void get_dev_name(char *dev_name, char *env_name, char *def_name_dma, char *def_name_shm, char *dev_type) {
  strcpy(dev_name, "/dev/");        // prefix device name
  if      ((strcmp(dev_type, "dma")) == 0) {
    (env_name == NULL) ? strcat(dev_name, def_name_dma) : strcat(dev_name, env_name);
  }
  else if ((strcmp(dev_type, "shm")) == 0) {
    (env_name == NULL) ? strcat(dev_name, def_name_shm) : strcat(dev_name, env_name);
  }
  else FATAL;
}

// Get channel device number (*val) from enivronment or default (for that type)
void get_dev_val(unsigned long *val, char *env_val, unsigned long def_val_dma, unsigned long def_val_shm, char *dev_type) {
  if (env_val == NULL) {
    if       (strcmp(dev_type, "dma") == 0) *val = def_val_dma;
    else if  (strcmp(dev_type, "shm") == 0) *val = def_val_shm;
    else     FATAL;
  }
  else       *val = strtol(env_val, NULL, 16);
}

// Initialize configuration for a new tag based on environment variables
void init_new_chan_from_envi(vchan *cp, uint32_t ctag, char dir) {
  cp->ctag = ctag;
  cp->dir  = dir;

  log_trace("%s: ctag=0x%08x dir=%c TX=%s RX=%s", __func__, ntohl(ctag), dir, getenv("DEV_NAME_TX"), getenv("DEV_NAME_RX"));
  if (dir == 't') { // TX
    get_dev_type(cp->dev_type,     getenv("DEV_TYPE_TX"), "dma");
    get_dev_name(cp->dev_name,     getenv("DEV_NAME_TX"), "dma_proxy_tx", "mem", cp->dev_type);
    get_dev_val (&(cp->mm.len),    getenv("DEV_MMAP_LE"), (sizeof(struct channel_buffer) * DMA_PKT_COUNT_TX), SHM_MMAP_LEN, cp->dev_type);
  }
  else {            // RX
    get_dev_type(cp->dev_type,     getenv("DEV_TYPE_RX"), "dma");
    get_dev_name(cp->dev_name,     getenv("DEV_NAME_RX"), "dma_proxy_rx", "mem", cp->dev_type);
    get_dev_val (&(cp->mm.len),    getenv("DEV_MMAP_LE"), (sizeof(struct channel_buffer) * DMA_PKT_COUNT_RX), SHM_MMAP_LEN, cp->dev_type);   // XXX dma default is an over estimate
    get_dev_val (&(cp->wait4new_client), getenv("DEV_WAIT_NE"), 0x0, 0x0, cp->dev_type);
  }
  get_dev_val(&(cp->mm.phys_addr), getenv("DEV_MMAP_AD"), DMA_ADDR_HOST, SHM_MMAP_ADDR, cp->dev_type);
//  log_trace("%s Env Vars: type=%s name=%s mlen=%s (%", __func__, getenv("DEV_TYPE_TX"), getenv("DEV_NAME_TX"), getenv("DEV_MMAP_LE"));
//  log_trace("%s Env Vars: type=%s name=%s mlen=%s", __func__, getenv("DEV_TYPE_RX"), getenv("DEV_NAME_RX"), , getenv("DEV_MMAP_LE"));
}

// Configure new channel
void init_new_chan(vchan *cp, uint32_t ctag, char dir, int index) {
  static int once=1;

  init_new_chan_from_envi(cp, ctag, dir);   // 1) Configure new tag
  dev_open_if_new(cp);                   // 2) Open device (if not already open)
//      log_trace("%s: Using %s device %s for ctag=0x%08x dir=%c", __func__, cp->dev_type, cp->dev_name, ntohl(cp->ctag), cp->dir);
  cp->pkt_buf_index = 0;
  if ((strcmp(cp->dev_type, "shm")) == 0) {
    cp->shm_addr      = cp->mm.virt_addr + (index * sizeof(shm_channel));
    cp->pkt_buf_count = SHM_PKT_COUNT;
    if ((cp->dir) == 't') shm_init_config_one(cp);  // 3) Configure SHM structure for new channel
    if ((cp->dir) == 'r') {
      rcvr_thread_start(cp);      // 4) Start rx thread for new receive tag
    }
  }
  else if ((strcmp(cp->dev_type, "dma")) == 0) {
    if ((cp->dir) == 'r') {
      cp->pkt_buf_count = DMA_PKT_COUNT_RX;
      if (once==1) {
        rcvr_thread_start(cp);      // 4) Start rx thread only once
        once = 0;
      }
    }
    else {
      cp->pkt_buf_count = DMA_PKT_COUNT_TX;
    }
  }
  else {log_warn("Unknown type=%s (name=%s)", cp->dev_type, cp->dev_name); FATAL;}
#if 1 >= PRINT_STATE_LEVEL
  vchan_print(cp, enclave_name);
#endif  // LOG_LEVEL_MIN
}

// Return pointer to Rx packet buffer for specified tag
//  a) If first call, then initialize all channels
//  b) If first call for a tag: 1) config channel info, 2) open device if new, 3) start thread
vchan *get_chan_info(gaps_tag *tag, char dir, int index) {
  uint32_t  ctag;
  int       chan_index;
  vchan    *cp;
  
  /* a) Initilize all channels (after locking from other application threads) */
  pthread_mutex_lock(&vchan_create);
  vchan_init_all_once();
  /* b) Find info for this tag (and possibly initialize*/
  ctag_encode(&ctag, tag);                   // Encoded ctag
  for(chan_index=0; chan_index < GAPS_TAG_MAX; chan_index++) {  // Break on finding tag or empty
    cp = &(vchan_info[chan_index]);
    if (cp->ctag == ctag) break;             // found existing slot for tag
    if (cp->ctag == 0) {                     // found empty slot (before tag)
      init_new_chan(cp, ctag, dir, index);
      break;
    }
  }
  /* c) Unlock and return chan_info pointer */
  if (chan_index >= GAPS_TAG_MAX) FATAL;
//  log_trace("%s chan_index=%d: ctag=0x%08x", __func__, chan_index, ctag);
  pthread_mutex_unlock(&vchan_create);
  return (cp);
}

/**********************************************************************/
/* C) Extract JSON Configuration Info (for all devices and TX/RX)     */
/**********************************************************************/
// For each (of m) halmaps in JSON file, proces the flow
//   Ensure a) Both sides use the same index for the same tag
//          b) Group indexes for same direction
//   For example using websrv app:
//      orange j=0 r=0: tag=<1,1,1>   green j=0 t=0: tag=<1,1,1>
//      orange j=1 t=9: tag=<2,2,2>   green j=1 r=9: tag=<2,2,2>
//      orange j=2 r=1: tag=<1,1,3>   green j=2 t=1: tag=<1,1,3>
//      orange j=3 t=8: tag=<2,2,4>   green j=3 r=8: tag=<2,2,4>
//      ...
//      orange j=9 t=5: tag=<2,2,10>   green j=9 r=5: tag=<2,2,10>
void json_process_all_flows(int m, struct json_object *j_envlave_halmaps) {
  int                 j, r, t;
  char                dir_0, from_name[STR_SIZE], to_name[STR_SIZE];
  gaps_tag            tag;
  struct json_object *j_halmap_element, *j_halmap_from, *j_halmap_to;
  struct json_object *j_halmap_mux, *j_halmap_sec, *j_halmap_typ;
  
  for (j=0; j<m; j++) {
    j_halmap_element = json_object_array_get_idx(j_envlave_halmaps, j);
    j_halmap_from    = json_object_object_get(j_halmap_element, "from");
    j_halmap_to      = json_object_object_get(j_halmap_element, "to");
    strcpy(from_name, json_object_get_string(j_halmap_from));
    strcpy(to_name,   json_object_get_string(j_halmap_to));
    j_halmap_mux = json_object_object_get(j_halmap_element, "mux");
    j_halmap_sec = json_object_object_get(j_halmap_element, "sec");
    j_halmap_typ = json_object_object_get(j_halmap_element, "typ");
    tag.mux = json_object_get_int(j_halmap_mux);
    tag.sec = json_object_get_int(j_halmap_sec);
    tag.typ = json_object_get_int(j_halmap_typ);
    // Match and Group Indexes (see function description)
    if ((strcmp(from_name, enclave_name)) == 0) {
      if (j==0) { dir_0 = 't'; t = 0; r = m-1; }
      log_debug("    %s j=%d t=%d: tag=<%d,%d,%d>", enclave_name, j, t, tag.mux, tag.sec, tag.typ);
      get_chan_info(&tag, 't', t);
      if (dir_0 == 't') t++;
      else              t--;
    }
    if ((strcmp(to_name, enclave_name)) == 0) {
      if (j==0) { dir_0 = 'r'; r = 0; t = m-1; }
      log_debug("    %s j=%d r=%d: tag=<%d,%d,%d>", enclave_name, j, r, tag.mux, tag.sec, tag.typ);
      get_chan_info(&tag, 'r', r);
      if (dir_0 == 't') r--;
      else              r++;
    }
  }
}

// Open and parse JSON configuration file (using json-c library)
void read_json_config_file(char *xcf) {
  int                 n, i, m;
  char                encl_name[STR_SIZE];
  struct json_object *j_root, *j_enclaves;
  struct json_object *j_enclave_element, *j_enclave_name, *j_envlave_halmaps;

  j_root = json_object_from_file(xcf);
  if (!j_root) { log_fatal("Could not find JSON file: %s", xcf); FATAL; }
  // A) Get all enclaves
  j_enclaves = json_object_object_get(j_root, "enclaves");
  n = json_object_array_length(j_enclaves);
  log_trace("%d enclaves in %s file", n, xcf);
  // B) For each (of n) enclaves in JSON list, get enclave information and halmaps
  for (i=0; i<n; i++) {
    j_enclave_element = json_object_array_get_idx(j_enclaves, i);
    j_enclave_name = json_object_object_get(j_enclave_element, "enclave");
    strcpy(encl_name, json_object_get_string(j_enclave_name));
    log_trace("  %d: enclave=%s", i, json_object_get_string(j_enclave_name));
    // C) Take halmaps from this nodes enclave
    if ((strcmp(enclave_name, encl_name)) == 0) {
      j_envlave_halmaps = json_object_object_get(j_enclave_element, "halmaps");
      m = json_object_array_length(j_envlave_halmaps);
      log_trace("  halmaps len=%d", m);
      json_process_all_flows(m, j_envlave_halmaps);
    }
  }
}

// initializing configuration using config file
void config_channels(void) {
  char  *e_env = getenv("ENCLAVE");
  char  *e_xcf = getenv("CONFIG_FILE");

  if (strlen(enclave_name) >= 1) return;    // already configured channels
  strcpy(enclave_name, e_env);
  log_debug("%s enclave initializing using config file %s", e_env, e_xcf);
  if ((e_env == NULL) || (e_xcf == NULL)) {
    log_fatal("Must specify environment variables 'ENCLAVE' and 'CONFIG_FILE'");
    exit (-1);
  }
  read_json_config_file(e_xcf);
}


/**********************************************************************/
/* D) Virtual Device read/write functions                                  */
/**********************************************************************/
/* Asynchronously send ADU to DMA driver in 'bw' packet */
void asyn_send(void *adu, gaps_tag *tag) {
  vchan  *cp;        // abstract channel struct pointer for any device type

  // a) Open channel once (and get device type, device name and channel struct
  log_debug("Start of %s for tag=<%d,%d,%d>", __func__, tag->mux, tag->sec, tag->typ);
  cp = get_chan_info(tag, 't', 0);
  pthread_mutex_lock(&(cp->lock));
  // b) encode packet into TX buffer and send */
  if (strcmp(cp->dev_type, "dma") == 0) dma_send(cp, adu, tag);
  if (strcmp(cp->dev_type, "shm") == 0) shm_send(cp, adu, tag);
  pthread_mutex_unlock(&(cp->lock));
}

// Receive packets via DMA in a loop (rate controled by FINISH_XFER blocking call)
void *rcvr_thread_function(thread_args *vargs) {
  vchan *cp = vargs->cp;
  int    buffer_id = 0;

  while (1) {
    log_trace("THREAD-1 %s: tag=0x%08x fd=%d index=%d (base_id=%d)", __func__, ntohl(cp->ctag), cp->fd, buffer_id, vargs->buffer_id_start);
#if 0 >= PRINT_STATE_LEVEL
    vchan_print(cp, enclave_name);
#endif  // PRINT_STATE_LEVEL
//    buffer_id = (vargs->buffer_id_start) + buffer_id_index;
    if      (strcmp(cp->dev_type, "dma") == 0) dma_rcvr(cp, buffer_id);
    else if (strcmp(cp->dev_type, "shm") == 0) shm_rcvr(cp, buffer_id);
    else {
      log_fatal("Unsupported device type %s\n", cp->dev_type);
      FATAL;
    }
    buffer_id = (buffer_id + 1) % (cp->pkt_buf_count);
    log_trace("THREAD-4 index=%d", buffer_id);
  }
}

/* Start a receiver thread */
void rcvr_thread_start(vchan *cp) {
  log_info("%s: ctag=0x%08x dev=%s dir=%c", __func__, ntohl(cp->ctag), cp->dev_name, cp->dir);
  cp->thd_args.cp = cp;
  cp->thd_args.buffer_id_start = 0;
  if (pthread_create(&(cp->thread_id), NULL, (void *) rcvr_thread_function, (void *)&(cp->thd_args)) != 0) FATAL;
}

/* Receive packet from driver (via rx thread), storing data and length in ADU */
int nonblock_recv(void *adu, gaps_tag *tag, vchan *cp) {
  int index_buf = cp->pkt_buf_index;
  int rv = -1;
  
  pthread_mutex_lock(&(cp->lock));
//  log_trace("%s: Check for received packet on tag=<%d,%d,%d>", __func__, tag->mux, tag->sec, tag->typ);
  if (cp->rx[index_buf].newd == 1) {                            // get packet from buffer if available)
#if 0 >= PRINT_STATE_LEVEL
    fprintf(stderr, "%s on buff index=%d", __func__, index_buf);
    vchan_print(cp, enclave_name);
#endif
    cmap_decode(cp->rx[index_buf].data, cp->rx[index_buf].data_len, adu, tag);   /* Put packet into ADU */
//    log_trace("XDCOMMS reads from buff=%p (index=%d): len=%d", cp->rx[index_buf].data, index_buf, cp->rx[index_buf].data_len);
    cp->rx[index_buf].newd = 0;                      // unmark newdata
    log_debug("XDCOMMS rx packet tag=<%d,%d,%d> len=%d", tag->mux, tag->sec, tag->typ, cp->rx[index_buf].data_len);
    if (cp->rx[index_buf].data_len > 0) rv = cp->rx[index_buf].data_len;
    cp->pkt_buf_index = (index_buf + 1) % (cp->pkt_buf_count);
  }
  pthread_mutex_unlock(&(cp->lock));
  return(rv);
}

/**********************************************************************/
/* E) XDCOMMS Utility functions (not sure if all are still needed)  */
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
/* F) XDCOMMS API                                                     */
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
    for (i=0; i < DATA_TYP_MAX; i++) cmap[i].valid=0;   /* mark all cmap entries invalid */
  }

  for (i=0; i < DATA_TYP_MAX; i++) {
    if (cmap[i].data_type == typ) break;
    if (cmap[i].valid == 0) break;
  }
  if (i >= DATA_TYP_MAX) {
    log_fatal("CMAP table is full (DATA_TYP_MAX=%d)\n", i);
    exit(EXIT_FAILURE);
  }
  cmap[i].data_type = typ;
  cmap[i].valid     = 1;
  cmap[i].encode    = encode;
  cmap[i].decode    = decode;
  log_trace("API registered new data typ = %d (index=%d)", typ, i);
  config_channels();
}

// Only kept to allow setting receiver timeout value
void *xdc_sub_socket_non_blocking(gaps_tag tag, int timeout) {
  log_debug("Start of %s: timeout = %d ms for tag=<%d,%d,%d>", __func__, timeout, tag.mux, tag.sec, tag.typ);
  config_channels();
  vchan *cp = get_chan_info(&tag, 'r', 0);
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

//  log_debug("Start of %s: tag=<%d,%d,%d>", __func__, tag->mux, tag->sec, tag->typ);
  cp              = get_chan_info(tag, 'r', 0);     // get buffer for tag (to communicate with thread)
  request.tv_sec  = RX_POLL_INTERVAL_NSEC/NSEC_IN_SEC;
  request.tv_nsec = RX_POLL_INTERVAL_NSEC % NSEC_IN_SEC;
  ntries          = 1 + (cp->retries);           // number of tries to rx packet
//  log_trace("%s: test %d times every %d (%d.%09d) ns", __func__, ntries, RX_POLL_INTERVAL_NSEC, request.tv_sec, request.tv_nsec);
  while ((ntries--) > 0)  {
//    if (nonblock_recv(adu, tag, cp) > 0)  return 0;
    if ((x=nonblock_recv(adu, tag, cp)) > 0) {
      time_trace("RX %08x (len=%d)", ntohl(cp->ctag), x);
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
  log_debug("Start of %s: tag=<%d,%d,%d>", __func__, tag->mux, tag->sec, tag->typ);
  while (xdc_recv(socket, adu, tag) < 0);
}
