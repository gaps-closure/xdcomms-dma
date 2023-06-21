#include "vchan.h"

void vchan_print(vchan *cp, char *enclave_name) {
  int index_buf;
  fprintf(stderr, "  %s enclave chan %08x: dir=%c typ=%s nam=%s fd=%d ut=0x%lx wn=%ld ret=%d every %d ns index=%d pkts/chan=%d (max=%d) thread_id=%d\n", enclave_name, ntohl(cp->ctag), cp->dir, cp->dev_type, cp->dev_name, cp->fd, cp->unix_seconds, cp->wait4new_client, cp->retries, RX_POLL_INTERVAL_NSEC, cp->pkt_buf_index, cp->pkt_buf_count, MAX_PKTS_PER_CHAN, cp->thread_id);
  fprintf(stderr, "  mmap len=0x%lx [pa=0x%lx va=%p prot=0x%x flag=0x%x] ",  cp->mm.len, cp->mm.phys_addr, cp->mm.virt_addr, cp->mm.prot, cp->mm.flags);
  fprintf(stderr, "shm addr = %p\n", cp->shm_addr);
  for (index_buf=0; index_buf<(cp->pkt_buf_count); index_buf++) {
    fprintf(stderr, "    i=%d: newd=%d rx_buf_ptr=%p len=%lx tran_id=%d\n", index_buf, cp->rx[index_buf].newd, cp->rx[index_buf].data, cp->rx[index_buf].data_len, cp->rx[index_buf].tid);
  }
}
