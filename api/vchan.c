#include "vchan.h"

void vchan_print(vchan *cp, char *enclave_name) {
  int index_buf;

  fprintf(stderr, "  %s enclave chan %08x: dir=%c typ=%s nam=%s fd=%d ut=0x%lx wn=%ld\n", enclave_name, ntohl(cp->ctag), cp->dir, cp->dev_type, cp->dev_name, cp->fd, cp->unix_seconds, cp->wait4new_client);
  fprintf(stderr, "  ret=%d every %d ns index=(t=%d r=%d of %d) max=%d thread_id=%ld\n", cp->retries, RX_POLL_INTERVAL_NSEC, cp->rvpb_index_thrd, cp->rvpb_index_recv, cp->rvpb_count, MAX_PKTS_PER_CHAN, cp->thread_id);

  for (index_buf=0; index_buf<(cp->rvpb_count); index_buf++) {
    fprintf(stderr, "    i=%d: newd=%d rx_buf_ptr=%p len=%lx tran_id=%d\n", index_buf, cp->rvpb[index_buf].newd, cp->rvpb[index_buf].data, cp->rvpb[index_buf].data_len, cp->rvpb[index_buf].tid);
  }

  if ( (strcmp(cp->dev_type, "dma") == 0) || (strcmp(cp->dev_type, "shm") == 0) ) {
    fprintf(stderr, "  mmap len=0x%lx [pa=0x%lx va=%p prot=0x%x flag=0x%x]\n",  cp->mm.len, cp->mm.phys_addr, cp->mm.virt_addr, cp->mm.prot, cp->mm.flags);
  }
  if (strcmp(cp->dev_type, "shm") == 0)  fprintf(stderr, "  SHM XDC: addr = %p\n", cp->shm_addr);
  if (strcmp(cp->dev_type, "file") == 0) fprintf(stderr, "  FILE XDC: pkt_index_last=%d pkt_index_next=%d pkt addr = %p\n", cp->file_info->pkt_index_last, cp->file_info->pkt_index_next, cp->file_info);
}
