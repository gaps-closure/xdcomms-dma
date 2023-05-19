#ifndef _DMA_PROXY_H_
#define _DMA_PROXY_H_
/**
 * Copyright (C) 2021 Xilinx, Inc
 *
 * Licensed under the Apache License, Version 2.0 (the "License"). You may
 * not use this file except in compliance with the License. A copy of the
 * License is located at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */
 /* This header file is shared by the DMA Proxy test application and the DMA
 * Proxy device driver. It defines the shared interface to allow DMA transfers
 * to be done from user space.
 *
 * The driver creates separate channel buffers for each transmit and receive
 * channel. The application may choose to use only a subset of the channel
 * buffers to allow prioritization of transmit vs receive. The buffer in the
 * data structure must be 1st in the channel interface so that the buffer
 * is cached aligned.
 */

#define BUFFER_SIZE (128 * 1024)	 	    /* must match driver exactly */
#ifdef SUE_DONIMOUS
#define BUFFER_COUNT 16					        /* driver only */
#else
#define BUFFER_COUNT 32					        /* driver only */
#endif

#define DMA_TX_PKT_BUF_COUNT 	1				      /* app only, must be <= to the number in the driver */
#define RX_BUFFER_COUNT 	    BUFFER_COUNT  /* app only, must be <= to the number in the driver */
#define BUFFER_INCREMENT	    1				      /* normally 1, but skipping buffers (2) defeats CPU prefetching */

#define FINISH_XFER 	   _IOW('a','a',int32_t*)
#define START_XFER 		   _IOW('a','b',int32_t*)
#define XFER 			       _IOR('a','c',int32_t*)

#define PKT_G1_ADU_SIZE_MAX   65528  // Max packet size with 16-bit data_len = 2^16 - 8 (see bw header)

enum proxy_status { PROXY_NO_ERROR = 0, PROXY_BUSY = 1, PROXY_TIMEOUT = 2, PROXY_ERROR = 3 };

struct channel_buffer {
	unsigned int      buffer[BUFFER_SIZE / sizeof(unsigned int)]; /* must be first field, do not move */
  enum proxy_status status;
	unsigned int      length;
} __attribute__ ((aligned (1024)));		/* 64 byte alignment required for DMA, but 1024 handy for viewing memory */

typedef struct _dma_channel {
  struct channel_buffer *buf_ptr;
  int    fd;
} dma_channel;

/* BW packet format (MIND format) */
typedef struct _sdh_bw {
  uint32_t  message_tag_ID;             /* Compressed Application Mux, Sec, Typ */
  uint16_t  data_len;                   /* Length (in bytes) */
  uint16_t  crc16;                      /* Error detection field */
  uint8_t   data[PKT_G1_ADU_SIZE_MAX];  /* Application data unit */
} bw;

#define DMA_ADDR_HOST     0x0UL            // Host System selects mmap physical memory address

#endif /* _DMA_PROXY_H */
