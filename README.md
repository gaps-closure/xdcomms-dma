# XDCOMMS Library

Partitioned application programs use the CLOSURE Cross-Domain Communication (XDCOMMS) library to communicate data through a Cross-Domain Guard (CDG). 


## XDCOMMS API

XDCOMMS-lib maintains the same API as the Hardware Abstraction Layer (HAL).
The calls to these API functions in the partitioned applications are 
automatically generated by the CLOSURE toolchain (RPC generator).

The API uses HAL tag structures to identify the abstract communication channels
with three orthogonal 32-bit unsigned identifiers:

```
typedef struct _tag {
  uint32_t    mux;      // session multiplexing handle identifying a unidirectional APP flow
  uint32_t    sec;      // identifies a CDG security policy used to processing APP data
  uint32_t    typ;      // identifies the data type (based on DFDL xsd definition)
} gaps_tag;
```

As the CLOSURE codecs handle serialization and de-serialization, applications can 
send and receive data using pointers to in-memory data structures and the 
tag (see above) for data item to be sent or received. In particular xdcomms provices: 
a) an asynchronous send, 
b) a blocking receive (until a message matching the specified tag is received), and 
c) a receive which supports a timeout (specified in the xdc_sub_socket_non_blocking() call). 

- void  xdc_asyn_send(void *socket, void *adu, gaps_tag *tag);
- int   xdc_recv(void *socket, void *adu, gaps_tag *tag);
- void  xdc_blocking_recv(void *socket, void *adu, gaps_tag *tag);

The socket pointer is not used for xdcomms-lib, so may be specified as NULL.
In addition to the send/receive calls, other calls specify the API log level, 
the receive timeout and the codec function (generated by CLOSURE):

- void  xdc_log_level(int new_level);
- void *xdc_sub_socket_non_blocking(gaps_tag tag, int timeout);
- void  xdc_register(codec_func_ptr encode, codec_func_ptr decode, int typ);


## XDCOMMS SUPPORTED Cross-Domain Guards

XDCOMMS-lib current version (version 0.5, October 2023) supports three main types of CDG:
- GE-MIND: Communicating with a proxy DMA driver using IOCTL commands, which in turn communicates with the XILINX AXI DMA/MCDMA driver on the GE MIND ZCU102 FPGA board.
- INTEL-ESCAPE: Copying data to and from shared memory mapped regions on the ESCAPE FPGA board. 
- X-ARBITOR: Reading a writing files with the help of the X-ARBITOR send and receive proxies.

In addition XDCOMMS-lib can be tested on a single host computer using: 
- Pseudo driver: which emulates the proxy DMA driver.
- Shared memory mapped regions of the host computer: as a stand-in for the ESCAPE FPGA board. 
- Files in directories on the host computer: as stand-in for the X-ARBITOR.


## XDCOMMS Configuration

One-way channel definitions (including definition of channel tags) are now directly read from the CLOSURE generated JSON configuration file (xdcomms.ini). Selection of device configuration is done through environment variables specified when running the partitioned application.


## XDCOMMS Installation
To install the xdcomms library, together with the test application and pseudo driver run the following commands:
```
  git clone git@github.com:gaps-closure/xdcomms-dma
  cd xdcomms-dma/
  make clean
  make 
```


## Running the Test Application

A test application is included with the xdcomms  
```
  cd ~/gaps/xdcomms-dma/test_app
```

### PSEUDO DMA (on a single host)
First, ensure that the psuedo driver is compiled and loaded:
```
    cd ~/gaps/xdcomms-dma/pseudo
    make
    sudo ./sue_donimous_unload
    sudo ./sue_donimous_load
    lsmod | grep sue
    cd ~/gaps/xdcomms-dma/test_app
```

Then we can run the test application 
```
ENCLAVE=orange CONFIG_FILE=xdconf_app_req_rep.json DEV_NAME_RX=sue_donimous_rx1 DEV_NAME_TX=sue_donimous_tx1 LD_LIBRARY_PATH=~/gaps/xdcomms-dma/api ./app_req_rep -v -e 2 -l 1

ENCLAVE=green CONFIG_FILE=xdconf_app_req_rep.json DEV_NAME_RX=sue_donimous_rx0 DEV_NAME_TX=sue_donimous_tx0 LD_LIBRARY_PATH=~/gaps/xdcomms-dma/api ./app_req_rep -v -l 1
```


### MIND DMA (on the XILINX board)
Note that the SERVER (orange on a53) must be started within 3 seconds after starting the CLIENT (green on the microblaze)
```
ENCLAVE=orange CONFIG_FILE=xdconf_app_req_rep.json XDCLOGLEVEL=0 ./app_req_rep -v -l 1 -e 2

ENCLAVE=green  CONFIG_FILE=xdconf_app_req_rep.json XDCLOGLEVEL=0 ./app_req_rep -v -l 1  
```


### ESCAPE SHM (on the ESCAPE board)
```
sudo ENCLAVE=orange CONFIG_FILE=xdconf_app_req_rep.json DEV_TYPE_RX=shm DEV_TYPE_TX=shm SHM_WAIT4NEW=1 LD_LIBRARY_PATH=~/gaps/xdcomms-dma/api ./app_req_rep -e 2 -v -l 1

sudo ENCLAVE=green CONFIG_FILE=xdconf_app_req_rep.json DEV_TYPE_RX=shm DEV_TYPE_TX=shm LD_LIBRARY_PATH=~/gaps/xdcomms-dma/api ./app_req_rep -v -l 1
```


### X-ARBITOR FILE (on a single host)
```
ENCLAVE=orange CONFIG_FILE=xdconf_app_req_rep.json DEV_TYPE_RX=file DEV_TYPE_TX=file LD_LIBRARY_PATH=~/gaps/xdcomms-dma/api DEV_NAME_RX=/tmp/xdc/2 DEV_NAME_TX=/tmp/xdc/1 XARB_IP=1.2.3.4 ./app_req_rep -v -l 1 -e 2

ENCLAVE=green CONFIG_FILE=xdconf_app_req_rep.json DEV_TYPE_RX=file DEV_TYPE_TX=file LD_LIBRARY_PATH=~/gaps/xdcomms-dma/api DEV_NAME_TX=/tmp/xdc/2 DEV_NAME_RX=/tmp/xdc/1 XARB_IP=1.2.3.5 ./app_req_rep -v -l 1
```


## Running the Websrv Application
The websrv application has been tested on all supported xdcomms environments.
Replace the IP addresses for MYADDR and CAMADDR with the correct ones for the experiment setup.

### INTEL ESCAPE SHM 
```
sudo CONFIG_FILE=../xdconf.ini LD_LIBRARY_PATH=~/gaps/xdcomms-dma/api \
ENCLAVE=orange MYADDR=10.109.23.128 CAMADDR=10.109.23.151 SHM_WAIT4NEW=1 \
DEV_TYPE_RX=shm DEV_TYPE_TX=shm XDCLOGLEVEL=0 ./websrv

sudo CONFIG_FILE=../xdconf.ini LD_LIBRARY_PATH=~/gaps/xdcomms-dma/api \
ENCLAVE=green \
DEV_TYPE_RX=shm DEV_TYPE_TX=shm XDCLOGLEVEL=0 ./websrv
```


### PSEUDO DMA (on a single host)
```
CONFIG_FILE=../xdconf.ini LD_LIBRARY_PATH=~/gaps/xdcomms-dma/api \
ENCLAVE=orange MYADDR=10.109.23.128 CAMADDR=10.109.23.151 \
DEV_NAME_RX=sue_donimous_rx1 DEV_NAME_TX=sue_donimous_tx1 XDCLOGLEVEL=0 ./websrv

CONFIG_FILE=../xdconf.ini LD_LIBRARY_PATH=~/gaps/xdcomms-dma/api \
ENCLAVE=green \
DEV_NAME_RX=sue_donimous_rx0 DEV_NAME_TX=sue_donimous_tx0 XDCLOGLEVEL=0 ./websrv
```


### GD MIND DMA 
``` 
XDCLOGLEVEL=0 MYADDR=10.109.23.247 CAMADDR=10.109.23.151 ENCLAVE=orange CONFIG_FILE=xdconf.ini ./websrv-vid

XDCLOGLEVEL=0 ENCLAVE=green CONFIG_FILE=xdconf.ini ./websrv-web
```
