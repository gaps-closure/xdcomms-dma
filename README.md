# XDCOMMS Library

Partitioned application programs use the CLOSURE Cross-Domain Communication (XDCOMMS) library to communicate data through a Cross-Domain Guard (CDG). 


## XDCOMMS API

XDCOMMS-lib maintains the same API as the Hardware Abstraction Layer (HAL), using HAL tags to identify the abstract communication channels with three orthogonal 32-bit unsigned identifiers:
- **mux** is a session multiplexing handle used to identify a unidirectional application flow.
- **sec** identifies a CDG security policy used to processing an ADU. 
- **typ** identifies the type of ADU (based on DFDL xsd definition). 

XDCOMMS-lib HAL API function calls: 
- void  xdc_log_level(int new_level);
- void  xdc_register(codec_func_ptr encode, codec_func_ptr decode, int typ);
- void *xdc_sub_socket_non_blocking(gaps_tag tag, int timeout);
- void  xdc_asyn_send(void *socket, void *adu, gaps_tag *tag);
- int   xdc_recv(void *socket, void *adu, gaps_tag *tag);
- void  xdc_blocking_recv(void *socket, void *adu, gaps_tag *tag);


## XDCOMMS SUPPORTED Cross-Domain Guards

XDCOMMS-lib current version (version 0.5) supports three main types of CDG:
- GE-MIND: Communicating with the proxy DMA driver using IOCTL commands, which in turn communicates with the XILINX AXI DMA/MCDMA driver on the GE MIND ZCU102 FPGA board.
- INTEL-ESCAPE: Copying data to and from shared memory mapped regions on the ESCAPE FPGA board. 
- X-ARBITOR: Reading a writing files with the X-ARBITOR send and receive proxies.

In addition XDCOMMS-lib can be tested on a single host computer using: 
- Pseudo driver emulation (as stand-in for the proxy DMA driver)
- Shared memory mapped regions of the host computer (instead of the ESCAPE FPGA memory). 
- Files in directories on the host computer 


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
