#ifndef _FILE_INFO_H_
#define _FILE_INFO_H_

#define FILE_MAX_BYTES                (0x10000 / sizeof(uint8_t))  // 64KB
#define FILE_COUNT                    32
#define FILE_DIR_PATH_DEFAULT         "/tmp/xdc"
#define FILE_DIR_SHARE                1   // Flows share directory? 0=no, 1=yes
#define XARBITOR_SEND_SCRIPT_FILENAME "./send-main"
#define XARBITOR_SEND_SCRIPT_ARGS     " -h 192.168.100.101 -g 1124 -p /home/closure/XA-FILES/xarbitor/api/test_apps/etc/pki_auth -c /home/closure/XA-FILES/xarbitor/api/test_apps/etc/credential/xarbitor_temporary_1.0.0.0_client.p12 --password=password -d "

typedef struct _file_channel {
  int     pkt_index_last;
  int     pkt_index_next;
  uint8_t pkt_buffer[FILE_MAX_BYTES];

} file_channel;

#endif /* _FILE_INFO_H_ */
