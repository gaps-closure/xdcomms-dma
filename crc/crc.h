/* CRC-16-CCITT with generator polynomial: x^16 + x^12 + x^5 + 1 */

#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <assert.h>

#define _CRC_P             0x8408   /* CRC-16-CCITT polynomial reversed */
#define _CRC_PPPINITFCS16  0xffff   /* Initial CRC value */

uint16_t crc16(uint8_t *, size_t);
