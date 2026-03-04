#ifndef __CRC_H__
#define __CRC_H__
#include "main.h"

unsigned char calc_crc8(unsigned char *pchMessage, unsigned int dwLength);
uint16_t calc_crc16(uint8_t *pchMessage, uint32_t dwLength);

#endif
