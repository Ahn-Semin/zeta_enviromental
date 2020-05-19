// ref: https://dev.re.kr/56, 2020.04.09
#ifndef _CRC16_MODBUS_H__
#define _CRC16_MODBUS_H__

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

int CRC16_MODBUS (const uint8_t *nData, uint16_t wLength);

#endif
//EOF