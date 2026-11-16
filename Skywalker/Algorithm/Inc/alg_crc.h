#ifndef ALG_CRC_H
#define ALG_CRC_H
#include "stdbool.h"
#include "stdint.h"
// CRC8初始值与查找表
extern const uint8_t crc8_init;
extern const uint8_t crc8_table[256];
// CRC16初始值与查找表
extern const uint16_t crc16_init;
extern const uint16_t crc16_table[256];
uint8_t GetCrc8(const uint8_t *data, uint16_t len);
uint16_t GetCrc16(const uint8_t *data, uint16_t len);
bool CheckCrc8(const uint8_t *data, uint16_t len);
bool CheckCrc16(const uint8_t *data, uint16_t len);
#endif // ALG_CRC_H
