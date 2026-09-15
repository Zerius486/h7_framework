#ifndef ALG_CRC_H
#define ALG_CRC_H
#include "stdbool.h"
#include "stdint.h"
// CRC8校验表
extern uint8_t crc8_init;
extern uint8_t crc8_table[256];
// CRC16校验表
extern uint16_t crc16_init;
extern uint16_t crc16_table[256];
uint8_t GetCrc8(uint8_t *data, uint16_t len);
uint16_t GetCrc16(uint8_t *data, uint16_t len);
bool CheckCrc8(uint8_t *data, uint16_t len);
bool CheckCrc16(uint8_t *data, uint16_t len);
#endif  // ALG_CRC_H