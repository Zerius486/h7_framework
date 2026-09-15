#ifndef BSP_SPI_H
#define BSP_SPI_H

#include "stm32h7xx_hal.h"

// SPI缓冲区大小和外设数量
enum {
  kSpiBufferSize = 512,
  kSpiNbr = 1,
};

// SPI接收回调函数指针
typedef void (*SpiRxCallback)(uint8_t *rx_buffer, uint16_t rx_length);

// SPI对象结构体
typedef struct {
  SPI_HandleTypeDef *hspi;            // SPI句柄
  uint8_t rx_buffer[kSpiBufferSize];  // 接收缓冲区
  uint8_t tx_buffer[kSpiBufferSize];  // 发送缓冲区
  uint8_t *rx_buffer_user;            // 用户接收缓冲区
  uint16_t transfer_length;           // 当前传输长度
  SpiRxCallback callback;             // 接收回调函数指针
} SpiObject;

extern SpiObject g_spi_object[kSpiNbr];

void SpiInit(SPI_HandleTypeDef *hspi, SpiRxCallback callback);
void SpiTransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *tx_data,
                        uint8_t *rx_data, uint16_t length);
HAL_StatusTypeDef SpiTransmitReceiveBlocking(SPI_HandleTypeDef *hspi,
                                             uint8_t *tx_data, uint8_t *rx_data,
                                             uint16_t length, uint32_t timeout);

#endif  // BSP_SPI_H
