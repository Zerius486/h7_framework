#ifndef BSP_FDCAN_H
#define BSP_FDCAN_H

#include "stm32h7xx_hal.h"

// FDCAN外设数量
enum { kFdcanNbr = 3 };

// FDCAN接收回调函数指针
typedef void (*FdcanRxCallback)(uint32_t std_id, uint8_t *rx_buffer);

// FDCAN对象结构体
typedef struct {
  FDCAN_HandleTypeDef *hfdcan;      // FDCAN句柄
  FDCAN_RxHeaderTypeDef rx_header;  // 接收帧头
  FDCAN_TxHeaderTypeDef tx_header;  // 发送帧头模板
  uint8_t rx_buffer[8];             // 接收缓冲区
  uint8_t fifo_active;              // 当前激活的FIFO
  FdcanRxCallback callback;         // 接收回调函数指针
} FdcanObject;

extern FdcanObject g_fdcan_object[kFdcanNbr];

void FdcanInit(FDCAN_HandleTypeDef *hfdcan, FdcanRxCallback callback);
void FdcanTransmit(FDCAN_HandleTypeDef *hfdcan, uint8_t *tx_data,
                   uint32_t tx_length, uint32_t std_id);

#endif  // BSP_FDCAN_H
