#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdbool.h>

#include "stm32h7xx_hal.h"

// UART缓冲区大小和外设数量
enum
{
  kUartBufferSize = 512,
  kUartNbr = 6,
};

// UART接收回调函数指针
typedef void (*UartRxCallback)(uint8_t *rx_buffer, uint16_t rx_length);

// UART对象结构体（DMA缓冲区置前并按32字节Cache Line对齐）
typedef struct
{
  uint8_t rx_buffer[kUartBufferSize] __attribute__((aligned(32))); // 环形接收缓冲区
  uint8_t tx_buffer[kUartBufferSize] __attribute__((aligned(32))); // 发送缓冲区
  UART_HandleTypeDef *huart;            // UART句柄
  uint16_t rx_old_pos;                  // 环形接收缓冲区已读取游标
  UartRxCallback callback;              // 接收回调函数指针
} UartObject;

extern UartObject g_uart_object[kUartNbr];

uint8_t UartIndex(UART_HandleTypeDef *huart);
bool UartIsRs485(UART_HandleTypeDef *huart);
bool UartSetBaudrate(UART_HandleTypeDef *huart, uint32_t baudrate);
void UartInit(UART_HandleTypeDef *huart, UartRxCallback callback);
void UartTransmit(UART_HandleTypeDef *huart, uint8_t *tx_data,
                  uint16_t tx_length);
bool UartTransmitTry(UART_HandleTypeDef *huart, uint8_t *tx_data,
                     uint16_t tx_length);
bool UartTransmitReady(UART_HandleTypeDef *huart);

#endif // BSP_UART_H
