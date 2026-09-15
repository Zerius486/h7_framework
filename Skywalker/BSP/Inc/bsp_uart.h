#ifndef BSP_UART_H
#define BSP_UART_H

#include "stm32h7xx_hal.h"

// UART缓冲区大小和外设数量
enum {
  kUartBufferSize = 512,
  kUartNbr = 6,
};

// UART接收回调函数指针
typedef void (*UartRxCallback)(uint8_t *rx_buffer, uint16_t rx_length);

// UART对象结构体
typedef struct {
  UART_HandleTypeDef *huart;             // UART句柄
  uint8_t rx_buffer_0[kUartBufferSize];  // 接收缓冲区0
  uint8_t rx_buffer_1[kUartBufferSize];  // 接收缓冲区1
  uint8_t *rx_buffer_active;             // 当前接收缓冲区
  uint8_t *rx_buffer_ready;              // 就绪接收缓冲区
  uint8_t tx_buffer[kUartBufferSize];    // 发送缓冲区
  UartRxCallback callback;               // 接收回调函数指针
} UartObject;

extern UartObject g_uart_object[kUartNbr];

uint8_t UartIndex(UART_HandleTypeDef *huart);
void UartInit(UART_HandleTypeDef *huart, UartRxCallback callback);
void UartTransmit(UART_HandleTypeDef *huart, uint8_t *tx_data,
                  uint16_t tx_length);

#endif  // BSP_UART_H
