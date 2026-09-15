#include "bsp_uart.h"

#include <stdint.h>
#include <string.h>

#include "stm32h7xx_hal.h"
#include "usart.h"

// H7 DMA无法访问DTCM，将UART DMA缓冲区放到DMA可访问SRAM。
UartObject g_uart_object[kUartNbr] __attribute__((section(".dma_buffer"),
                                                 aligned(32)));

/**
 * @brief 获取UART对象实例索引
 * @param huart UART句柄
 * @return UART对象实例索引，0xFF表示无效索引
 */
uint8_t UartIndex(UART_HandleTypeDef *huart) {
  if (huart == &huart1) {
    return 0;
  }
  if (huart == &huart2) {
    return 1;
  }
  if (huart == &huart3) {
    return 2;
  }
  if (huart == &huart5) {
    return 3;
  }
  if (huart == &huart7) {
    return 4;
  }
  if (huart == &huart10) {
    return 5;
  }
  return 0xFF;
}

static void UartDisableHalfTransferIt(UART_HandleTypeDef *huart) {
  if (huart != NULL && huart->hdmarx != NULL) {
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
  }
}

static void UartStartRxDma(uint32_t index) {
  UART_HandleTypeDef *huart = g_uart_object[index].huart;
  if (huart == NULL || g_uart_object[index].rx_buffer_active == NULL) {
    return;
  }

  (void)HAL_UARTEx_ReceiveToIdle_DMA(
      huart, g_uart_object[index].rx_buffer_active, kUartBufferSize);
  UartDisableHalfTransferIt(huart);
}

/**
 * @brief 初始化UART
 * @param huart UART句柄
 * @param callback 接收回调函数指针
 */
void UartInit(UART_HandleTypeDef *huart, UartRxCallback callback) {
  uint32_t index = UartIndex(huart);
  if (index == 0xFFU) {
    return;
  }

  memset(&g_uart_object[index], 0, sizeof(g_uart_object[index]));
  g_uart_object[index].huart = huart;
  g_uart_object[index].callback = callback;
  g_uart_object[index].rx_buffer_active = g_uart_object[index].rx_buffer_0;
  g_uart_object[index].rx_buffer_ready = NULL;
  UartStartRxDma(index);
}

/**
 * @brief 发送UART数据帧
 * @param huart UART句柄
 * @param tx_data 发送数据指针
 * @param tx_length 发送长度
 */
void UartTransmit(UART_HandleTypeDef *huart, uint8_t *tx_data,
                  uint16_t tx_length) {
  uint32_t index = UartIndex(huart);
  if (index == 0xFFU || tx_data == NULL || tx_length == 0U ||
      tx_length > kUartBufferSize || huart->gState != HAL_UART_STATE_READY) {
    return;
  }

  memcpy(g_uart_object[index].tx_buffer, tx_data, tx_length);
  (void)HAL_UART_Transmit_DMA(huart, g_uart_object[index].tx_buffer, tx_length);
}

/**
 * @brief HAL库 UART接收事件回调函数
 * @param huart UART句柄
 * @param rx_length 接收长度
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t rx_length) {
  uint32_t index = UartIndex(huart);
  if (index == 0xFFU) {
    return;
  }

  if (rx_length == 0U || rx_length > kUartBufferSize) {
    UartStartRxDma(index);
    return;
  }

  g_uart_object[index].rx_buffer_ready = g_uart_object[index].rx_buffer_active;
  g_uart_object[index].rx_buffer_active =
      (g_uart_object[index].rx_buffer_active ==
       g_uart_object[index].rx_buffer_0)
          ? g_uart_object[index].rx_buffer_1
          : g_uart_object[index].rx_buffer_0;
  UartStartRxDma(index);

  if (g_uart_object[index].callback != NULL) {
    g_uart_object[index].callback(g_uart_object[index].rx_buffer_ready,
                                  rx_length);
  }
}

/**
 * @brief HAL库 UART错误回调函数
 * @param huart UART句柄
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  uint32_t index = UartIndex(huart);
  if (index == 0xFFU) {
    return;
  }

  (void)HAL_UART_AbortReceive(huart);
  UartStartRxDma(index);
}
