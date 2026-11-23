#include "bsp_uart.h"

#include <stdint.h>
#include <string.h>

#include "stm32h7xx_hal.h"
#include "usart.h"

// UART对象实例，放置于.dma_buffer段并按32字节对齐，保证Cortex-M7 Cache一致性
UartObject g_uart_object[kUartNbr]
    __attribute__((section(".dma_buffer"), aligned(32))) = {0};

/**
 * @brief 获取UART对象实例索引
 * @param huart UART句柄
 * @return UART对象实例索引，0xFF表示无效索引
 */
uint8_t UartIndex(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    return 0;
  }
  if (huart == &huart2)
  {
    return 1;
  }
  if (huart == &huart3)
  {
    return 2;
  }
  if (huart == &huart5)
  {
    return 3;
  }
  if (huart == &huart7)
  {
    return 4;
  }
  if (huart == &huart10)
  {
    return 5;
  }
  return 0xFF;
}

/**
 * @brief 判断是否为RS485电机总线串口（USART2/USART3）
 * @param huart UART句柄
 * @return true 为RS485串口; false 非RS485串口
 */
bool UartIsRs485(UART_HandleTypeDef *huart)
{
  return huart == &huart2 || huart == &huart3;
}

/**
 * @brief 动态配置UART波特率（非RS485串口适用）
 * @param huart UART句柄
 * @param baudrate 目标波特率（bps）
 * @return true 配置成功; false 参数无效或HAL初始化失败
 */
bool UartSetBaudrate(UART_HandleTypeDef *huart, uint32_t baudrate)
{
  if (huart == NULL || baudrate == 0U || UartIndex(huart) == 0xFFU)
  {
    return false;
  }
  if (huart->Init.BaudRate == baudrate)
  {
    return true;
  }

  (void)HAL_UART_Abort(huart);
  huart->Init.BaudRate = baudrate;
  return HAL_UART_Init(huart) == HAL_OK;
}

static void UartStartRxDma(uint32_t index)
{
  UART_HandleTypeDef *huart = g_uart_object[index].huart;
  if (huart == NULL)
  {
    return;
  }

  g_uart_object[index].rx_old_pos = 0U;
  __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_FEF |
                                   UART_CLEAR_NEF | UART_CLEAR_PEF);
  if (HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart_object[index].rx_buffer,
                                   kUartBufferSize) != HAL_OK)
  {
    (void)HAL_UART_AbortReceive(huart);
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_FEF |
                                     UART_CLEAR_NEF | UART_CLEAR_PEF);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(
        huart, g_uart_object[index].rx_buffer, kUartBufferSize);
  }
}

/**
 * @brief 初始化UART
 * @param huart UART句柄
 * @param callback 接收回调函数指针
 */
void UartInit(UART_HandleTypeDef *huart, UartRxCallback callback)
{
  uint32_t index = UartIndex(huart);
  if (index == 0xFFU)
  {
    return;
  }

  memset(&g_uart_object[index], 0, sizeof(g_uart_object[index]));
  g_uart_object[index].huart = huart;
  g_uart_object[index].callback = callback;
  UartStartRxDma(index);
}

/**
 * @brief 发送UART数据帧
 * @param huart UART句柄
 * @param tx_data 发送数据指针
 * @param tx_length 发送长度
 */
void UartTransmit(UART_HandleTypeDef *huart, uint8_t *tx_data,
                  uint16_t tx_length)
{
  (void)UartTransmitTry(huart, tx_data, tx_length);
}

/**
 * @brief 提交UART DMA发送
 * @param huart UART句柄
 * @param tx_data 发送数据指针
 * @param tx_length 发送长度
 * @return 是否成功提交
 */
bool UartTransmitTry(UART_HandleTypeDef *huart, uint8_t *tx_data,
                     uint16_t tx_length)
{
  uint32_t index = UartIndex(huart);
  if (index == 0xFFU || tx_data == NULL || tx_length == 0U ||
      tx_length > kUartBufferSize)
  {
    return false;
  }

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (!UartTransmitReady(huart))
  {
    if (primask == 0U)
    {
      __enable_irq();
    }
    return false;
  }

  memcpy(g_uart_object[index].tx_buffer, tx_data, tx_length);
  HAL_StatusTypeDef status =
      HAL_UART_Transmit_DMA(huart, g_uart_object[index].tx_buffer, tx_length);
  if (primask == 0U)
  {
    __enable_irq();
  }
  return status == HAL_OK;
}

/**
 * @brief 检查UART DMA发送通道是否空闲
 * @param huart UART句柄
 * @return 是否可立即提交发送
 */
bool UartTransmitReady(UART_HandleTypeDef *huart)
{
  return huart != NULL && UartIndex(huart) != 0xFFU &&
         huart->gState == HAL_UART_STATE_READY && huart->hdmatx != NULL &&
         huart->hdmatx->State == HAL_DMA_STATE_READY;
}

/**
 * @brief HAL库 UART接收事件回调函数（Circular DMA环形增量读取）
 * @param huart UART句柄
 * @param rx_pos 当前DMA写入游标（0..kUartBufferSize）
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t rx_pos)
{
  uint32_t index = UartIndex(huart);
  if (index == 0xFFU || rx_pos > kUartBufferSize)
  {
    return;
  }

  uint16_t old_pos = g_uart_object[index].rx_old_pos;
  if (rx_pos == old_pos)
  {
    return;
  }

  UartRxCallback callback = g_uart_object[index].callback;
  uint8_t *rx_buf = g_uart_object[index].rx_buffer;

  if (rx_pos > old_pos)
  {
    uint16_t len = rx_pos - old_pos;
    g_uart_object[index].rx_old_pos =
        (rx_pos == kUartBufferSize) ? 0U : rx_pos;
    if (callback != NULL)
    {
      callback(&rx_buf[old_pos], len);
    }
  }
  else
  {
    uint16_t tail_len = kUartBufferSize - old_pos;
    g_uart_object[index].rx_old_pos = rx_pos;
    if (callback != NULL)
    {
      callback(&rx_buf[old_pos], tail_len);
      if (rx_pos > 0U)
      {
        callback(&rx_buf[0], rx_pos);
      }
    }
  }
}

/**
 * @brief HAL库 UART错误回调函数
 * @param huart UART句柄
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  uint32_t index = UartIndex(huart);
  if (index == 0xFFU || g_uart_object[index].huart == NULL)
  {
    return;
  }

  (void)HAL_UART_AbortReceive(huart);
  UartStartRxDma(index);
}
