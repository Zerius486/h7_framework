#include "bsp_spi.h"

#include <stdint.h>
#include <string.h>

#include "spi.h"
#include "stm32h7xx_hal.h"

// SPI对象实例
SpiObject g_spi_object[kSpiNbr] = {0};

/**
 * @brief 获取SPI对象实例索引
 * @param hspi SPI句柄
 * @return SPI对象实例索引，0xFF表示无效索引
 */
static uint8_t SpiIndex(SPI_HandleTypeDef *hspi) {
  if (hspi == &hspi2) {
    return 0;
  }
  return 0xFF;
}

/**
 * @brief 初始化SPI
 * @param hspi SPI句柄
 * @param callback 接收回调函数指针
 */
void SpiInit(SPI_HandleTypeDef *hspi, SpiRxCallback callback) {
  uint32_t index = SpiIndex(hspi);
  if (index == 0xFFU) {
    return;
  }

  g_spi_object[index].hspi = hspi;
  g_spi_object[index].rx_buffer_user = NULL;
  g_spi_object[index].transfer_length = 0;
  g_spi_object[index].callback = callback;
}

/**
 * @brief SPI同时发送和接收数据
 * @param hspi SPI句柄
 * @param tx_data 发送数据指针
 * @param rx_data 接收数据指针 (可以为NULL，如果不需要接收)
 * @param length 数据长度
 */
void SpiTransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *tx_data,
                        uint8_t *rx_data, uint16_t length) {
  uint32_t index = SpiIndex(hspi);
  if (index == 0xFFU || tx_data == NULL || length == 0U ||
      length > kSpiBufferSize || hspi->State != HAL_SPI_STATE_READY) {
    return;
  }

  memcpy(g_spi_object[index].tx_buffer, tx_data, length);
  g_spi_object[index].rx_buffer_user = rx_data;
  g_spi_object[index].transfer_length = length;
  (void)HAL_SPI_TransmitReceive_DMA(hspi, g_spi_object[index].tx_buffer,
                                    g_spi_object[index].rx_buffer, length);
}

HAL_StatusTypeDef SpiTransmitReceiveBlocking(SPI_HandleTypeDef *hspi,
                                             uint8_t *tx_data, uint8_t *rx_data,
                                             uint16_t length,
                                             uint32_t timeout) {
  uint32_t index = SpiIndex(hspi);
  if (index == 0xFFU || tx_data == NULL || rx_data == NULL || length == 0U ||
      length > kSpiBufferSize) {
    return HAL_ERROR;
  }

  return HAL_SPI_TransmitReceive(hspi, tx_data, rx_data, length, timeout);
}

/**
 * @brief HAL库 SPI传输完成回调函数
 * @param hspi SPI句柄
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  uint32_t index = SpiIndex(hspi);
  if (index == 0xFFU) {
    return;
  }

  uint16_t transfer_length = g_spi_object[index].transfer_length;
  if (g_spi_object[index].rx_buffer_user != NULL && transfer_length > 0U) {
    memcpy(g_spi_object[index].rx_buffer_user, g_spi_object[index].rx_buffer,
           transfer_length);
  }

  if (g_spi_object[index].callback != NULL) {
    g_spi_object[index].callback(g_spi_object[index].rx_buffer,
                                 transfer_length);
  }

  g_spi_object[index].rx_buffer_user = NULL;
  g_spi_object[index].transfer_length = 0;
}

/**
 * @brief HAL库 SPI错误回调函数
 * @param hspi SPI句柄
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
  uint32_t index = SpiIndex(hspi);
  if (index == 0xFFU) {
    return;
  }

  g_spi_object[index].rx_buffer_user = NULL;
  g_spi_object[index].transfer_length = 0;
  (void)HAL_SPI_Abort(hspi);
}
