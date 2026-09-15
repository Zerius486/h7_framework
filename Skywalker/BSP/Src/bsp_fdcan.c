#include "bsp_fdcan.h"

#include <stdint.h>
#include <string.h>

#include "fdcan.h"
#include "stm32h7xx_hal.h"

// FDCAN对象实例
FdcanObject g_fdcan_object[kFdcanNbr] = {0};

/**
 * @brief 获取FDCAN对象实例索引
 * @param hfdcan FDCAN句柄
 * @return FDCAN对象实例索引，0xFF表示无效索引
 */
static uint8_t FdcanIndex(FDCAN_HandleTypeDef *hfdcan) {
  if (hfdcan == &hfdcan1) {
    return 0;
  }
  if (hfdcan == &hfdcan2) {
    return 1;
  }
  if (hfdcan == &hfdcan3) {
    return 2;
  }
  return 0xFF;
}

static uint32_t FdcanLengthToDlc(uint32_t length) {
  static const uint32_t kClassicCanDlc[] = {
      FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
      FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
      FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8,
  };

  if (length > 8U) {
    length = 8U;
  }
  return kClassicCanDlc[length];
}

/**
 * @brief 配置FDCAN过滤器，接收所有标准帧
 * @param hfdcan FDCAN句柄
 */
static void FdcanFilterConfigAll(FDCAN_HandleTypeDef *hfdcan) {
  uint32_t index = FdcanIndex(hfdcan);
  if (index == 0xFFU) {
    return;
  }

  FDCAN_FilterTypeDef filter_config = {0};
  filter_config.IdType = FDCAN_STANDARD_ID;
  filter_config.FilterIndex = 0;
  filter_config.FilterType = FDCAN_FILTER_MASK;
  filter_config.FilterConfig = (g_fdcan_object[index].fifo_active == 1U)
                                   ? FDCAN_FILTER_TO_RXFIFO1
                                   : FDCAN_FILTER_TO_RXFIFO0;
  filter_config.FilterID1 = 0x0000U;
  filter_config.FilterID2 = 0x0000U;
  (void)HAL_FDCAN_ConfigFilter(hfdcan, &filter_config);
}

/**
 * @brief 配置全局过滤器
 * @param hfdcan FDCAN句柄
 */
static void FdcanGlobalFilterConfig(FDCAN_HandleTypeDef *hfdcan) {
  (void)HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
}

/**
 * @brief 初始化FDCAN对象实例
 * @param hfdcan FDCAN句柄
 * @param callback 接收回调函数指针
 */
void FdcanInit(FDCAN_HandleTypeDef *hfdcan, FdcanRxCallback callback) {
  uint32_t index = FdcanIndex(hfdcan);
  if (index == 0xFFU) {
    return;
  }

  g_fdcan_object[index].hfdcan = hfdcan;
  g_fdcan_object[index].tx_header.IdType = FDCAN_STANDARD_ID;
  g_fdcan_object[index].tx_header.TxFrameType = FDCAN_DATA_FRAME;
  g_fdcan_object[index].tx_header.DataLength = FDCAN_DLC_BYTES_8;
  g_fdcan_object[index].tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  g_fdcan_object[index].tx_header.BitRateSwitch = FDCAN_BRS_OFF;
  g_fdcan_object[index].tx_header.FDFormat = FDCAN_CLASSIC_CAN;
  g_fdcan_object[index].tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  g_fdcan_object[index].tx_header.MessageMarker = 0;
  g_fdcan_object[index].callback = callback;
  g_fdcan_object[index].fifo_active = 0;

  FdcanFilterConfigAll(hfdcan);
  FdcanGlobalFilterConfig(hfdcan);

  uint32_t rx_notification = (g_fdcan_object[index].fifo_active == 1U)
                                 ? FDCAN_IT_RX_FIFO1_NEW_MESSAGE
                                 : FDCAN_IT_RX_FIFO0_NEW_MESSAGE;
  (void)HAL_FDCAN_ActivateNotification(hfdcan, rx_notification, 0);
  (void)HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_BUS_OFF, 0);
  (void)HAL_FDCAN_Start(hfdcan);
}

/**
 * @brief 发送FDCAN数据帧
 * @param hfdcan FDCAN句柄
 * @param tx_data 发送数据指针
 * @param tx_length 发送数据长度
 * @param std_id 标准ID
 */
void FdcanTransmit(FDCAN_HandleTypeDef *hfdcan, uint8_t *tx_data,
                   uint32_t tx_length, uint32_t std_id) {
  uint32_t index = FdcanIndex(hfdcan);
  if (index == 0xFFU || (tx_data == NULL && tx_length > 0U)) {
    return;
  }

  if (tx_length > 8U) {
    tx_length = 8U;
  }
  uint8_t tx_buffer[8] = {0};
  if (tx_length > 0U) {
    memcpy(tx_buffer, tx_data, tx_length);
  }

  FDCAN_TxHeaderTypeDef tx_header = g_fdcan_object[index].tx_header;
  tx_header.Identifier = std_id;
  tx_header.DataLength = FdcanLengthToDlc(tx_length);

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  (void)HAL_FDCAN_AddMessageToTxFifoQ(g_fdcan_object[index].hfdcan,
                                      &tx_header, tx_buffer);
  if (primask == 0U) {
    __enable_irq();
  }
}

/**
 * @brief HAL库 FDCAN FIFO0接收回调函数
 * @param hfdcan FDCAN句柄
 * @param RxFifo0ITs FIFO0中断标志
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) {
    return;
  }

  uint32_t index = FdcanIndex(hfdcan);
  if (index == 0xFFU) {
    return;
  }

  while (HAL_FDCAN_GetRxMessage(g_fdcan_object[index].hfdcan, FDCAN_RX_FIFO0,
                                &g_fdcan_object[index].rx_header,
                                g_fdcan_object[index].rx_buffer) == HAL_OK) {
    if (g_fdcan_object[index].callback != NULL) {
      g_fdcan_object[index].callback(g_fdcan_object[index].rx_header.Identifier,
                                     g_fdcan_object[index].rx_buffer);
    }
  }
}

/**
 * @brief HAL库 FDCAN FIFO1接收回调函数
 * @param hfdcan FDCAN句柄
 * @param RxFifo1ITs FIFO1中断标志
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo1ITs) {
  if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == 0U) {
    return;
  }

  uint32_t index = FdcanIndex(hfdcan);
  if (index == 0xFFU) {
    return;
  }

  while (HAL_FDCAN_GetRxMessage(g_fdcan_object[index].hfdcan, FDCAN_RX_FIFO1,
                                &g_fdcan_object[index].rx_header,
                                g_fdcan_object[index].rx_buffer) == HAL_OK) {
    if (g_fdcan_object[index].callback != NULL) {
      g_fdcan_object[index].callback(g_fdcan_object[index].rx_header.Identifier,
                                     g_fdcan_object[index].rx_buffer);
    }
  }
}

/**
 * @brief HAL库 FDCAN错误状态回调函数
 * @param hfdcan FDCAN句柄
 * @param ErrorStatusITs 错误状态中断标志
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t ErrorStatusITs) {
  if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0U) {
    (void)HAL_FDCAN_Stop(hfdcan);
    (void)HAL_FDCAN_Start(hfdcan);
  }
}
