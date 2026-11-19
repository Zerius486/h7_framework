#include "arm_math.h"
#include "stm32h7xx_hal.h"

#include <math.h>
#include <string.h>

#define HOST_MAX_MAT_DIM 12U
#define HOST_UART_TX_CAP 512U

static uint32_t s_host_tick_ms = 0U;
static uint32_t s_host_irq_lock_count = 0U;
static uint8_t s_host_last_uart_tx[HOST_UART_TX_CAP];
static uint16_t s_host_last_uart_tx_len = 0U;

uint32_t HAL_GetTick(void)
{
  return s_host_tick_ms;
}

void HAL_Delay(uint32_t delay_ms)
{
  s_host_tick_ms += delay_ms;
}

void HostResetEnv(void)
{
  s_host_tick_ms = 0U;
  s_host_irq_lock_count = 0U;
  s_host_last_uart_tx_len = 0U;
  memset(s_host_last_uart_tx, 0, sizeof(s_host_last_uart_tx));
}

void HostSetTick(uint32_t tick_ms)
{
  s_host_tick_ms = tick_ms;
}

void HostAdvanceTick(uint32_t delta_ms)
{
  s_host_tick_ms += delta_ms;
}

uint32_t HostGetIrqLockCount(void)
{
  return s_host_irq_lock_count;
}

void HostIncIrqLockCount(void)
{
  s_host_irq_lock_count++;
}

void HostGetLastUartTx(const uint8_t **data, uint16_t *len)
{
  if (data != NULL)
  {
    *data = s_host_last_uart_tx;
  }
  if (len != NULL)
  {
    *len = s_host_last_uart_tx_len;
  }
}

void arm_mat_init_f32(arm_matrix_instance_f32 *S, uint16_t nRows,
                      uint16_t nColumns, float32_t *pData)
{
  if (S == NULL)
  {
    return;
  }
  S->numRows = nRows;
  S->numCols = nColumns;
  S->pData = pData;
}

arm_status arm_mat_add_f32(const arm_matrix_instance_f32 *pSrcA,
                           const arm_matrix_instance_f32 *pSrcB,
                           arm_matrix_instance_f32 *pDst)
{
  if (pSrcA == NULL || pSrcB == NULL || pDst == NULL ||
      pSrcA->pData == NULL || pSrcB->pData == NULL || pDst->pData == NULL)
  {
    return ARM_MATH_ARGUMENT_ERROR;
  }
  if (pSrcA->numRows != pSrcB->numRows || pSrcA->numCols != pSrcB->numCols ||
      pSrcA->numRows != pDst->numRows || pSrcA->numCols != pDst->numCols)
  {
    return ARM_MATH_SIZE_MISMATCH;
  }
  uint32_t count = (uint32_t)pSrcA->numRows * (uint32_t)pSrcA->numCols;
  for (uint32_t i = 0U; i < count; ++i)
  {
    pDst->pData[i] = pSrcA->pData[i] + pSrcB->pData[i];
  }
  return ARM_MATH_SUCCESS;
}

arm_status arm_mat_sub_f32(const arm_matrix_instance_f32 *pSrcA,
                           const arm_matrix_instance_f32 *pSrcB,
                           arm_matrix_instance_f32 *pDst)
{
  if (pSrcA == NULL || pSrcB == NULL || pDst == NULL ||
      pSrcA->pData == NULL || pSrcB->pData == NULL || pDst->pData == NULL)
  {
    return ARM_MATH_ARGUMENT_ERROR;
  }
  if (pSrcA->numRows != pSrcB->numRows || pSrcA->numCols != pSrcB->numCols ||
      pSrcA->numRows != pDst->numRows || pSrcA->numCols != pDst->numCols)
  {
    return ARM_MATH_SIZE_MISMATCH;
  }
  uint32_t count = (uint32_t)pSrcA->numRows * (uint32_t)pSrcA->numCols;
  for (uint32_t i = 0U; i < count; ++i)
  {
    pDst->pData[i] = pSrcA->pData[i] - pSrcB->pData[i];
  }
  return ARM_MATH_SUCCESS;
}

arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 *pSrcA,
                            const arm_matrix_instance_f32 *pSrcB,
                            arm_matrix_instance_f32 *pDst)
{
  if (pSrcA == NULL || pSrcB == NULL || pDst == NULL ||
      pSrcA->pData == NULL || pSrcB->pData == NULL || pDst->pData == NULL)
  {
    return ARM_MATH_ARGUMENT_ERROR;
  }
  if (pSrcA->numCols != pSrcB->numRows || pSrcA->numRows != pDst->numRows ||
      pSrcB->numCols != pDst->numCols)
  {
    return ARM_MATH_SIZE_MISMATCH;
  }
  uint16_t rows = pSrcA->numRows;
  uint16_t inner = pSrcA->numCols;
  uint16_t cols = pSrcB->numCols;
  if (rows > HOST_MAX_MAT_DIM || inner > HOST_MAX_MAT_DIM ||
      cols > HOST_MAX_MAT_DIM)
  {
    return ARM_MATH_LENGTH_ERROR;
  }

  float32_t tmp[HOST_MAX_MAT_DIM * HOST_MAX_MAT_DIM];
  for (uint16_t r = 0U; r < rows; ++r)
  {
    for (uint16_t c = 0U; c < cols; ++c)
    {
      float32_t sum = 0.0f;
      for (uint16_t k = 0U; k < inner; ++k)
      {
        sum += pSrcA->pData[r * inner + k] * pSrcB->pData[k * cols + c];
      }
      tmp[r * cols + c] = sum;
    }
  }
  memcpy(pDst->pData, tmp, (size_t)rows * cols * sizeof(float32_t));
  return ARM_MATH_SUCCESS;
}

arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 *pSrc,
                             arm_matrix_instance_f32 *pDst)
{
  if (pSrc == NULL || pDst == NULL || pSrc->pData == NULL ||
      pDst->pData == NULL)
  {
    return ARM_MATH_ARGUMENT_ERROR;
  }
  if (pSrc->numRows != pDst->numCols || pSrc->numCols != pDst->numRows)
  {
    return ARM_MATH_SIZE_MISMATCH;
  }
  uint16_t rows = pSrc->numRows;
  uint16_t cols = pSrc->numCols;
  if (rows > HOST_MAX_MAT_DIM || cols > HOST_MAX_MAT_DIM)
  {
    return ARM_MATH_LENGTH_ERROR;
  }

  float32_t tmp[HOST_MAX_MAT_DIM * HOST_MAX_MAT_DIM];
  for (uint16_t r = 0U; r < rows; ++r)
  {
    for (uint16_t c = 0U; c < cols; ++c)
    {
      tmp[c * rows + r] = pSrc->pData[r * cols + c];
    }
  }
  memcpy(pDst->pData, tmp, (size_t)rows * cols * sizeof(float32_t));
  return ARM_MATH_SUCCESS;
}

arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32 *pSrc,
                               arm_matrix_instance_f32 *pDst)
{
  if (pSrc == NULL || pDst == NULL || pSrc->pData == NULL ||
      pDst->pData == NULL)
  {
    return ARM_MATH_ARGUMENT_ERROR;
  }
  if (pSrc->numRows != pSrc->numCols || pSrc->numRows != pDst->numRows ||
      pSrc->numCols != pDst->numCols)
  {
    return ARM_MATH_SIZE_MISMATCH;
  }
  uint16_t n = pSrc->numRows;
  if (n == 0U || n > HOST_MAX_MAT_DIM)
  {
    return ARM_MATH_LENGTH_ERROR;
  }

  float32_t aug[HOST_MAX_MAT_DIM][2U * HOST_MAX_MAT_DIM];
  for (uint16_t r = 0U; r < n; ++r)
  {
    for (uint16_t c = 0U; c < n; ++c)
    {
      aug[r][c] = pSrc->pData[r * n + c];
      aug[r][c + n] = (r == c) ? 1.0f : 0.0f;
    }
  }

  for (uint16_t i = 0U; i < n; ++i)
  {
    uint16_t pivot = i;
    float32_t max_abs = fabsf(aug[i][i]);
    for (uint16_t r = (uint16_t)(i + 1U); r < n; ++r)
    {
      float32_t val = fabsf(aug[r][i]);
      if (val > max_abs)
      {
        max_abs = val;
        pivot = r;
      }
    }
    if (!isfinite(max_abs) || max_abs < 1.0e-12f)
    {
      return ARM_MATH_SINGULAR;
    }
    if (pivot != i)
    {
      for (uint16_t c = 0U; c < (uint16_t)(2U * n); ++c)
      {
        float32_t tmp = aug[i][c];
        aug[i][c] = aug[pivot][c];
        aug[pivot][c] = tmp;
      }
    }
    float32_t inv_diag = 1.0f / aug[i][i];
    for (uint16_t c = 0U; c < (uint16_t)(2U * n); ++c)
    {
      aug[i][c] *= inv_diag;
    }
    for (uint16_t r = 0U; r < n; ++r)
    {
      if (r == i)
      {
        continue;
      }
      float32_t factor = aug[r][i];
      for (uint16_t c = 0U; c < (uint16_t)(2U * n); ++c)
      {
        aug[r][c] -= factor * aug[i][c];
      }
    }
  }

  for (uint16_t r = 0U; r < n; ++r)
  {
    for (uint16_t c = 0U; c < n; ++c)
    {
      pDst->pData[r * n + c] = aug[r][c + n];
    }
  }
  return ARM_MATH_SUCCESS;
}

typedef void (*HostUartRxCallback)(uint8_t *rx_buffer, uint16_t rx_length);
typedef void (*HostSpiRxCallback)(uint8_t *rx_buffer, uint16_t rx_length);

__attribute__((weak)) void UartInit(UART_HandleTypeDef *huart,
                                    HostUartRxCallback callback)
{
  (void)huart;
  (void)callback;
}

__attribute__((weak)) bool UartTransmitTry(UART_HandleTypeDef *huart,
                                           uint8_t *tx_data, uint16_t tx_length)
{
  (void)huart;
  if (tx_data == NULL || tx_length == 0U || tx_length > HOST_UART_TX_CAP)
  {
    return false;
  }
  memcpy(s_host_last_uart_tx, tx_data, tx_length);
  s_host_last_uart_tx_len = tx_length;
  return true;
}

__attribute__((weak)) void UartTransmit(UART_HandleTypeDef *huart,
                                        uint8_t *tx_data, uint16_t tx_length)
{
  (void)UartTransmitTry(huart, tx_data, tx_length);
}

__attribute__((weak)) bool UartTransmitReady(UART_HandleTypeDef *huart)
{
  (void)huart;
  return true;
}

__attribute__((weak)) void SpiInit(SPI_HandleTypeDef *hspi,
                                   HostSpiRxCallback callback)
{
  (void)hspi;
  (void)callback;
}

__attribute__((weak)) void SpiTransmitReceive(SPI_HandleTypeDef *hspi,
                                              uint8_t *tx_data,
                                              uint8_t *rx_data, uint16_t length)
{
  (void)hspi;
  (void)tx_data;
  (void)rx_data;
  (void)length;
}

__attribute__((weak)) HAL_StatusTypeDef SpiTransmitReceiveBlocking(
    SPI_HandleTypeDef *hspi, uint8_t *tx_data, uint8_t *rx_data,
    uint16_t length, uint32_t timeout)
{
  (void)hspi;
  (void)tx_data;
  (void)rx_data;
  (void)length;
  (void)timeout;
  return HAL_OK;
}
