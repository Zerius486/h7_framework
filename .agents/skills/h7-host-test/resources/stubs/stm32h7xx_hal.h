#ifndef STM32H7XX_HAL_H
#define STM32H7XX_HAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum
{
  HAL_OK = 0x00U,
  HAL_ERROR = 0x01U,
  HAL_BUSY = 0x02U,
  HAL_TIMEOUT = 0x03U,
} HAL_StatusTypeDef;

typedef struct
{
  uint32_t Instance;
} UART_HandleTypeDef;

typedef struct
{
  uint32_t Instance;
} SPI_HandleTypeDef;

typedef struct
{
  uint32_t Instance;
} TIM_HandleTypeDef;

typedef struct
{
  uint32_t Instance;
} FDCAN_HandleTypeDef;

typedef struct
{
  uint32_t Instance;
} IWDG_HandleTypeDef;

uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t delay_ms);

void HostResetEnv(void);
void HostSetTick(uint32_t tick_ms);
void HostAdvanceTick(uint32_t delta_ms);
uint32_t HostGetIrqLockCount(void);
void HostGetLastUartTx(const uint8_t **data, uint16_t *len);

void HostIncIrqLockCount(void);

static inline uint32_t __get_PRIMASK(void)
{
  return 0U;
}

static inline void __disable_irq(void)
{
  HostIncIrqLockCount();
}

static inline void __enable_irq(void)
{
}

static inline void __DMB(void)
{
}

#endif // STM32H7XX_HAL_H
