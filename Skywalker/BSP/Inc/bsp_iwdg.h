#ifndef BSP_IWDG_H
#define BSP_IWDG_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 初始化并启动 STM32H723 独立看门狗 (IWDG1)
 * @param timeout_ms 超时复位时间（ms，建议 100 ~ 4000，基于 LSI=32kHz、32 分频）
 * @return true 初始化成功，false 参数非法或初始化失败
 */
bool BspIwdgInit(uint32_t timeout_ms);

/**
 * @brief 喂狗（重载 IWDG1 计数器）
 */
void BspIwdgFeed(void);

#endif // BSP_IWDG_H
