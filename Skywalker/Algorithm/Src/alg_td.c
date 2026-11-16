#include "alg_td.h"

#include <math.h>
#include <stddef.h>
/**
 * @brief 初始化微分追踪器
 * @param td 微分追踪器结构体指针
 * @param r 跟踪速度因子
 */
void TdInit(TdObject *td, float r)
{
  if (td == NULL)
  {
    return;
  }
  td->x1 = 0.0f;
  td->x2 = 0.0f;
  td->r = (isfinite(r) && r > 0.0f) ? r : 0.0f;
}
/**
 * @brief 更新微分追踪器
 * @param td 微分追踪器结构体指针
 * @param input 输入信号
 * @param dt 采样时间（s）
 */
void TdUpdate(TdObject *td, float input, float dt)
{
  if (td == NULL || !isfinite(input) || !isfinite(dt) || dt <= 0.0f ||
      !isfinite(td->r) || td->r <= 0.0f)
  {
    return;
  }
  // 限制前向欧拉步长因子 r * dt < 2，防止数值发散
  float effective_r = td->r;
  if (effective_r * dt >= 1.9f)
  {
    effective_r = 1.9f / dt;
  }
  float x1_dot = td->x2;
  float x2_dot =
      -effective_r * effective_r * (td->x1 - input) - 2.0f * effective_r * td->x2;
  td->x1 += x1_dot * dt;
  td->x2 += x2_dot * dt;
}
