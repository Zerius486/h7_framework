#include "alg_td.h"
/**
 * @brief 初始化微分追踪器
 * @param td 微分追踪器结构体指针
 * @param r 跟踪速度因子
 */
void TdInit(TdObject *td, float r) {
  td->x1 = 0.0f;
  td->x2 = 0.0f;
  td->r = r;
}
/**
 * @brief 更新微分追踪器
 * @param td 微分追踪器结构体指针
 * @param input 输入信号
 * @param dt 时间步长
 */
void TdUpdate(TdObject *td, float input, float dt) {
  float x1_dot = td->x2;
  float x2_dot = -td->r * td->r * (td->x1 - input) - 2 * td->r * td->x2;
  td->x1 += x1_dot * dt;
  td->x2 += x2_dot * dt;
}