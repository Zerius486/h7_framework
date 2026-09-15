#include "alg_td_nl.h"

#include "math.h"
/**
 * @brief 初始化非线性微分追踪器
 * @param td_nl 非线性微分追踪器结构体指针
 * @param r 速度因子
 * @param h 滤波因子
 */
void TdNlInit(TdNlObject *td_nl, float r, float h) {
  td_nl->x1 = 0.0f;
  td_nl->x2 = 0.0f;
  td_nl->r = r;
  td_nl->h = h;
}
/**
 * @brief 浮点符号函数
 * @param x 输入值
 * @return 符号值
 */
static float SignF(float x) {
  if (x > 0.0f)
    return 1.0f;
  else if (x < 0.0f)
    return -1.0f;
  else
    return 0.0f;
}
/**
 * @brief 最速控制综合函数
 * @param x_1 误差信号
 * @param x_2 微分信号
 * @param r 速度因子
 * @param h 滤波因子
 * @return 函数值
 */
static float Fhan(float x_1, float x_2, float r, float h) {
  float d = r * h;
  float a_0 = r * h * h;
  float y = x_1 + a_0;
  float a_1 = sqrtf(d * (d + 8.0f * fabsf(y)));
  float a_2 = a_0 + SignF(y) * (a_1 - d) / 2.0f;
  float s_y = (SignF(y + d) - SignF(y - d)) / 2.0f;
  float a = (x_2 + y - a_2) * s_y + a_2;
  float s_a = (SignF(a + d) - SignF(a - d)) / 2.0f;
  float Fhan = -r * (a / d - SignF(a)) * s_a - r * SignF(a);
  return Fhan;
}
/**
 * @brief 更新非线性微分追踪器
 * @param td_nl 非线性微分追踪器结构体指针
 * @param input 当前输入值
 * @param dt 时间步长
 */
void TdNlUpdate(TdNlObject *td_nl, float input, float dt) {
  float fh = Fhan(td_nl->x1 - input, td_nl->x2, td_nl->r, td_nl->h);
  float x1_dot = td_nl->x2;
  float x2_dot = -fh;
  td_nl->x1 += x1_dot * dt;
  td_nl->x2 += x2_dot * dt;
}