#ifndef ALG_CONVERSION_H
#define ALG_CONVERSION_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* 通用标量数学工具 ----------------------------------------------------------*/

/**
 * @brief 浮点标量限幅
 * @param value 输入值
 * @param lower 下限
 * @param upper 上限
 * @return 限幅后的值
 */
static inline float MathClamp(float value, float lower, float upper)
{
  return fminf(fmaxf(value, lower), upper);
}

/**
 * @brief 浮点死区滤波
 * @param value 输入值
 * @param threshold 死区半宽阈值（正数）
 * @return 处于死区内返回0，否则原值返回
 */
static inline float MathDeadband(float value, float threshold)
{
  if (fabsf(value) < threshold)
  {
    return 0.0f;
  }
  return value;
}

/**
 * @brief 符号函数
 * @param value 输入值
 * @return 正数返回1.0f，负数返回-1.0f，零返回0.0f
 */
static inline float MathSign(float value)
{
  if (value > 0.0f)
  {
    return 1.0f;
  }
  if (value < 0.0f)
  {
    return -1.0f;
  }
  return 0.0f;
}

/**
 * @brief 将角度归一化至 [-pi, pi] 闭区间
 * @param angle 输入角度（rad）
 * @return 归一化后的角度（rad）
 */
static inline float MathWrapPi(float angle)
{
  return remainderf(angle, 2.0f * (float)M_PI);
}

/**
 * @brief 将角度归一化至 [0, 2*pi) 前闭后开区间
 * @param angle 输入角度（rad）
 * @return 归一化后的角度（rad）
 */
static inline float MathWrap2Pi(float angle)
{
  float wrapped = fmodf(angle, 2.0f * (float)M_PI);
  if (wrapped < 0.0f)
  {
    wrapped += 2.0f * (float)M_PI;
  }
  return wrapped;
}

/**
 * @brief 计算从当前角度到目标角度的最短有向差角（[-pi, pi]）
 * @param target 目标角度（rad）
 * @param current 当前角度（rad）
 * @return 最短旋转差角（rad）
 */
static inline float MathAngleDiff(float target, float current)
{
  return remainderf(target - current, 2.0f * (float)M_PI);
}

/* 常用物理量与编码器转换接口 ------------------------------------------------*/

float RpmToRadS(float rpm);
float RadSToRpm(float rad_s);
float EncoderToRadian(uint16_t encoder_value);
uint16_t RadianToEncoder(float radian);
float RadianToDegree(float radian);
float DegreeToRadian(float degree);
float EncoderToDegree(uint16_t encoder_value);
uint16_t DegreeToEncoder(float degree);

#endif // ALG_CONVERSION_H