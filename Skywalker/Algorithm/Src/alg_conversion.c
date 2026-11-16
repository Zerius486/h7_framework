#include "alg_conversion.h"

#include "math.h"
#include "stdint.h"
static const float kTwoPiF = 2.0f * (float)M_PI;
static const float kRpmToRadS = (float)M_PI / 30.0f;
static const float kRadSToRpm = 30.0f / (float)M_PI;
static const float kRadToDeg = 180.0f / (float)M_PI;
static const float kDegToRad = (float)M_PI / 180.0f;

/**
 * @brief 将转速从转每分钟转换为弧度每秒
 * @param rpm 转速（r/min）
 * @return 角速度（rad/s）
 */
float RpmToRadS(float rpm)
{
  return rpm * kRpmToRadS;
}
/**
 * @brief 将角速度从弧度每秒转换为转每分钟
 * @param rad_s 角速度（rad/s）
 * @return 转速（r/min）
 */
float RadSToRpm(float rad_s)
{
  return rad_s * kRadSToRpm;
}
/**
 * @brief 将编码器值转换为弧度
 * @param encoder_value 编码器原始值
 * @return 角度（rad）
 */
float EncoderToRadian(uint16_t encoder_value)
{
  return (float)(encoder_value & 0x1FFFU) * (kTwoPiF / 8192.0f);
}
/**
 * @brief 将弧度转换为编码器值
 * @param radian 角度（rad）
 * @return 编码器值
 */
uint16_t RadianToEncoder(float radian)
{
  if (!isfinite(radian))
  {
    return 0U;
  }
  float wrapped = MathWrap2Pi(radian);
  return (uint16_t)(wrapped * (8192.0f / kTwoPiF)) & 0x1FFFU;
}
/**
 * @brief 将弧度转换为角度
 * @param radian 角度（rad）
 * @return 角度（°）
 */
float RadianToDegree(float radian)
{
  return radian * kRadToDeg;
}
/**
 * @brief 将角度转换为弧度
 * @param degree 角度（°）
 * @return 角度（rad）
 */
float DegreeToRadian(float degree)
{
  return degree * kDegToRad;
}
/**
 * @brief 将编码器值转换为角度
 * @param encoder_value 编码器原始值
 * @return 角度（°）
 */
float EncoderToDegree(uint16_t encoder_value)
{
  return (float)(encoder_value & 0x1FFFU) * (360.0f / 8192.0f);
}
/**
 * @brief 将角度转换为编码器值
 * @param degree 角度（°）
 * @return 编码器值
 */
uint16_t DegreeToEncoder(float degree)
{
  if (!isfinite(degree))
  {
    return 0U;
  }
  float wrapped = fmodf(degree, 360.0f);
  if (wrapped < 0.0f)
  {
    wrapped += 360.0f;
  }
  return (uint16_t)(wrapped * (8192.0f / 360.0f)) & 0x1FFFU;
}
