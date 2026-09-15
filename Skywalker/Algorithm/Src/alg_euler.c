#include "alg_euler.h"

#include "math.h"
/**
 * @brief 四元数转换为欧拉角
 * @param q 四元数
 * @return 欧拉角
 */
Euler QuaternionToEuler(Quaternion q) {
  Euler euler;
  // 计算roll
  float sr_cp = 2.0f * (q.w * q.x + q.y * q.z);
  float cr_cp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
  euler.roll = atan2f(sr_cp, cr_cp);
  // 计算pitch
  float sp = 2.0f * (q.w * q.y - q.z * q.x);
  if (fabsf(sp) >= 1.0f)
    euler.pitch = copysignf(M_PI / 2.0f, sp);
  else
    euler.pitch = asinf(sp);
  // 计算yaw
  float sy_cp = 2.0f * (q.w * q.z + q.x * q.y);
  float cy_cp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
  euler.yaw = atan2f(sy_cp, cy_cp);
  return euler;
}