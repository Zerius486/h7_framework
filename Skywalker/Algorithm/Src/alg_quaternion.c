#include "alg_quaternion.h"

#include "math.h"
/**
 * @brief 四元数乘法
 * @param q1 四元数1
 * @param q2 四元数2
 * @return 乘法结果
 */
Quaternion QuaternionMultiply(Quaternion q1, Quaternion q2) {
  Quaternion result;
  result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
  result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
  result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
  result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
  return result;
}
/**
 * @brief 四元数归一化
 * @param q 输入四元数
 * @return 归一化后的四元数
 */
Quaternion QuaternionNormalize(Quaternion q) {
  float magnitude = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (magnitude > 0.0f) {
    float inv_mag = 1.0f / magnitude;
    q.w *= inv_mag;
    q.x *= inv_mag;
    q.y *= inv_mag;
    q.z *= inv_mag;
  }
  return q;
}
/**
 * @brief 欧拉角转四元数
 * @param euler 欧拉角
 * @return 转换后的四元数
 */
Quaternion EulerToQuaternion(Euler euler) {
  Quaternion q;
  float cy = cosf(euler.yaw * 0.5f);
  float sy = sinf(euler.yaw * 0.5f);
  float cp = cosf(euler.pitch * 0.5f);
  float sp = sinf(euler.pitch * 0.5f);
  float cr = cosf(euler.roll * 0.5f);
  float sr = sinf(euler.roll * 0.5f);
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}