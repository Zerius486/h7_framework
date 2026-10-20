#ifndef ALG_EULER_H
#define ALG_EULER_H
// 欧拉角结构体
typedef struct
{
  float roll;  // 横滚角（rad）
  float pitch; // 俯仰角（rad）
  float yaw;   // 偏航角（rad）
} Euler;
// 欧拉角速度结构体
typedef struct
{
  float roll_rate;  // 横滚角速度（rad/s）
  float pitch_rate; // 俯仰角速度（rad/s）
  float yaw_rate;   // 偏航角速度（rad/s）
} EulerRate;
#include "alg_quaternion.h"
Euler QuaternionToEuler(Quaternion q);
#endif // ALG_EULER_H
