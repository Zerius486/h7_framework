#ifndef ALG_EULER_H
#define ALG_EULER_H
// 欧拉角结构体
typedef struct {
  float roll;
  float pitch;
  float yaw;
} Euler;
// 欧拉角速度结构体
typedef struct {
  float roll_rate;
  float pitch_rate;
  float yaw_rate;
} EulerRate;
#include "alg_quaternion.h"
Euler QuaternionToEuler(Quaternion q);
#endif  // ALG_EULER_H
