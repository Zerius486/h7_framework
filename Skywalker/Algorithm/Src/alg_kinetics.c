#include "alg_kinetics.h"

#include <math.h>
/**
 * @brief 初始化底盘结构体
 * @param chassis 底盘结构体指针
 * @param type 底盘类型
 */
void ChassisInit(Chassis *chassis, ChassisType type) {
  chassis->type = type;
  chassis->velocity.vx = 0.0f;
  chassis->velocity.vy = 0.0f;
  chassis->velocity.omega = 0.0f;
  for (int i = 0; i < 4; i++) {
    chassis->state.wheel_speed[i] = 0.0f;
    chassis->state.wheel_angle[i] = 0.0f;
  }
}
/**
 * @brief 设置底盘速度
 * @param chassis 底盘结构体指针
 * @param vx x方向速度
 * @param vy y方向速度
 * @param omega 旋转速度
 */
void ChassisSetVelocity(Chassis *chassis, float vx, float vy, float omega) {
  chassis->velocity.vx = vx;
  chassis->velocity.vy = vy;
  chassis->velocity.omega = omega;
}
/**
 * @brief 设置底盘参数
 * @param chassis 底盘结构体指针
 * @param lx x方向半轮距
 * @param ly y方向半轮距
 */
void ChassisSetParams(Chassis *chassis, float lx, float ly) {
  chassis->params.lx = lx;
  chassis->params.ly = ly;
}
/**
 * @brief 更新底盘状态
 * @param chassis 底盘结构体指针
 */
void ChassisUpdateState(Chassis *chassis) {
  switch (chassis->type) {
    case kSteerWheel: {
      float vx[4] = {
          chassis->velocity.vx - chassis->velocity.omega * chassis->params.ly,
          chassis->velocity.vx - chassis->velocity.omega * chassis->params.ly,
          chassis->velocity.vx + chassis->velocity.omega * chassis->params.ly,
          chassis->velocity.vx + chassis->velocity.omega * chassis->params.ly};
      float vy[4] = {
          chassis->velocity.vy + chassis->velocity.omega * chassis->params.lx,
          chassis->velocity.vy - chassis->velocity.omega * chassis->params.lx,
          chassis->velocity.vy - chassis->velocity.omega * chassis->params.lx,
          chassis->velocity.vy + chassis->velocity.omega * chassis->params.lx};
      for (int i = 0; i < 4; i++) {
        chassis->state.wheel_speed[i] = sqrtf(vx[i] * vx[i] + vy[i] * vy[i]);
        chassis->state.wheel_angle[i] = atan2f(vy[i], vx[i]);
      }
      break;
    }
    case kMecanumWheel:
      chassis->state.wheel_speed[0] =
          chassis->velocity.vx - chassis->velocity.vy +
          chassis->velocity.omega * (chassis->params.lx + chassis->params.ly);
      chassis->state.wheel_speed[1] =
          -chassis->velocity.vx - chassis->velocity.vy +
          chassis->velocity.omega * (chassis->params.lx + chassis->params.ly);
      chassis->state.wheel_speed[2] =
          -chassis->velocity.vx + chassis->velocity.vy -
          chassis->velocity.omega * (chassis->params.lx + chassis->params.ly);
      chassis->state.wheel_speed[3] =
          chassis->velocity.vx + chassis->velocity.vy -
          chassis->velocity.omega * (chassis->params.lx + chassis->params.ly);
      break;
    case kOmniWheel:
      chassis->state.wheel_speed[0] =
          kSqrt2 / 2 * (chassis->velocity.vx + chassis->velocity.vy) +
          chassis->velocity.omega * (chassis->params.lx + chassis->params.ly);
      chassis->state.wheel_speed[1] =
          kSqrt2 / 2 * (-chassis->velocity.vx + chassis->velocity.vy) +
          chassis->velocity.omega * (chassis->params.lx + chassis->params.ly);
      chassis->state.wheel_speed[2] =
          kSqrt2 / 2 * (-chassis->velocity.vx - chassis->velocity.vy) +
          chassis->velocity.omega * (chassis->params.lx + chassis->params.ly);
      chassis->state.wheel_speed[3] =
          kSqrt2 / 2 * (chassis->velocity.vx - chassis->velocity.vy) +
          chassis->velocity.omega * (chassis->params.lx + chassis->params.ly);
      break;
    default:
      break;
  }
}