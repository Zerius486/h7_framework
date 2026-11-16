#include "alg_kinetics.h"

#include <math.h>
#include <stddef.h>

#include "alg_conversion.h"

static const float kHalfSqrt2 = 0.70710678119f;

/**
 * @brief 初始化底盘结构体
 * @param chassis 底盘结构体指针
 * @param type 底盘类型
 */
void ChassisInit(Chassis *chassis, ChassisType type)
{
  if (chassis == NULL)
  {
    return;
  }
  chassis->type = type;
  chassis->velocity.vx = 0.0f;
  chassis->velocity.vy = 0.0f;
  chassis->velocity.omega = 0.0f;
  chassis->params.lx = 0.0f;
  chassis->params.ly = 0.0f;
  chassis->has_wheel_angle_feedback = false;
  for (int i = 0; i < 4; i++)
  {
    chassis->state.wheel_speed[i] = 0.0f;
    chassis->state.wheel_angle[i] = 0.0f;
    chassis->wheel_angle_feedback[i] = 0.0f;
  }
}
/**
 * @brief 设置底盘速度
 * @param chassis 底盘结构体指针
 * @param vx x方向速度（m/s）
 * @param vy y方向速度（m/s）
 * @param omega 旋转角速度（rad/s）
 */
void ChassisSetVelocity(Chassis *chassis, float vx, float vy, float omega)
{
  if (chassis == NULL || !isfinite(vx) || !isfinite(vy) || !isfinite(omega))
  {
    return;
  }
  chassis->velocity.vx = vx;
  chassis->velocity.vy = vy;
  chassis->velocity.omega = omega;
}
/**
 * @brief 设置底盘参数
 * @param chassis 底盘结构体指针
 * @param lx x方向半轮距（m）
 * @param ly y方向半轮距（m）
 */
void ChassisSetParams(Chassis *chassis, float lx, float ly)
{
  if (chassis == NULL || !isfinite(lx) || !isfinite(ly))
  {
    return;
  }
  chassis->params.lx = lx;
  chassis->params.ly = ly;
}
/**
 * @brief 设置舵轮实际转角，用于选择最短转向路径
 * @param chassis 底盘结构体指针
 * @param angle 四轮实际转角（rad）
 */
void ChassisSetWheelAngleFeedback(Chassis *chassis, const float angle[4])
{
  if (chassis == NULL || angle == NULL)
  {
    return;
  }
  for (int i = 0; i < 4; i++)
  {
    if (!isfinite(angle[i]))
    {
      return;
    }
  }
  for (int i = 0; i < 4; i++)
  {
    chassis->wheel_angle_feedback[i] = angle[i];
  }
  chassis->has_wheel_angle_feedback = true;
}
/**
 * @brief 更新底盘状态
 * @param chassis 底盘结构体指针
 */
void ChassisUpdateState(Chassis *chassis)
{
  if (chassis == NULL || !isfinite(chassis->velocity.vx) ||
      !isfinite(chassis->velocity.vy) || !isfinite(chassis->velocity.omega) ||
      !isfinite(chassis->params.lx) || !isfinite(chassis->params.ly))
  {
    return;
  }

  switch (chassis->type)
  {
  case kSteerWheel:
  {
    float omega_ly = chassis->velocity.omega * chassis->params.ly;
    float omega_lx = chassis->velocity.omega * chassis->params.lx;
    float vx[4] = {chassis->velocity.vx - omega_ly,
                   chassis->velocity.vx - omega_ly,
                   chassis->velocity.vx + omega_ly,
                   chassis->velocity.vx + omega_ly};
    float vy[4] = {chassis->velocity.vy + omega_lx,
                   chassis->velocity.vy - omega_lx,
                   chassis->velocity.vy - omega_lx,
                   chassis->velocity.vy + omega_lx};
    for (int i = 0; i < 4; i++)
    {
      float speed = sqrtf(vx[i] * vx[i] + vy[i] * vy[i]);
      if (!isfinite(speed))
      {
        speed = 0.0f;
      }
      chassis->state.wheel_speed[i] = speed;
      if (speed > kSteerWheelSpeedEpsilon)
      {
        float wheel_angle = atan2f(vy[i], vx[i]);
        float angle_feedback = chassis->has_wheel_angle_feedback
                                   ? chassis->wheel_angle_feedback[i]
                                   : chassis->state.wheel_angle[i];
        float angle_error = MathWrapPi(wheel_angle - angle_feedback);
        if (fabsf(angle_error) > (float)M_PI / 2.0f)
        {
          chassis->state.wheel_speed[i] = -speed;
          wheel_angle = MathWrapPi(wheel_angle + (float)M_PI);
        }
        chassis->state.wheel_angle[i] = wheel_angle;
      }
    }
    break;
  }
  case kMecanumWheel:
  {
    float rot = chassis->velocity.omega *
                (chassis->params.lx + chassis->params.ly);
    chassis->state.wheel_speed[0] =
        chassis->velocity.vx - chassis->velocity.vy - rot;
    chassis->state.wheel_speed[1] =
        chassis->velocity.vx + chassis->velocity.vy - rot;
    chassis->state.wheel_speed[2] =
        chassis->velocity.vx - chassis->velocity.vy + rot;
    chassis->state.wheel_speed[3] =
        chassis->velocity.vx + chassis->velocity.vy + rot;
    break;
  }
  case kOmniWheel:
  {
    float rot = kHalfSqrt2 * chassis->velocity.omega *
                (chassis->params.lx + chassis->params.ly);
    float vx_proj = kHalfSqrt2 * chassis->velocity.vx;
    float vy_proj = kHalfSqrt2 * chassis->velocity.vy;
    chassis->state.wheel_speed[0] = -vx_proj + vy_proj + rot;
    chassis->state.wheel_speed[1] = -vx_proj - vy_proj + rot;
    chassis->state.wheel_speed[2] = vx_proj - vy_proj + rot;
    chassis->state.wheel_speed[3] = vx_proj + vy_proj + rot;
    break;
  }
  default:
    break;
  }
}
