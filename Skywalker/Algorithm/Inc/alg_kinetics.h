#ifndef ALG_KINETICS_H
#define ALG_KINETICS_H

#include <stdbool.h>

static const float kSqrt2 = 1.41421356237f;
static const float kSteerWheelSpeedEpsilon = 1e-4f;

// 轮序为左前、左后、右后、右前，x向前、y向左，omega逆时针为正
// 麦轮采用X型辊子布置，轮速向前为正；全向轮正向依次为135°、-135°、-45°、45°
// 底盘类型枚举
typedef enum
{
  kSteerWheel = 0,
  kMecanumWheel = 1,
  kOmniWheel = 2
} ChassisType;
// 底盘速度结构体
typedef struct
{
  float vx;    // x方向速度（m/s）
  float vy;    // y方向速度（m/s）
  float omega; // 旋转角速度（rad/s）
} ChassisVelocity;
// 底盘状态结构体
typedef struct
{
  float wheel_speed[4]; // 四轮线速度（m/s），可为负值
  float wheel_angle[4]; // 四轮转角（rad）
} ChassisState;
// 底盘参数结构体
typedef struct
{
  float lx; // x方向半轮距（m）
  float ly; // y方向半轮距（m）
} ChassisParams;
// 底盘结构体
typedef struct
{
  ChassisType type;               // 底盘类型
  ChassisVelocity velocity;       // 速度
  ChassisState state;             // 状态
  ChassisParams params;           // 车体参数
  float wheel_angle_feedback[4]; // 舵轮实际转角（rad）
  bool has_wheel_angle_feedback; // 已设置实际转角
} Chassis;
void ChassisInit(Chassis *chassis, ChassisType type);
void ChassisSetVelocity(Chassis *chassis, float vx, float vy, float omega);
void ChassisSetParams(Chassis *chassis, float lx, float ly);
void ChassisSetWheelAngleFeedback(Chassis *chassis, const float angle[4]);
void ChassisUpdateState(Chassis *chassis);
#endif // ALG_KINETICS_H
