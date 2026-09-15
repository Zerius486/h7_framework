#ifndef ALG_KINETICS_H
#define ALG_KINETICS_H

static const float kSqrt2 = 1.41421356237f;

// 底盘类型枚举
typedef enum { kSteerWheel = 0, kMecanumWheel = 1, kOmniWheel = 2 } ChassisType;
// 底盘速度结构体
typedef struct {
  float vx;
  float vy;
  float omega;
} ChassisVelocity;
// 底盘状态结构体
typedef struct {
  float wheel_speed[4];  // 四轮速度
  float wheel_angle[4];  // 四轮转角
} ChassisState;
// 底盘参数结构体
typedef struct {
  float lx;  // x方向半轮距
  float ly;  // y方向半轮距
} ChassisParams;
// 底盘结构体
typedef struct {
  ChassisType type;          // 底盘类型
  ChassisVelocity velocity;  // 速度
  ChassisState state;        // 状态
  ChassisParams params;      // 车体参数
} Chassis;
void ChassisInit(Chassis *chassis, ChassisType type);
void ChassisSetVelocity(Chassis *chassis, float vx, float vy, float omega);
void ChassisSetParams(Chassis *chassis, float lx, float ly);
void ChassisUpdateState(Chassis *chassis);
#endif  // ALG_KINETICS_H
