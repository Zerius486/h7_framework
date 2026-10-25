#ifndef ALG_DOB_H
#define ALG_DOB_H

#include "stdbool.h"

// 扰动观测器配置结构体
typedef struct
{
  float J_motor;          // 电机转子惯量（kg·m²）
  float J_load;           // 输出轴侧负载惯量（kg·m²）
  float reduction_ratio;  // 转子转速与输出轴转速之比，直驱为1
  float torque_constant;  // 转子侧转矩常数（N·m/A）
  float sampling_time;    // 采样时间（s）
  float filter_cutoff_hz; // 低通滤波截止频率（Hz）
  float max_compensation; // 最大补偿电流（A）
} DobConfig;

// 扰动观测器结构体
typedef struct
{
  DobConfig config;            // 扰动观测器配置
  float J_equivalent;           // 转子侧等效惯量（kg·m²）
  float filter_decay;           // 低通滤波衰减系数
  float filter_derivative_gain; // 离散带限微分增益
  float filtered_current;       // 滤波后的转矩电流（A）
  float filtered_omega;         // 滤波后的转子角速度（rad/s）
  float estimated_disturbance; // 等效扰动力矩（N·m），正值阻碍正向旋转
  float compensation_current;  // 补偿电流（A）
  bool has_sample;             // 已接收采样标志
  bool is_enabled;             // 扰动观测器使能标志
} DobObject;

bool DobInit(DobObject *dob, const DobConfig *config);
void DobReset(DobObject *dob);
float DobUpdate(DobObject *dob, float actual_torque_current, float rotor_omega);

#endif // ALG_DOB_H
