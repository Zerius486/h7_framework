#include "alg_pid.h"

#include "math.h"
#include "stdbool.h"
/**
 * @brief 重置PID控制器状态
 * @param pid_object PID控制器结构体指针
 */
void PidReset(PidObject *pid_object) {
  pid_object->reference = 0.0f;
  pid_object->feedback = 0.0f;
  pid_object->error = 0.0f;
  pid_object->error_sum = 0.0f;
  pid_object->previous_error = 0.0f;
  pid_object->output = 0.0f;
  pid_object->previous_reference = 0.0f;
  pid_object->previous_feedback = 0.0f;
}
/**
 * @brief 按配置初始化PID控制器
 * @param pid_object PID控制器结构体指针
 * @param pid_config PID配置结构体指针
 */
void PidInit(PidObject *pid_object, PidConfig *pid_config) {
  pid_object->is_enabled = true;
  pid_object->kp = pid_config->kp;
  pid_object->ki = pid_config->ki;
  pid_object->kd = pid_config->kd;
  pid_object->sampling_time = pid_config->sampling_time;
  PidReset(pid_object);
  pid_object->is_derivativeonfeedback_enabled =
      pid_config->is_derivativeonfeedback_enabled;
  pid_object->is_feedforward_enabled = pid_config->is_feedforward_enabled;
  pid_object->kff = pid_config->kff;
  pid_object->is_antiwindup_enabled = pid_config->is_antiwindup_enabled;
  pid_object->output_max = pid_config->output_max;
  pid_object->integral_max = pid_config->integral_max;
  pid_object->is_deadband_enabled = pid_config->is_deadband_enabled;
  pid_object->deadband_threshold = pid_config->deadband_threshold;
  pid_object->is_integralseparate_enabled =
      pid_config->is_integralseparate_enabled;
  pid_object->integral_separate_threshold =
      pid_config->integral_separate_threshold;
}
/**
 * @brief 计算PID输出
 * @param pid_object PID控制器结构体指针
 * @param reference 目标值
 * @param feedback 反馈值
 * @return PID输出值
 */
float PidCalculate(PidObject *pid_object, float reference, float feedback) {
  if (pid_object->sampling_time <= 0.0f) {
    return 0.0f;
  }
  // 更新控制器状态
  pid_object->previous_reference = pid_object->reference;
  pid_object->previous_feedback = pid_object->feedback;
  pid_object->reference = reference;
  pid_object->feedback = feedback;
  pid_object->error = reference - feedback;
  float p_out = pid_object->kp * pid_object->error;
  float d_out = 0.0f;
  // 微分先行
  if (pid_object->is_derivativeonfeedback_enabled) {
    d_out = pid_object->kd *
            (pid_object->feedback - pid_object->previous_feedback) /
            pid_object->sampling_time;
  } else {
    d_out = pid_object->kd * (pid_object->error - pid_object->previous_error) /
            pid_object->sampling_time;
  }
  // 前馈计算
  float ff_out = 0.0f;
  if (pid_object->is_feedforward_enabled) {
    ff_out = pid_object->kff *
             (pid_object->reference - pid_object->previous_reference) /
             pid_object->sampling_time;
  }
  // 积分分离逻辑
  bool is_integral_separate = false;
  if (pid_object->is_integralseparate_enabled &&
      fabsf(pid_object->error) > pid_object->integral_separate_threshold) {
    is_integral_separate = true;
  }
  if (!is_integral_separate) {
    bool is_integral_updated = true;
    float i_out_potential =
        pid_object->ki *
        (pid_object->error_sum +
         0.5f * (pid_object->error + pid_object->previous_error) *
             pid_object->sampling_time);
    if (pid_object->is_antiwindup_enabled) {
      float total_output_potential = p_out + d_out + ff_out + i_out_potential;
      if ((total_output_potential > pid_object->output_max &&
           pid_object->error > 0.0f) ||
          (total_output_potential < -pid_object->output_max &&
           pid_object->error < 0.0f)) {
        is_integral_updated = false;
      }
      if (pid_object->error * pid_object->previous_error < 0) {
        pid_object->error_sum *= 0.90f;
      }
    }
    if (is_integral_updated) {
      pid_object->error_sum +=
          0.5f * (pid_object->error + pid_object->previous_error) *
          pid_object->sampling_time;
    }
  }
  if (pid_object->is_antiwindup_enabled) {
    if (pid_object->error_sum > pid_object->integral_max) {
      pid_object->error_sum = pid_object->integral_max;
    } else if (pid_object->error_sum < -pid_object->integral_max) {
      pid_object->error_sum = -pid_object->integral_max;
    }
  }
  float i_out = 0.0f;
  i_out = pid_object->ki * pid_object->error_sum;
  pid_object->output = p_out + i_out + d_out + ff_out;
  if (pid_object->is_deadband_enabled) {
    if (pid_object->output > -pid_object->deadband_threshold &&
        pid_object->output < pid_object->deadband_threshold) {
      pid_object->output = 0.0f;
    }
  }
  if (pid_object->is_antiwindup_enabled) {
    if (pid_object->output > pid_object->output_max) {
      pid_object->output = pid_object->output_max;
    } else if (pid_object->output < -pid_object->output_max) {
      pid_object->output = -pid_object->output_max;
    }
  }
  pid_object->previous_error = pid_object->error;
  return pid_object->output;
}