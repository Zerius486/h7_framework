#include "alg_chassis_tune.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/**
 * @brief 初始化整车落地测试工作区
 * @param tune 测试对象指针
 * @param config 配置结构体指针
 * @return 是否成功
 */
bool ChassisTuneInit(ChassisTuneObject *tune, const ChassisTuneConfig *config)
{
  if (tune == NULL || config == NULL || !isfinite(config->sampling_time) ||
      config->sampling_time <= 0.0f || config->sampling_time > 0.05f ||
      !isfinite(config->speed) || config->speed <= 0.0f ||
      !isfinite(config->omega) || config->omega <= 0.0f ||
      !isfinite(config->distance) || config->distance <= 0.0f ||
      !isfinite(config->angle) || config->angle <= 0.0f ||
      !isfinite(config->response_time) || config->response_time < 0.0f)
  {
    return false;
  }
  memset(tune, 0, sizeof(*tune));
  tune->config = *config;
  AutotuneConfig yaw = {.sampling_time = config->sampling_time,
                        .current_limit = config->omega,
                        .speed_limit = config->omega,
                        .position_min = -config->angle,
                        .position_max = config->angle,
                        .response_time = config->response_time,
                        .identification = config->identification,
                        .optimize = config->optimize};
  return AutotuneInit(&tune->yaw, &yaw);
}

/**
 * @brief 开始静止检查及协调运动测试
 * @param tune 测试对象指针
 * @return 是否成功启动
 */
bool ChassisTuneStart(ChassisTuneObject *tune)
{
  if (tune == NULL || (tune->state >= kChassisTunePrepare &&
                       tune->state <= kChassisTuneLateral))
  {
    return false;
  }
  ChassisTuneConfig config = tune->config;
  if (!ChassisTuneInit(tune, &config))
  {
    return false;
  }
  tune->state = kChassisTunePrepare;
  return true;
}

/**
 * @brief 取消落地测试并归零运动指令
 * @param tune 测试对象指针
 */
void ChassisTuneCancel(ChassisTuneObject *tune)
{
  if (tune != NULL && tune->state >= kChassisTunePrepare &&
      tune->state <= kChassisTuneLateral)
  {
    AutotuneCancel(&tune->yaw);
    tune->state = kChassisTuneCancelled;
    tune->command = (ChassisVelocity){0};
    tune->aligning = false;
  }
}

/**
 * @brief 判断落地测试是否正在接管运动目标
 * @param tune 测试对象指针
 * @return 是否处于测试阶段
 */
bool ChassisTuneIsActive(const ChassisTuneObject *tune)
{
  return tune != NULL && tune->state >= kChassisTunePrepare &&
         tune->state <= kChassisTuneLateral;
}

/**
 * @brief 切换阶段并清除阶段统计
 * @param tune 测试对象指针
 * @param state 新阶段
 */
static void ChassisTunePhase(ChassisTuneObject *tune, ChassisTuneState state)
{
  tune->state = state;
  tune->elapsed = 0.0f;
  tune->samples = 0U;
  tune->sum_error = 0.0f;
  tune->sum_cross = 0.0f;
  tune->sine = 0.0f;
  tune->cosine = 0.0f;
  tune->still_time = 0.0f;
  tune->frequency =
      fmaxf(1.0f, 4.0f * tune->config.speed / tune->config.distance);
}

/**
 * @brief 结束未通过反馈或运动范围检查的测试
 * @param tune 测试对象指针
 * @param error 失败原因
 */
static void ChassisTuneFail(ChassisTuneObject *tune, AutotuneError error)
{
  tune->state = kChassisTuneFailed;
  tune->error = error;
  tune->command = (ChassisVelocity){0};
  tune->aligning = false;
}

/**
 * @brief 更新落地测试，轮速用于平移反馈，陀螺仪用于旋转反馈
 * @param tune 测试对象指针
 * @param velocity 轮速反算的车体速度
 * @param gyro 安装方向修正后的陀螺角速度（rad/s）
 * @param executed_omega 上周期发送给运动学的旋转指令（rad/s）
 * @return 本周期协调车体速度指令
 */
ChassisVelocity ChassisTuneUpdate(ChassisTuneObject *tune,
                                  const ChassisVelocity *velocity, float gyro,
                                  float executed_omega)
{
  ChassisVelocity zero = {0};
  if (tune == NULL || tune->state < kChassisTunePrepare ||
      tune->state > kChassisTuneLateral)
  {
    return zero;
  }
  tune->command = zero;
  tune->aligning = false;
  if (velocity == NULL || !isfinite(velocity->vx) || !isfinite(velocity->vy) ||
      !isfinite(velocity->omega) || !isfinite(gyro) ||
      !isfinite(executed_omega))
  {
    ChassisTuneFail(tune, kAutotuneInvalidSample);
    return zero;
  }
  float dt = tune->config.sampling_time;
  tune->elapsed += dt;
  tune->total_time += dt;
  float omega = gyro - tune->gyro_bias;
  if (tune->state != kChassisTunePrepare)
  {
    float heading = tune->heading + 0.5f * omega * dt;
    tune->x +=
        (velocity->vx * cosf(heading) - velocity->vy * sinf(heading)) * dt;
    tune->y +=
        (velocity->vx * sinf(heading) + velocity->vy * cosf(heading)) * dt;
    tune->heading += omega * dt;
    if (hypotf(tune->x, tune->y) >= tune->config.distance ||
        fabsf(tune->heading) >= tune->config.angle ||
        fabsf(omega) > 1.2f * tune->config.omega)
    {
      ChassisTuneFail(tune, kAutotuneMotionLimit);
      return zero;
    }
  }
  if (tune->state == kChassisTunePrepare)
  {
    if (hypotf(velocity->vx, velocity->vy) > 0.03f ||
        fabsf(velocity->omega) > 0.03f || fabsf(gyro) > 0.1f)
    {
      ChassisTuneFail(tune, kAutotuneTrackingError);
      return zero;
    }
    tune->sum_gyro += gyro;
    tune->sum_gyro_square += gyro * gyro;
    tune->samples++;
    if (tune->elapsed >= 1.0f)
    {
      tune->gyro_bias = tune->sum_gyro / (float)tune->samples;
      float variance = tune->sum_gyro_square / (float)tune->samples -
                       tune->gyro_bias * tune->gyro_bias;
      if (variance > 0.0004f)
      {
        ChassisTuneFail(tune, kAutotuneInvalidSample);
        return zero;
      }
      (void)AutotuneStart(&tune->yaw, 0.0f, 0.0f, NULL);
      ChassisTunePhase(tune, kChassisTuneYaw);
    }
  }
  else if (tune->state == kChassisTuneYaw)
  {
    // 舵轮先对准纯旋转方向，预对准期间驱动电流保持为零。
    if (tune->elapsed < 1.0f)
    {
      tune->command.omega = 0.001f;
      tune->aligning = true;
    }
    else
    {
      AutotuneTrackOutput(&tune->yaw, executed_omega);
      tune->command.omega = AutotuneUpdate(&tune->yaw, tune->heading, omega,
                                           executed_omega, omega);
      if (tune->yaw.state == kAutotuneFailed)
      {
        ChassisTuneFail(tune, tune->yaw.error);
      }
      else if (tune->yaw.state == kAutotuneComplete)
      {
        tune->next = kChassisTuneForward;
        ChassisTunePhase(tune, kChassisTuneBrake);
      }
    }
  }
  else if (tune->state == kChassisTuneBrake)
  {
    if (hypotf(velocity->vx, velocity->vy) < 0.03f && fabsf(omega) < 0.03f)
    {
      tune->still_time += dt;
    }
    else
    {
      tune->still_time = 0.0f;
    }
    if (tune->elapsed >= 1.0f && tune->still_time >= 0.5f)
    {
      ChassisTunePhase(tune, tune->next);
    }
    else if (tune->elapsed > 4.0f)
    {
      ChassisTuneFail(tune, kAutotuneTrackingError);
    }
  }
  else
  {
    bool lateral = tune->state == kChassisTuneLateral;
    ChassisTuneResponse *response = lateral ? &tune->lateral : &tune->forward;
    float phase = tune->frequency * fmaxf(tune->elapsed - 1.0f, 0.0f);
    float reference = tune->config.speed * sinf(phase);
    if (tune->elapsed < 1.0f)
    {
      reference = 0.001f;
      tune->aligning = true;
    }
    if (lateral)
    {
      tune->command.vy = reference;
    }
    else
    {
      tune->command.vx = reference;
    }
    if (!tune->aligning)
    {
      float feedback = lateral ? velocity->vy : velocity->vx;
      float cross = lateral ? velocity->vx : velocity->vy;
      float error = (reference - feedback) / tune->config.speed;
      tune->sum_error += error * error;
      tune->sum_cross += cross * cross;
      tune->sine += feedback * sinf(phase) * dt;
      tune->cosine += feedback * cosf(phase) * dt;
      response->peak = fmaxf(response->peak, fabsf(error));
      tune->samples++;
      response->rms = sqrtf(tune->sum_error / (float)tune->samples);
      response->coupling =
          sqrtf(tune->sum_cross / (float)tune->samples) / tune->config.speed;
      if (phase >= 12.5663706f)
      {
        float duration = 12.5663706f / tune->frequency;
        response->gain = 2.0f * hypotf(tune->sine, tune->cosine) /
                         (duration * tune->config.speed);
        response->lag = atan2f(-tune->cosine, tune->sine);
        tune->next = lateral ? kChassisTuneComplete : kChassisTuneLateral;
        ChassisTunePhase(tune, kChassisTuneBrake);
        tune->command = zero;
      }
    }
  }
  return tune->command;
}
