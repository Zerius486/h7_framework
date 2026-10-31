#include "app_tune_log.h"

#include <stdio.h>

typedef enum
{
  kAppTuneLogNone,
  kAppTuneLogAutotune,
  kAppTuneLogMotorTune,
} AppTuneLogMode;

static DebugLogObject g_app_tune_log;
static AppTuneLogMode g_app_tune_log_mode;
static uint8_t g_app_tune_log_stage;
static uint8_t g_app_tune_log_phase;
static uint8_t g_app_tune_log_candidate;

/**
 * @brief 获取自动整定阶段名称
 * @param tune 整定对象指针
 * @return 英文阶段名称
 */
static const char *AppTuneAutotuneStage(const AutotuneObject *tune)
{
  if (tune->state == kAutotuneIdentify)
  {
    switch (tune->identification_stage)
    {
      case kAutotuneProbe:
        return "IDENTIFY_PROBE";
      case kAutotuneQuasistatic:
        return "IDENTIFY_QUASISTATIC";
      case kAutotuneDynamic:
        return "IDENTIFY_DYNAMIC";
      case kAutotuneModelValidation:
        return "MODEL_VALIDATION";
      default:
        return "IDENTIFY_UNKNOWN";
    }
  }
  switch (tune->state)
  {
    case kAutotuneSpeed:
      return "SPEED_TEST";
    case kAutotunePosition:
      return "POSITION_TEST";
    case kAutotuneDob:
      return "DOB_TEST";
    default:
      return "UNKNOWN";
  }
}

/**
 * @brief 获取自动整定错误名称
 * @param error 整定错误
 * @return 英文错误名称
 */
static const char *AppTuneAutotuneError(AutotuneError error)
{
  switch (error)
  {
    case kAutotuneInvalidSample:
      return "INVALID_SAMPLE";
    case kAutotuneMotionLimit:
      return "MOTION_LIMIT";
    case kAutotuneInsufficientModel:
      return "INSUFFICIENT_MODEL";
    case kAutotuneTrackingError:
      return "TRACKING_ERROR";
    case kAutotunePredictionError:
      return "PREDICTION_FAILED";
    default:
      return "UNKNOWN";
  }
}

/**
 * @brief 获取单电机性能整定阶段名称
 * @param state 整定状态
 * @return 英文阶段名称
 */
static const char *AppTuneMotorStage(MotorTuneState state)
{
  switch (state)
  {
    case kMotorTuneIdentify:
      return "IDENTIFY";
    case kMotorTunePrepare:
      return "PREPARE";
    case kMotorTuneCharacterize:
      return "CHARACTERIZE";
    case kMotorTuneSearch:
      return "CANDIDATE_TRAIN";
    case kMotorTuneValidate:
      return "CANDIDATE_VALIDATE";
    default:
      return "UNKNOWN";
  }
}

/**
 * @brief 获取单电机性能整定错误名称
 * @param error 整定错误
 * @return 英文错误名称
 */
static const char *AppTuneMotorError(MotorTuneError error)
{
  switch (error)
  {
    case kMotorTuneInvalidSample:
      return "INVALID_SAMPLE";
    case kMotorTuneMotionLimit:
      return "MOTION_LIMIT";
    case kMotorTuneIdentificationFailed:
      return "IDENTIFICATION_FAILED";
    case kMotorTuneOutputMissing:
      return "OUTPUT_TRACK_MISSING";
    case kMotorTunePrepareTimeout:
      return "PREPARE_TIMEOUT";
    case kMotorTuneNoFeasibleCandidate:
      return "NO_FEASIBLE_CANDIDATE";
    default:
      return "UNKNOWN";
  }
}

/**
 * @brief 初始化应用整定记录器
 * @param config 调试记录配置指针
 */
void AppTuneLogInit(const DebugLogConfig *config)
{
  g_app_tune_log_mode = kAppTuneLogNone;
  if (config != NULL)
  {
    (void)DebugLogInit(&g_app_tune_log, config);
  }
}

/**
 * @brief 周期发送缓存日志，整定结束后仍需调用
 */
void AppTuneLogProcess(void)
{
  DebugLogProcess(&g_app_tune_log);
}

/**
 * @brief 开始自动记录一次整定运行
 * @param mode 整定类型
 * @param time_ms 当前时间（ms）
 * @param name 英文运行名称
 * @return 是否已启动记录
 */
static bool AppTuneLogBegin(AppTuneLogMode mode, uint32_t time_ms,
                            const char *name)
{
  if (g_app_tune_log_mode != kAppTuneLogNone || g_app_tune_log.recording)
  {
    return false;
  }
  DebugLogStartNamed(&g_app_tune_log, time_ms, name);
  if (!g_app_tune_log.recording)
  {
    return false;
  }
  g_app_tune_log_mode = mode;
  g_app_tune_log_stage = 0xFFU;
  g_app_tune_log_phase = 0xFFU;
  g_app_tune_log_candidate = 0xFFU;
  return true;
}

/**
 * @brief 记录控制采样与整定模型摘要
 * @param time_ms 当前时间（ms）
 * @param stage 当前阶段
 * @param flags 阶段细分或错误编号
 * @param reference 当前目标值
 * @param position 当前角度（rad）
 * @param omega 当前角速度（rad/s）
 * @param actual_current 实际反馈电流（A）
 * @param requested_current 未限幅请求电流（A）
 * @param executed_current 最终执行电流指令（A）
 * @param model 已辨识模型
 */
static void AppTuneLogWrite(uint32_t time_ms, uint8_t stage, uint8_t flags,
                            float reference, float position, float omega,
                            float actual_current, float requested_current,
                            float executed_current, bool limited,
                            const SysidModel *model)
{
  DebugLogSample sample = {.time_ms = time_ms,
                           .stage = stage,
                           .flags = flags,
                           .limited = limited,
                           .reference = reference,
                           .position = position,
                           .omega = omega,
                           .actual_current = actual_current,
                           .requested_current = requested_current,
                           .executed_current = executed_current};
  if (model != NULL)
  {
    sample.model_inertia = model->inertia;
    sample.model_fitness = model->fitness;
  }
  DebugLogSampleWrite(&g_app_tune_log, &sample);
}

/**
 * @brief 自动记录单电机模型辨识与整定过程
 * @param tune 整定对象指针
 * @param time_ms 当前时间（ms）
 * @param position 当前连续角度（rad）
 * @param omega 当前角速度（rad/s）
 * @param actual_current 实际反馈电流（A）
 * @param requested_current 未限幅请求电流（A）
 * @param executed_current 最终执行电流指令（A）
 */
void AppTuneLogAutotune(const AutotuneObject *tune, uint32_t time_ms,
                        float position, float omega, float actual_current,
                        float requested_current, float executed_current)
{
  AppTuneLogProcess();
  if (tune == NULL)
  {
    return;
  }
  bool active = AutotuneIsActive(tune);
  if (active && g_app_tune_log_mode == kAppTuneLogNone &&
      !AppTuneLogBegin(kAppTuneLogAutotune, time_ms, "MODE AUTOTUNE"))
  {
    return;
  }
  if (g_app_tune_log_mode != kAppTuneLogAutotune)
  {
    return;
  }
  if (!active)
  {
    char message[DEBUG_LOG_MAX_PAYLOAD];
    if (tune->state == kAutotuneComplete)
    {
      (void)snprintf(message, sizeof(message),
                     "RUN_COMPLETE M=%.7g FITNESS=%.4f VFIT=%.4f",
                     tune->result.model.inertia, tune->result.model.fitness,
                     tune->result.model.velocity_fitness);
      DebugLogInfo(&g_app_tune_log, time_ms, message);
    }
    else
    {
      if (tune->state == kAutotuneCancelled)
      {
        (void)snprintf(message, sizeof(message), "RUN_CANCELLED");
        DebugLogInfo(&g_app_tune_log, time_ms, message);
      }
      else
      {
        (void)snprintf(message, sizeof(message), "RUN_FAILED REASON=%s",
                       AppTuneAutotuneError(tune->error));
        DebugLogError(&g_app_tune_log, time_ms, message);
      }
    }
    AppTuneLogWrite(time_ms, (uint8_t)tune->state, (uint8_t)tune->error,
                    tune->speed_pid.reference, position, omega, actual_current,
                    requested_current, executed_current,
                    fabsf(executed_current - requested_current) > 0.001f ||
                        fabsf(requested_current) >=
                            0.999f * tune->config.current_limit,
                    &tune->result.model);
    DebugLogStop(&g_app_tune_log, time_ms);
    g_app_tune_log_mode = kAppTuneLogNone;
    return;
  }
  uint8_t phase = tune->state == kAutotuneIdentify
                      ? (uint8_t)tune->identification_stage
                      : 0U;
  if (tune->state != g_app_tune_log_stage || phase != g_app_tune_log_phase)
  {
    char message[DEBUG_LOG_MAX_PAYLOAD];
    (void)snprintf(message, sizeof(message), "STAGE %s",
                   AppTuneAutotuneStage(tune));
    DebugLogInfo(&g_app_tune_log, time_ms, message);
    g_app_tune_log_stage = (uint8_t)tune->state;
    g_app_tune_log_phase = phase;
  }
  AppTuneLogWrite(
      time_ms, (uint8_t)tune->state, phase, tune->speed_pid.reference, position,
      omega, actual_current, requested_current, executed_current,
      fabsf(executed_current - requested_current) > 0.001f ||
          fabsf(requested_current) >= 0.999f * tune->config.current_limit,
      &tune->result.model);
}

/**
 * @brief 自动记录单电机性能搜索与帕累托验证过程
 * @param tune 整定对象指针
 * @param time_ms 当前时间（ms）
 * @param position 当前连续角度（rad）
 * @param omega 当前角速度（rad/s）
 * @param actual_current 实际反馈电流（A）
 */
void AppTuneLogMotorTune(const MotorTuneObject *tune, uint32_t time_ms,
                         float position, float omega, float actual_current)
{
  AppTuneLogProcess();
  if (tune == NULL)
  {
    return;
  }
  bool active = MotorTuneIsActive(tune);
  if (active && g_app_tune_log_mode == kAppTuneLogNone &&
      !AppTuneLogBegin(kAppTuneLogMotorTune, time_ms, "MODE MOTOR_TUNE"))
  {
    return;
  }
  if (g_app_tune_log_mode != kAppTuneLogMotorTune)
  {
    return;
  }
  if (!active)
  {
    char message[DEBUG_LOG_MAX_PAYLOAD];
    if (tune->state == kMotorTuneComplete)
    {
      (void)snprintf(message, sizeof(message),
                     "RUN_COMPLETE CANDIDATES=%u PARETO=%u",
                     tune->result.candidate_count, tune->result.pareto_count);
      DebugLogInfo(&g_app_tune_log, time_ms, message);
    }
    else
    {
      if (tune->state == kMotorTuneCancelled)
      {
        (void)snprintf(message, sizeof(message), "RUN_CANCELLED");
        DebugLogInfo(&g_app_tune_log, time_ms, message);
      }
      else
      {
        (void)snprintf(message, sizeof(message), "RUN_FAILED REASON=%s",
                       AppTuneMotorError(tune->error));
        DebugLogError(&g_app_tune_log, time_ms, message);
      }
    }
    AppTuneLogWrite(time_ms, (uint8_t)tune->state, (uint8_t)tune->error,
                    tune->reference, position, omega, actual_current,
                    tune->requested_current, tune->executed_current,
                    fabsf(tune->executed_current - tune->requested_current) >
                            0.001f ||
                        fabsf(tune->requested_current) >=
                            0.999f * tune->config.plant.current_limit,
                    &tune->result.model);
    DebugLogStop(&g_app_tune_log, time_ms);
    g_app_tune_log_mode = kAppTuneLogNone;
    return;
  }
  if (tune->state != g_app_tune_log_stage ||
      tune->candidate_index != g_app_tune_log_candidate)
  {
    char message[DEBUG_LOG_MAX_PAYLOAD];
    (void)snprintf(message, sizeof(message), "STAGE %s CANDIDATE=%u",
                   AppTuneMotorStage(tune->state), tune->candidate_index);
    DebugLogInfo(&g_app_tune_log, time_ms, message);
    g_app_tune_log_stage = (uint8_t)tune->state;
    g_app_tune_log_candidate = tune->candidate_index;
  }
  uint8_t flags =
      tune->state == kMotorTuneSearch || tune->state == kMotorTuneValidate
          ? tune->candidate_index
          : tune->segment;
  AppTuneLogWrite(
      time_ms, (uint8_t)tune->state, flags, tune->reference, position, omega,
      actual_current, tune->requested_current, tune->executed_current,
      fabsf(tune->executed_current - tune->requested_current) > 0.001f ||
          fabsf(tune->requested_current) >=
              0.999f * tune->config.plant.current_limit,
      &tune->result.model);
}
