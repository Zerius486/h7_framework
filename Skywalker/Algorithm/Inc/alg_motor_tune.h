#ifndef ALG_MOTOR_TUNE_H
#define ALG_MOTOR_TUNE_H

#include "alg_autotune.h"

#define MOTOR_TUNE_MAX_CANDIDATES 27U
#define MOTOR_TUNE_HOLD_POINTS 3U

#define MOTOR_TUNE_OBJECTIVE_ERROR (1U << 0)
#define MOTOR_TUNE_OBJECTIVE_OVERSHOOT (1U << 1)
#define MOTOR_TUNE_OBJECTIVE_SETTLING (1U << 2)
#define MOTOR_TUNE_OBJECTIVE_RIPPLE (1U << 3)
#define MOTOR_TUNE_OBJECTIVE_SLEW (1U << 4)
#define MOTOR_TUNE_OBJECTIVE_SATURATION (1U << 5)

// 单电机性能整定状态
typedef enum
{
  kMotorTuneIdle,
  kMotorTuneIdentify,
  kMotorTunePrepare,
  kMotorTuneCharacterize,
  kMotorTuneSearch,
  kMotorTuneValidate,
  kMotorTuneComplete,
  kMotorTuneFailed,
  kMotorTuneCancelled
} MotorTuneState;

// 单电机性能整定失败原因
typedef enum
{
  kMotorTuneNoError,
  kMotorTuneInvalidSample,
  kMotorTuneMotionLimit,
  kMotorTuneIdentificationFailed,
  kMotorTuneOutputMissing,
  kMotorTunePrepareTimeout,
  kMotorTuneNoFeasibleCandidate
} MotorTuneError;

// 候选参数测试结果
typedef enum
{
  kMotorTuneUntested,
  kMotorTuneInvalidDesign,
  kMotorTuneRejected,
  kMotorTuneValidationRejected,
  kMotorTuneVerified
} MotorTuneCandidateState;

// 物理参数搜索范围
typedef struct
{
  float lower;
  float upper;
  uint8_t count; // 1至3，1表示固定，乘积作为默认测试预算
} MotorTuneRange;

// 实测性能结构体
typedef struct
{
  float rms_error;     // 按测试幅度归一化的跟踪均方根误差
  float overshoot;     // 保持段相对本次目标变化量的最大超调比例
  float settling_time; // 到达保持目标后的最坏调节时间（s）
  float idle_ripple;   // 保持误差的最大标准差（rad/s或rad）
  float rms_slew;      // 最终电流指令变化率均方根（A/s）
  float saturation;    // 内部饱和或外部分配受限的采样比例
  bool settled;        // 所有非零目标变化均在窗口末尾稳定
} MotorTuneMetrics;

// 期望性能上限，0表示不约束对应指标
typedef struct
{
  float rms_error;
  float overshoot;
  float settling_time;
  float idle_ripple;
  float rms_slew;
  float saturation;
} MotorTuneLimits;

// 单电机性能整定配置结构体
typedef struct
{
  AutotuneConfig plant;       // 电流、速度、连续角度范围和辨识方法
  MotorTuneRange bandwidth;   // 速度环自然频率（rad/s）
  MotorTuneRange damping;     // 阻尼比
  MotorTuneRange feedforward; // 加速度前馈比例，0至1.5
  MotorTuneLimits limits;
  float speed_amplitude;                // 速度测试最大目标（rad/s）
  float position_amplitude;             // 位置测试幅度（rad），速度模式不使用
  float segment_time;                   // 单个运动/保持段时长（s）
  float settling_band;                  // 稳定带相对测试幅度的比例，0至0.5
  MotorTuneLimits comparison_tolerance; // 指标比较容差，单位与对应指标一致
  uint8_t objectives;         // 帕累托目标位掩码，0使用误差和电流变化率
  uint8_t search_budget;      // 候选预算，0使用三个count的乘积，上限27
  uint8_t validation_repeats; // 独立验证次数，0使用2次，上限3
} MotorTuneConfig;

// 保持位置与实测电流
typedef struct
{
  float position; // 连续角度（rad）
  float current;  // 平均反馈电流（A）
  uint32_t count;
} MotorTuneHold;

// 基准控制器下的机构特征，方向下标0为负向、1为正向
typedef struct
{
  float tested_speed;       // 受行程约束后的速度测试幅度（rad/s）
  float direction_rms[2];   // 方向跟踪误差（rad/s）
  float low_speed_rms[2];   // 低速保持误差（rad/s）
  float working_gain[2];    // 高速保持段实际速度/目标速度
  float startup_delay[2];   // 低速启动响应时间（s）
  float startup_current[2]; // 响应前最大反馈电流绝对值（A）
  float reversal_delay;     // 换向响应时间（s）
  float reversal_travel;    // 换向响应前反馈坐标位移绝对值（rad）
  uint32_t direction_count[2];
  uint32_t low_speed_count[2];
  uint32_t working_count[2];
  bool startup_observed[2];
  bool reversal_observed;
  MotorTuneHold hold[MOTOR_TUNE_HOLD_POINTS]; // 负位置、中心、正位置
} MotorTuneProfile;

// 候选物理参数与可直接使用的PID配置
typedef struct
{
  float bandwidth;
  float damping;
  float feedforward;
  PidConfig speed;
  PidConfig position;
  MotorTuneMetrics training;
  MotorTuneMetrics validation;
  MotorTuneCandidateState state;
  uint8_t validation_runs;
} MotorTuneCandidate;

// 有限测试集中的帕累托结果
typedef struct
{
  SysidModel model;
  MotorTuneProfile profile;
  MotorTuneCandidate candidate[MOTOR_TUNE_MAX_CANDIDATES];
  uint8_t pareto[MOTOR_TUNE_MAX_CANDIDATES]; // candidate数组下标
  uint8_t candidate_count;
  uint8_t evaluated_count;
  uint8_t verified_count;
  uint8_t pareto_count;
} MotorTuneResult;

// 单电机性能整定结构体，建议静态分配
typedef struct
{
  MotorTuneConfig config;
  MotorTuneResult result;
  MotorTuneState state;
  MotorTuneError error;
  AutotuneObject baseline;
  PidObject speed_pid;
  PidObject position_pid;
  MotorTuneState next_state;
  MotorTuneMetrics metrics;
  float center;
  float speed_amplitude;
  float elapsed;
  float ready_time;
  float requested_current;
  float executed_current;
  float previous_current;
  float reference;
  float target;
  float previous_target;
  float last_outside;
  float sum_error;
  float sum_slew;
  float hold_mean;
  float hold_variance;
  float crossing_position;
  float direction_sum[2];
  float low_speed_sum[2];
  float working_sum[2];
  uint32_t segment_ticks;
  uint32_t tick;
  uint32_t samples;
  uint32_t hold_samples;
  uint32_t limited_samples;
  uint8_t segment;
  uint8_t candidate_index;
  uint8_t search_round;
  bool position_test;
  bool pending;
  bool tracked;
  bool initialized;
} MotorTuneObject;

bool MotorTuneInit(MotorTuneObject *tune, const MotorTuneConfig *config);
bool MotorTuneStart(MotorTuneObject *tune, float position, float omega);
bool MotorTuneIsActive(const MotorTuneObject *tune);
void MotorTuneCancel(MotorTuneObject *tune);
float MotorTuneUpdate(MotorTuneObject *tune, float position, float omega,
                      float actual_current);
bool MotorTuneTrackOutput(MotorTuneObject *tune, float executed_current);
bool MotorTuneExport(const MotorTuneObject *tune, uint8_t pareto_index,
                     PidConfig *speed, PidConfig *position);

#endif // ALG_MOTOR_TUNE_H
