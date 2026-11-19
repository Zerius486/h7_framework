#ifndef ALG_POWER_H
#define ALG_POWER_H

#include <stdbool.h>
#include <stdint.h>

#define POWER_MAX_MOTORS 8U

// 功率控制模式（针对初版自研超电设计渐进降级策略）
typedef enum
{
  kPowerModeRefereeOnly = 0,   // 纯裁判系统缓冲能量闭环（不依赖超电）
  kPowerModeSupercapAssist = 1 // 超电辅助 + 裁判缓冲能量硬兜底
} PowerControlMode;

// 功率分配策略
typedef enum
{
  kPowerAllocProportional = 0, // 解析等比例缩减（全向/麦轮保合外力方向）
  kPowerAllocKktWeighted = 1   // KKT 闭式加权分配（舵轮转向优先/抗打滑）
} PowerAllocStrategy;

// 单电机四项式功率模型：P = k_cu*i^2 + k_t*omega*i + k_iron*omega^2 + idle_loss
typedef struct
{
  float copper_loss;     // 铜损系数 k_cu（W/A²）
  float torque_constant; // 转矩/反电动势系数 k_t（W/(rad/s·A)）
  float iron_loss;       // 铁损/机械阻尼系数 k_iron（W/(rad/s)²）
  float idle_loss;       // 静态待机损耗 P0（W）
} PowerMotorModel;

// 能量预算控制器配置（支持裁判缓冲能量 + 自研超电双环兜底）
typedef struct
{
  PowerControlMode mode;       // 功率控制模式
  float referee_buffer_target; // 裁判系统缓冲能量目标工作点（J，默认 40J）
  float referee_buffer_danger; // 裁判系统缓冲能量危险线（J，默认 15J）
  float referee_buffer_kp;     // 缓冲能量比例增益（W/J，默认 2.0）
  float min_power_ratio;       // 最低保底功率占裁判上限比例（默认 0.70）
  float supercap_energy_target;// 超电剩余能量目标工作点（%，默认 50%）
  float supercap_energy_min;   // 超电最低允许放电能量（%，默认 15%）
  float supercap_kp;           // 超电能量比例增益（W/%，默认 2.0）
  float boost_margin_watts;    // 超电健康时允许的额外超发上限（W，默认 60W）
  uint32_t supercap_timeout_ms;// 超电通信超时判定（ms，默认 200ms）
} PowerBudgetConfig;

// 能量预算控制器运行时状态
typedef struct
{
  PowerBudgetConfig config;
  float power_budget;        // 本拍允许的底盘总电功率上限（W）
  uint32_t last_cap_tick;    // 最近一次收到有效超电数据的时间戳（ms）
  uint32_t cap_recovery_tick;// 超电故障/超时后连续恢复正常的起始时间戳（ms）
  bool has_cap_sample;       // 是否曾收到至少一帧有效超电报文
  bool cap_recovering;       // 是否处于超电恢复消抖窗口
  bool supercap_active;      // 当前周期超电是否实际参与辅助
  bool supercap_fault_latched; // 超电数据异常/掉线降级标志
  bool initialized;
} PowerBudgetController;

// 在线四参数功率模型辨识器（可选，仅在超电功率采样验证可靠后启用）
typedef struct
{
  float theta[4];        // [idle_loss, k_cu, k_t, k_iron]
  float covariance[4][4];
  float forgetting;      // 遗忘因子（如 0.995f）
  float min_theta[4];
  float max_theta[4];
  uint32_t sample_count;
  bool initialized;
} PowerModelSysid;

void PowerBudgetConfigInit(PowerBudgetConfig *config);
bool PowerBudgetInit(PowerBudgetController *ctrl,
                     const PowerBudgetConfig *config);
void PowerBudgetReset(PowerBudgetController *ctrl);
float PowerBudgetUpdate(PowerBudgetController *ctrl, float referee_power_limit,
                        float referee_buffer_energy, bool cap_online,
                        float cap_energy_pct, bool cap_fault,
                        uint32_t now_ms);

float PowerEstimateMotor(const PowerMotorModel *model, float current,
                         float omega);
float PowerEstimateTotal(const PowerMotorModel *models, const float *currents,
                         const float *omegas, uint8_t count);
float PowerAllocate(const PowerMotorModel *models, const float *desired_currents,
                    const float *omegas, const float *weights, uint8_t count,
                    float budget, PowerAllocStrategy strategy,
                    float *allocated_currents);

bool PowerSysidInit(PowerModelSysid *sysid, const PowerMotorModel *initial_sum,
                    float forgetting);
bool PowerSysidUpdate(PowerModelSysid *sysid, const float *currents,
                      const float *omegas, uint8_t count, float measured_power);

#endif // ALG_POWER_H
