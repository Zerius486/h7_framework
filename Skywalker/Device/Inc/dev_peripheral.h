/**
 * @file dev_peripheral.h
 * @brief 统一外设生命周期状态机与安全看门狗守护抽象
 * @note 严格依据 docs/peripheral_state_machine.drawio 设计图实现。
 *       涵盖外设未挂载、自检握手、零偏标定、安全待命、闭环使能、
 *       通信离线超时急停保护、防暴冲多圈角重同步以及硬件故障自愈链。
 */

#ifndef DEV_PERIPHERAL_H
#define DEV_PERIPHERAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 外设类型枚举
typedef enum {
  kPeripheralTypeMotorDji = 0, // 大疆 CAN 电机 (M3508 / M2006 / GM6020)
  kPeripheralTypeMotorDm = 1,  // 达妙 CAN 电机 (J4310 / J8009P)
  kPeripheralTypeMotorLk = 2,  // 瓴控 CAN 电机
  kPeripheralTypeImu = 3,      // 惯导传感器 (BMI088 等)
  kPeripheralTypeRemote = 4,   // 遥控器 (DJI DT7 等)
  kPeripheralTypeReferee = 5,  // 裁判系统通信
  kPeripheralTypeVision = 6,   // 视觉 / 上位机通信
  kPeripheralTypeCustom = 7,   // 自定义外设
} PeripheralType;

// 外设统一生命周期状态枚举（严格对齐设计图 7 大核心状态）
typedef enum {
  kPeripheralStateUninit = 0,      // 状态 1：未挂载 / 上电初始态 (Unmounted / Uninitialized)
  kPeripheralStateProbing = 1,     // 状态 2：通信握手与自检态 (Probing & Self-Test)
  kPeripheralStateCalibrating = 2, // 状态 3：零偏标定与机械零点对齐态 (Calibrating & Zeroing)
  kPeripheralStateDisabled = 3,    // 状态 4：安全失能待命态 (Disabled / Standby, is_enabled = false)
  kPeripheralStateEnabled = 4,     // 状态 5：闭环使能运行态 (Enabled / Closed-Loop Active, is_enabled = true)
  kPeripheralStateOffline = 5,     // 异常态 A：通信超时失联保护态 (Offline / Timeout)
  kPeripheralStateFault = 6,       // 异常态 B：硬件/驱动故障锁死态 (Fault Locked)
} PeripheralState;

// 默认超时阈值定义（单位：毫秒 ms）
enum {
  kPeripheralDefaultTimeoutMotorMs = 50,         // 电机反馈接收超时（默认 50ms）
  kPeripheralDefaultTimeoutRemoteMs = 100,       // 遥控器 DBUS 接收超时（默认 100ms）
  kPeripheralDefaultTimeoutRefereeMs = 500,      // 裁判系统接收超时（默认 500ms）
  kPeripheralDefaultTimeoutImuMs = 20,           // 惯导通信超时（默认 20ms）
  kPeripheralDefaultTimeoutVisionMs = 200,       // 视觉通信超时（默认 200ms）
  kPeripheralDefaultReconnectIntervalMs = 100,   // 低频重连探测周期（100ms，即 10Hz）
  kPeripheralMaxRegisteredCount = 32,            // 管理器支持的最大外设登记数量
};

/**
 * @brief 外设专属操作回调集 (Hooks)
 * @note 各设备根据自身硬件特性实现，无需关心的回调可设为 NULL。
 */
typedef struct {
  /**
   * @brief 使能回调（例如达妙电机发送 0xFC 帧进入使能）
   * @param device_ptr 所属具体设备的指针
   * @return true 使能指令成功下发并确认; false 使能失败
   */
  bool (*on_enable)(void *device_ptr);

  /**
   * @brief 失能回调（例如达妙电机发送 0xFD 帧，大疆电机目标电流置 0）
   * @param device_ptr 所属具体设备的指针
   */
  void (*on_disable)(void *device_ptr);

  /**
   * @brief 离线保护回调（通信超时触发，执行强制急停、输出归零、清空 PID 积分）
   * @param device_ptr 所属具体设备的指针
   */
  void (*on_offline)(void *device_ptr);

  /**
   * @brief 防暴冲重同步回调（离线重连探测成功、首帧恢复时执行，例如重新对齐单圈/多圈展开基准角）
   * @param device_ptr 所属具体设备的指针
   */
  void (*on_resync)(void *device_ptr);

  /**
   * @brief 硬件故障清除回调（例如达妙电机发送 0xFB 清错帧）
   * @param device_ptr 所属具体设备的指针
   * @return true 错误清除成功; false 清除失败
   */
  bool (*on_clear_fault)(void *device_ptr);

  /**
   * @brief 低频重连探测回调（处于 Offline 状态时按 5~10Hz 定期调用，发送失能/查询帧尝试唤醒）
   * @param device_ptr 所属具体设备的指针
   */
  void (*on_reconnect_probe)(void *device_ptr);
} PeripheralOps;

/**
 * @brief 统一外设生命周期守护对象 (PeripheralGuard)
 */
typedef struct {
  const char *name;               // 外设名称字符串（用于日志和调试识别）
  PeripheralType type;            // 外设类型
  PeripheralState state;          // 当前生命周期状态
  PeripheralState last_state;     // 上一周期状态（用于捕获跃迁边缘）
  uint32_t error_code;            // 硬件/驱动层错误码（0 表示无错误）

  uint32_t last_rx_tick;          // 最近一次收到合法报文的时间戳 (ms)
  uint32_t state_enter_tick;      // 进入当前状态的时间戳 (ms)
  uint32_t timeout_ms;            // 离线超时判定阈值 (ms)
  uint32_t reconnect_interval_ms; // 离线探测发送间隔 (ms)
  uint32_t last_probe_tick;       // 上次下发重连探测帧的时间戳 (ms)

  uint16_t rx_fps_counter;        // 实时帧率滑动计数器
  uint16_t rx_fps;                // 当前实时接收帧率 (Hz)
  uint32_t last_fps_tick;         // 上次帧率结算时间戳 (ms)

  bool has_rx_frame;              // 是否已接收到至少一帧合法报文
  bool has_state_enter_tick;      // 是否已记录进入当前状态的时间戳
  bool has_probe_tick;            // 是否已记录上次探测时间戳
  bool has_fps_tick;              // 是否已记录帧率统计起始时间戳

  void *device_ptr;               // 指向宿主具体设备对象的指针
  PeripheralOps ops;              // 外设操作钩子
} PeripheralGuard;

/**
 * @brief 初始化外设守护状态机
 * @param guard 守护对象指针
 * @param name 外设名称标识（如 "dm_yaw", "dji_wheel_fl", "bmi088"）
 * @param type 外设类别
 * @param timeout_ms 离线超时判定阈值（单位 ms，传 0 则使用该类型的默认值）
 * @param device_ptr 指向宿主设备实例的指针（将作为参数传递给 ops 回调）
 * @param ops 设备操作回调集（传 NULL 表示无额外动作）
 */
void PeripheralGuardInit(PeripheralGuard *guard, const char *name,
                        PeripheralType type, uint32_t timeout_ms,
                        void *device_ptr, const PeripheralOps *ops);

/**
 * @brief 接收报文喂狗通知
 * @note 必须在总线 RX 回调或数据解码成功处调用，刷新通信活跃时间戳。
 *       若当前处于 Offline 离线态，该调用将自动触发防暴冲重同步 (on_resync) 并流转至待命态。
 * @param guard 守护对象指针
 * @param now_tick 当前时间戳（ms，通常传 osKernelGetTickCount() 或 HAL_GetTick()）
 */
void PeripheralGuardFeed(PeripheralGuard *guard, uint32_t now_tick);

/**
 * @brief 周期性状态轮询与看门狗判定
 * @note 建议在周期任务（如 100Hz 的 ManagerTaskLoop 或 1kHz 控制循环）中调用。
 *       负责超时离线判定、低频重连探测脉冲触发及实时通信帧率计算。
 * @param guard 守护对象指针
 * @param now_tick 当前时间戳 (ms)
 */
void PeripheralGuardUpdate(PeripheralGuard *guard, uint32_t now_tick);

/**
 * @brief 通知自检/握手成功，进入零偏标定态
 * @param guard 守护对象指针
 */
void PeripheralGuardNotifyProbingPassed(PeripheralGuard *guard);

/**
 * @brief 通知标定已完成，状态进入安全失能待命态
 * @param guard 守护对象指针
 */
void PeripheralGuardNotifyCalibrated(PeripheralGuard *guard);

/**
 * @brief 请求使能外设（进入闭环使能运行态）
 * @note 仅当外设处于 Disabled 待命态且 error_code == 0 时允许使能。
 * @param guard 守护对象指针
 * @return true 成功进入使能态; false 处于故障/离线等非待命状态，拒绝使能
 */
bool PeripheralGuardEnable(PeripheralGuard *guard);

/**
 * @brief 请求失能外设（退回安全待命态）
 * @param guard 守护对象指针
 * @return true 成功失能; false 失能失败
 */
bool PeripheralGuardDisable(PeripheralGuard *guard);

/**
 * @brief 触发硬件或驱动故障
 * @note 立即切入 Fault 锁死态，切断输出并记录错误码。
 * @param guard 守护对象指针
 * @param error_code 错误码（非零）
 */
void PeripheralGuardTriggerFault(PeripheralGuard *guard, uint32_t error_code);

/**
 * @brief 执行故障清除与自愈恢复
 * @note 调用 ops.on_clear_fault，并在错误码清零后安全退回 Disabled 待命态（严禁直接跳回使能态）。
 * @param guard 守护对象指针
 * @return true 故障清除成功并已恢复待命; false 清除失败
 */
bool PeripheralGuardClearFault(PeripheralGuard *guard);

/**
 * @brief 获取状态机状态的可读文本字符串
 * @param state 状态枚举
 * @return 状态英文名称
 */
const char *PeripheralGetStateName(PeripheralState state);

/* ========================================================================= */
/* 快速内联查询辅助函数                                                      */
/* ========================================================================= */

/**
 * @brief 检查外设通信是否在线（排除 Uninit 与 Offline）
 */
static inline bool PeripheralIsOnline(const PeripheralGuard *guard) {
  if (guard == NULL) {
    return false;
  }
  return (guard->state != kPeripheralStateUninit) &&
         (guard->state != kPeripheralStateOffline);
}

/**
 * @brief 检查外设是否已就绪（处于待命 Disabled 或已使能 Enabled 态）
 */
static inline bool PeripheralIsReady(const PeripheralGuard *guard) {
  if (guard == NULL) {
    return false;
  }
  return (guard->state == kPeripheralStateDisabled) ||
         (guard->state == kPeripheralStateEnabled);
}

/**
 * @brief 检查外设是否处于正常闭环使能运行态
 */
static inline bool PeripheralIsEnabled(const PeripheralGuard *guard) {
  if (guard == NULL) {
    return false;
  }
  return guard->state == kPeripheralStateEnabled;
}

/**
 * @brief 检查外设是否处于故障锁死态
 */
static inline bool PeripheralIsFault(const PeripheralGuard *guard) {
  if (guard == NULL) {
    return false;
  }
  return guard->state == kPeripheralStateFault;
}

/* ========================================================================= */
/* 全局外设集中管理器 (PeripheralManager)                                    */
/* ========================================================================= */

/**
 * @brief 全局外设管理器初始化（幂等，保留先于调度器启动阶段已登记的外设）
 */
void PeripheralManagerInit(void);

/**
 * @brief 复位并清空全局外设管理器登记表（仅用于测试或系统重载）
 */
void PeripheralManagerReset(void);

/**
 * @brief 登记注册一个外设到管理器中
 * @param guard 外设守护指针
 * @param is_critical 是否为关键外设（若为 true，该外设离线或故障将触发系统级安全联锁）
 * @return true 登记成功; false 登记已满
 */
bool PeripheralManagerRegister(PeripheralGuard *guard, bool is_critical);

/**
 * @brief 周期性批量更新所有已登记的外设
 * @note 建议在 ManagerTaskLoop (例如 100Hz) 中调用
 * @param now_tick 当前时间戳 (ms)
 */
void PeripheralManagerUpdateAll(uint32_t now_tick);

/**
 * @brief 检查所有关键外设是否全部在线
 * @return true 全部关键外设在线; false 存在关键外设离线
 */
bool PeripheralManagerIsAllCriticalOnline(void);

/**
 * @brief 检查是否存在任何关键外设发生硬件故障
 * @return true 存在故障; false 全部关键外设正常
 */
bool PeripheralManagerHasCriticalFault(void);

/**
 * @brief 全局一键安全急停所有已登记外设并锁存急停状态
 * @note 强制所有处于使能态的外设调用 Disable 退回待命态并下发零电流，
 *       在调用 PeripheralManagerClearEmergencyStop() 解锁前禁止任何外设重新使能
 */
void PeripheralManagerEmergencyStopAll(void);

/**
 * @brief 解除全局安全急停锁存状态
 */
void PeripheralManagerClearEmergencyStop(void);

/**
 * @brief 查询当前是否处于全局安全急停锁存状态
 * @return true 处于急停锁存态; false 正常态
 */
bool PeripheralManagerIsEmergencyStopped(void);

/**
 * @brief 获取发生离线或故障的外设总数
 * @return 异常外设数量
 */
uint32_t PeripheralManagerGetAbnormalCount(void);

#ifdef __cplusplus
}
#endif

#endif // DEV_PERIPHERAL_H
