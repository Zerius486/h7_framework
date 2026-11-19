/**
 * @file dev_peripheral.c
 * @brief 统一外设生命周期状态机与安全看门狗守护实现
 */

#include "dev_peripheral.h"

#include <stddef.h>
#include <string.h>

#include "stm32h7xx_hal.h"

static inline uint32_t PeripheralEnterCritical(void) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static inline void PeripheralExitCritical(uint32_t primask) {
  if (primask == 0U) {
    __enable_irq();
  }
}

/* ========================================================================= */
/* 单个外设守护对象 (PeripheralGuard) 实现                                    */
/* ========================================================================= */

void PeripheralGuardInit(PeripheralGuard *guard, const char *name,
                        PeripheralType type, uint32_t timeout_ms,
                        void *device_ptr, const PeripheralOps *ops) {
  if (guard == NULL) {
    return;
  }

  guard->name = (name != NULL) ? name : "unnamed_dev";
  guard->type = type;
  guard->state = kPeripheralStateProbing; // 绑定总线后初始进入握手自检态
  guard->last_state = kPeripheralStateUninit;
  guard->error_code = 0U;

  // 根据外设类别分配默认超时阈值
  if (timeout_ms > 0U) {
    guard->timeout_ms = timeout_ms;
  } else {
    switch (type) {
      case kPeripheralTypeMotorDji:
      case kPeripheralTypeMotorDm:
      case kPeripheralTypeMotorLk:
        guard->timeout_ms = kPeripheralDefaultTimeoutMotorMs;
        break;
      case kPeripheralTypeRemote:
        guard->timeout_ms = kPeripheralDefaultTimeoutRemoteMs;
        break;
      case kPeripheralTypeReferee:
        guard->timeout_ms = kPeripheralDefaultTimeoutRefereeMs;
        break;
      case kPeripheralTypeImu:
        guard->timeout_ms = kPeripheralDefaultTimeoutImuMs;
        break;
      case kPeripheralTypeVision:
        guard->timeout_ms = kPeripheralDefaultTimeoutVisionMs;
        break;
      case kPeripheralTypeCustom:
      default:
        guard->timeout_ms = kPeripheralDefaultTimeoutMotorMs;
        break;
    }
  }

  uint32_t now_tick = HAL_GetTick();
  guard->reconnect_interval_ms = kPeripheralDefaultReconnectIntervalMs;
  guard->last_rx_tick = now_tick;
  guard->state_enter_tick = now_tick;
  guard->last_probe_tick = 0U;
  guard->rx_fps_counter = 0U;
  guard->rx_fps = 0U;
  guard->last_fps_tick = now_tick;
  guard->has_rx_frame = false;
  guard->has_state_enter_tick = true;
  guard->has_probe_tick = false;
  guard->has_fps_tick = true;

  guard->device_ptr = device_ptr;
  if (ops != NULL) {
    guard->ops = *ops;
  } else {
    memset(&guard->ops, 0, sizeof(PeripheralOps));
  }
}

void PeripheralGuardFeed(PeripheralGuard *guard, uint32_t now_tick) {
  if (guard == NULL) {
    return;
  }

  bool trigger_resync = false;
  uint32_t primask = PeripheralEnterCritical();

  guard->last_rx_tick = now_tick;
  guard->has_rx_frame = true;
  guard->rx_fps_counter++;

  // 1. 如果当前处于离线失联态：说明报文恢复，触发重连与防暴冲重同步
  if (guard->state == kPeripheralStateOffline) {
    trigger_resync = true;
    // 重同步后退回安全待命态（严禁直接跳到使能态）
    guard->last_state = guard->state;
    guard->state = kPeripheralStateDisabled;
    guard->state_enter_tick = now_tick;
    guard->has_state_enter_tick = true;
  } else if (guard->state == kPeripheralStateProbing) {
    // 2. 如果当前处于握手自检态：首帧合法报文到达，握手通过，进入待命态
    guard->last_state = guard->state;
    guard->state = kPeripheralStateDisabled;
    guard->state_enter_tick = now_tick;
    guard->has_state_enter_tick = true;
  }

  PeripheralExitCritical(primask);

  if (trigger_resync && guard->ops.on_resync != NULL) {
    guard->ops.on_resync(guard->device_ptr);
  }
}

static inline uint32_t PeripheralElapsedTick(uint32_t now_tick,
                                             uint32_t past_tick) {
  int32_t diff = (int32_t)(now_tick - past_tick);
  return (diff > 0) ? (uint32_t)diff : 0U;
}

void PeripheralGuardUpdate(PeripheralGuard *guard, uint32_t now_tick) {
  if (guard == NULL) {
    return;
  }

  bool trigger_offline = false;
  bool trigger_probe = false;
  uint32_t primask = PeripheralEnterCritical();

  if (!guard->has_state_enter_tick) {
    guard->state_enter_tick = now_tick;
    guard->has_state_enter_tick = true;
  }

  // 1. 通信离线超时检测（排除 Uninit、Fault 以及已处于 Offline 的外设）
  if (guard->state != kPeripheralStateUninit &&
      guard->state != kPeripheralStateOffline &&
      guard->state != kPeripheralStateFault) {
    // 若已收过报文则按 last_rx_tick 计时；若尚在 Probing 且未收到首帧，则按入态时间计时以允许 Probing 超时转入 Offline
    uint32_t reference_tick =
        guard->has_rx_frame ? guard->last_rx_tick : guard->state_enter_tick;
    if (PeripheralElapsedTick(now_tick, reference_tick) > guard->timeout_ms) {
      guard->last_state = guard->state;
      guard->state = kPeripheralStateOffline;
      guard->state_enter_tick = now_tick;
      guard->has_state_enter_tick = true;
      trigger_offline = true;
    }
  }

  // 2. 离线状态下的低频重连探测脉冲（5~10Hz）
  if (guard->state == kPeripheralStateOffline) {
    if (!guard->has_probe_tick ||
        PeripheralElapsedTick(now_tick, guard->last_probe_tick) >=
            guard->reconnect_interval_ms) {
      guard->last_probe_tick = now_tick;
      guard->has_probe_tick = true;
      trigger_probe = true;
    }
  }

  // 3. 实时接收帧率计算（以 1000ms 为窗口统计）
  if (!guard->has_fps_tick) {
    guard->last_fps_tick = now_tick;
    guard->has_fps_tick = true;
  } else if (PeripheralElapsedTick(now_tick, guard->last_fps_tick) >= 1000U) {
    guard->rx_fps = guard->rx_fps_counter;
    guard->rx_fps_counter = 0U;
    guard->last_fps_tick = now_tick;
  }

  PeripheralExitCritical(primask);

  if (trigger_offline) {
    // 触发安全保护动作：输出强制归零、清空积分
    if (guard->ops.on_offline != NULL) {
      guard->ops.on_offline(guard->device_ptr);
    } else if (guard->ops.on_disable != NULL) {
      guard->ops.on_disable(guard->device_ptr);
    }
  }

  if (trigger_probe && guard->ops.on_reconnect_probe != NULL) {
    guard->ops.on_reconnect_probe(guard->device_ptr);
  }
}

void PeripheralGuardNotifyProbingPassed(PeripheralGuard *guard) {
  if (guard == NULL) {
    return;
  }
  uint32_t now_tick = HAL_GetTick();
  uint32_t primask = PeripheralEnterCritical();
  if (guard->state == kPeripheralStateProbing) {
    guard->last_state = guard->state;
    guard->state = kPeripheralStateCalibrating;
    guard->state_enter_tick = now_tick;
    guard->has_state_enter_tick = true;
  }
  PeripheralExitCritical(primask);
}

void PeripheralGuardNotifyCalibrated(PeripheralGuard *guard) {
  if (guard == NULL) {
    return;
  }
  uint32_t now_tick = HAL_GetTick();
  uint32_t primask = PeripheralEnterCritical();
  if (guard->state == kPeripheralStateCalibrating ||
      guard->state == kPeripheralStateProbing) {
    guard->last_state = guard->state;
    guard->state = kPeripheralStateDisabled;
    guard->state_enter_tick = now_tick;
    guard->has_state_enter_tick = true;
  }
  PeripheralExitCritical(primask);
}

bool PeripheralGuardEnable(PeripheralGuard *guard) {
  if (guard == NULL) {
    return false;
  }

  uint32_t primask = PeripheralEnterCritical();
  bool can_enable =
      (guard->state == kPeripheralStateDisabled && guard->error_code == 0U);
  PeripheralExitCritical(primask);

  // 仅在安全待命态且无故障时允许切入使能态
  if (!can_enable) {
    return false;
  }

  // 若注册了使能回调，在临界区外执行硬件使能指令（如达妙 0xFC）
  if (guard->ops.on_enable != NULL) {
    if (!guard->ops.on_enable(guard->device_ptr)) {
      return false;
    }
  }

  uint32_t now_tick = HAL_GetTick();
  primask = PeripheralEnterCritical();
  bool still_valid =
      (guard->state == kPeripheralStateDisabled && guard->error_code == 0U);
  if (still_valid) {
    guard->last_state = guard->state;
    guard->state = kPeripheralStateEnabled;
    guard->state_enter_tick = now_tick;
    guard->has_state_enter_tick = true;
  }
  PeripheralExitCritical(primask);

  if (!still_valid && guard->ops.on_disable != NULL) {
    guard->ops.on_disable(guard->device_ptr);
  }
  return still_valid;
}

bool PeripheralGuardDisable(PeripheralGuard *guard) {
  if (guard == NULL) {
    return false;
  }

  uint32_t now_tick = HAL_GetTick();
  uint32_t primask = PeripheralEnterCritical();
  if (guard->state == kPeripheralStateEnabled) {
    guard->last_state = guard->state;
    guard->state = kPeripheralStateDisabled;
    guard->state_enter_tick = now_tick;
    guard->has_state_enter_tick = true;
  }
  PeripheralExitCritical(primask);

  if (guard->ops.on_disable != NULL) {
    guard->ops.on_disable(guard->device_ptr);
  }
  return true;
}

void PeripheralGuardTriggerFault(PeripheralGuard *guard, uint32_t error_code) {
  if (guard == NULL) {
    return;
  }

  uint32_t now_tick = HAL_GetTick();
  uint32_t primask = PeripheralEnterCritical();
  guard->error_code = (error_code != 0U) ? error_code : 1U;
  if (guard->state != kPeripheralStateFault) {
    guard->last_state = guard->state;
    guard->state = kPeripheralStateFault;
    guard->state_enter_tick = now_tick;
    guard->has_state_enter_tick = true;
  }
  PeripheralExitCritical(primask);

  // 故障发生，强制执行安全切断动作
  if (guard->ops.on_disable != NULL) {
    guard->ops.on_disable(guard->device_ptr);
  } else if (guard->ops.on_offline != NULL) {
    guard->ops.on_offline(guard->device_ptr);
  }
}

bool PeripheralGuardClearFault(PeripheralGuard *guard) {
  if (guard == NULL) {
    return false;
  }

  uint32_t primask = PeripheralEnterCritical();
  bool is_fault = (guard->state == kPeripheralStateFault);
  PeripheralExitCritical(primask);

  if (!is_fault) {
    return true;
  }

  // 调用硬件专属清错操作（如达妙 0xFB 清错帧）
  if (guard->ops.on_clear_fault != NULL) {
    if (!guard->ops.on_clear_fault(guard->device_ptr)) {
      return false; // 清错指令未获确认，保持故障态
    }
  }

  // 故障清除成功，必须退回 Disabled 待命态（严禁直接使能）
  uint32_t now_tick = HAL_GetTick();
  primask = PeripheralEnterCritical();
  if (guard->state == kPeripheralStateFault) {
    guard->error_code = 0U;
    guard->last_state = guard->state;
    guard->state = kPeripheralStateDisabled;
    guard->state_enter_tick = now_tick;
    guard->has_state_enter_tick = true;
  }
  PeripheralExitCritical(primask);
  return true;
}

const char *PeripheralGetStateName(PeripheralState state) {
  switch (state) {
    case kPeripheralStateUninit:
      return "Uninitialized";
    case kPeripheralStateProbing:
      return "Probing";
    case kPeripheralStateCalibrating:
      return "Calibrating";
    case kPeripheralStateDisabled:
      return "Disabled";
    case kPeripheralStateEnabled:
      return "Enabled";
    case kPeripheralStateOffline:
      return "Offline";
    case kPeripheralStateFault:
      return "Fault";
    default:
      return "Unknown";
  }
}

/* ========================================================================= */
/* 全局外设集中管理器 (PeripheralManager) 实现                                */
/* ========================================================================= */

typedef struct {
  PeripheralGuard *guards[kPeripheralMaxRegisteredCount];
  bool is_critical[kPeripheralMaxRegisteredCount];
  uint32_t count;
  bool initialized;
} PeripheralRegistry;

static PeripheralRegistry g_peripheral_registry = {0};

void PeripheralManagerInit(void) {
  uint32_t primask = PeripheralEnterCritical();
  g_peripheral_registry.initialized = true;
  PeripheralExitCritical(primask);
}

void PeripheralManagerReset(void) {
  uint32_t primask = PeripheralEnterCritical();
  memset(&g_peripheral_registry, 0, sizeof(PeripheralRegistry));
  g_peripheral_registry.initialized = true;
  PeripheralExitCritical(primask);
}

bool PeripheralManagerRegister(PeripheralGuard *guard, bool is_critical) {
  if (guard == NULL) {
    return false;
  }

  uint32_t primask = PeripheralEnterCritical();

  // 避免重复注册
  for (uint32_t i = 0; i < g_peripheral_registry.count; ++i) {
    if (g_peripheral_registry.guards[i] == guard) {
      g_peripheral_registry.is_critical[i] = is_critical;
      PeripheralExitCritical(primask);
      return true;
    }
  }

  if (g_peripheral_registry.count >= kPeripheralMaxRegisteredCount) {
    PeripheralExitCritical(primask);
    return false;
  }

  uint32_t idx = g_peripheral_registry.count;
  g_peripheral_registry.guards[idx] = guard;
  g_peripheral_registry.is_critical[idx] = is_critical;
  g_peripheral_registry.count++;
  PeripheralExitCritical(primask);
  return true;
}

void PeripheralManagerUpdateAll(uint32_t now_tick) {
  for (uint32_t i = 0; i < g_peripheral_registry.count; ++i) {
    PeripheralGuard *g = g_peripheral_registry.guards[i];
    if (g != NULL) {
      PeripheralGuardUpdate(g, now_tick);
    }
  }
}

bool PeripheralManagerIsAllCriticalOnline(void) {
  for (uint32_t i = 0; i < g_peripheral_registry.count; ++i) {
    if (g_peripheral_registry.is_critical[i]) {
      const PeripheralGuard *g = g_peripheral_registry.guards[i];
      if (g == NULL || !PeripheralIsOnline(g)) {
        return false;
      }
    }
  }
  return true;
}

bool PeripheralManagerHasCriticalFault(void) {
  for (uint32_t i = 0; i < g_peripheral_registry.count; ++i) {
    if (g_peripheral_registry.is_critical[i]) {
      const PeripheralGuard *g = g_peripheral_registry.guards[i];
      if (g != NULL && PeripheralIsFault(g)) {
        return true;
      }
    }
  }
  return false;
}

void PeripheralManagerEmergencyStopAll(void) {
  for (uint32_t i = 0; i < g_peripheral_registry.count; ++i) {
    PeripheralGuard *g = g_peripheral_registry.guards[i];
    if (g != NULL && g->state == kPeripheralStateEnabled) {
      PeripheralGuardDisable(g);
    }
  }
}

uint32_t PeripheralManagerGetAbnormalCount(void) {
  uint32_t count = 0U;
  for (uint32_t i = 0; i < g_peripheral_registry.count; ++i) {
    const PeripheralGuard *g = g_peripheral_registry.guards[i];
    if (g != NULL) {
      if (g->state == kPeripheralStateOffline || g->state == kPeripheralStateFault) {
        count++;
      }
    }
  }
  return count;
}
