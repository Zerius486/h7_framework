/**
 * @file app_config.c
 * @brief 机器人配置及初始化实现
 * @attention
 * 本文件与 app_robot.c、app_protocol.h/.c 平级，按机器人需求配置设备和算法。
 */

#include "app_config.h"

#include <stddef.h>

#include "usart.h"
#include "app_tune_log.h"

const AppRobotConfig g_app_robot_config = {
    .board_role = kAppBoardSingle,
    .chassis_source = kAppControlRemote,
    .gimbal_source = kAppControlRemote,
    .shoot_source = kAppControlRemote,
};

const AppTaskConfig g_app_task_config = {
    .manager_period_ms = 1U,
    .gimbal_period_ms = 1U,
    .chassis_period_ms = 1U,
    .shoot_period_ms = 1U,
};

const AppCommunicationConfig g_app_communication_config = {
    .remote_huart = NULL,
    .host_huart = NULL,
    .vision_huart = &huart10,
    .referee_rx_huart = NULL,
    .referee_tx_huart = NULL,
    .board_link_hfdcan = NULL,
};

const AppChassisConfig g_app_chassis_config = {
    .type = kSteerWheel,
    .params = {0.0f, 0.0f},
    .wheel_radius = 0.0f,
    .max_speed = 0.0f,
    .max_omega = 0.0f,
};

const AppGimbalConfig g_app_gimbal_config = {
    .yaw_min = 0.0f,
    .yaw_max = 0.0f,
    .pitch_min = 0.0f,
    .pitch_max = 0.0f,
};

const AppShootConfig g_app_shoot_config = {
    .friction_omega = 0.0f,
    .feed_angle = 0.0f,
    .max_frequency = 0.0f,
};

const DebugLogConfig g_app_debug_log_config = {
    .huart = &huart7,
    .enabled = false,
    .binary_enabled = true,
    .text_enabled = true,
    .sample_period = 1U,
    .baudrate = 921600U,
};

/**
 * @brief 初始化设备及通信，在外设初始化后调用一次
 */
void AppConfigInit(void)
{
  AppTuneLogInit(&g_app_debug_log_config);
}
