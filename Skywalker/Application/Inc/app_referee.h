#ifndef APP_REFEREE_H
#define APP_REFEREE_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp_uart.h"

enum {
  kRefereeSof = 0xA5,                          // 裁判系统帧头
  kRefereeHeaderLength = 5,                    // 帧头长度
  kRefereeCmdIdLength = 2,                     // 命令码长度
  kRefereeTailLength = 2,                      // 帧尾CRC16长度
  kRefereeFrameMinLength = 9,                  // 最小帧长度
  kRefereeDataOffset = 7,                      // 数据段起始偏移
  kRefereeQueueSize = 10,                      // 解包后帧队列深度
  kRefereeRxStreamSize = kUartBufferSize * 2,  // 流式接收缓存长度
  kRefereeMaxInteractiveDataLength = 113,      // 学生交互最大数据长度
};

// 裁判系统命令码
typedef enum {
  kRefereeCmdGameState = 0x0001,               // 比赛状态
  kRefereeCmdGameResult = 0x0002,              // 比赛结果
  kRefereeCmdGameRobotHp = 0x0003,             // 机器人血量
  kRefereeCmdEventData = 0x0101,               // 场地事件
  kRefereeCmdSupplyProjectileAction = 0x0102,  // 补给站动作
  kRefereeCmdGameRobotState = 0x0201,          // 机器人状态
  kRefereeCmdPowerHeatData = 0x0202,           // 功率热量
  kRefereeCmdGameRobotPosition = 0x0203,       // 机器人位置
  kRefereeCmdBuff = 0x0204,                    // 机器人增益
  kRefereeCmdAerialRobotEnergy = 0x0205,       // 空中机器人能量
  kRefereeCmdRobotHurt = 0x0206,               // 受击信息
  kRefereeCmdShootData = 0x0207,               // 射击信息
  kRefereeCmdProjectileAllowance = 0x0208,     // 弹丸允许量
  kRefereeCmdStudentInteractive = 0x0301,      // 学生机器人交互
} RefereeCommandId;

#pragma pack(push, 1)

typedef struct {
  uint8_t sof;           // 起始字节，固定0xA5
  uint16_t data_length;  // 数据段长度
  uint8_t sequence;      // 包序号
  uint8_t crc8;          // 帧头CRC8
} RefereeFrameHeader;

typedef struct {
  uint8_t game_type : 4;       // 比赛类型
  uint8_t game_progress : 4;   // 当前比赛阶段
  uint16_t stage_remain_time;  // 当前阶段剩余时间（s）
} RefereeGameState;

typedef struct {
  uint8_t winner;  // 获胜方
} RefereeGameResult;

typedef struct {
  uint16_t red_1_robot_hp;   // 红1英雄血量
  uint16_t red_2_robot_hp;   // 红2工程血量
  uint16_t red_3_robot_hp;   // 红3步兵血量
  uint16_t red_4_robot_hp;   // 红4步兵血量
  uint16_t red_5_robot_hp;   // 红5步兵血量
  uint16_t red_7_robot_hp;   // 红7哨兵血量
  uint16_t red_outpost_hp;   // 红方前哨站血量
  uint16_t red_base_hp;      // 红方基地血量
  uint16_t blue_1_robot_hp;  // 蓝1英雄血量
  uint16_t blue_2_robot_hp;  // 蓝2工程血量
  uint16_t blue_3_robot_hp;  // 蓝3步兵血量
  uint16_t blue_4_robot_hp;  // 蓝4步兵血量
  uint16_t blue_5_robot_hp;  // 蓝5步兵血量
  uint16_t blue_7_robot_hp;  // 蓝7哨兵血量
  uint16_t blue_outpost_hp;  // 蓝方前哨站血量
  uint16_t blue_base_hp;     // 蓝方基地血量
} RefereeGameRobotHp;

typedef struct {
  uint32_t event_type;  // 场地事件位标志
} RefereeEventData;

typedef struct {
  uint8_t supply_projectile_id;    // 补给站口ID
  uint8_t supply_robot_id;         // 补给机器人ID
  uint8_t supply_projectile_step;  // 补弹步骤
  uint8_t supply_projectile_num;   // 补弹数量
} RefereeSupplyProjectileAction;

typedef struct {
  uint8_t robot_id;                         // 本机器人ID
  uint8_t robot_level;                      // 机器人等级
  uint16_t remain_hp;                       // 当前血量
  uint16_t max_hp;                          // 血量上限
  uint16_t shooter_id1_17mm_cooling_rate;   // 1号17mm枪口冷却速率
  uint16_t shooter_id1_17mm_cooling_limit;  // 1号17mm枪口热量上限
  uint16_t shooter_id1_17mm_speed_limit;    // 1号17mm枪口初速上限
  uint16_t shooter_id2_17mm_cooling_rate;   // 2号17mm枪口冷却速率
  uint16_t shooter_id2_17mm_cooling_limit;  // 2号17mm枪口热量上限
  uint16_t shooter_id2_17mm_speed_limit;    // 2号17mm枪口初速上限
  uint16_t shooter_id1_42mm_cooling_rate;   // 42mm枪口冷却速率
  uint16_t shooter_id1_42mm_cooling_limit;  // 42mm枪口热量上限
  uint16_t shooter_id1_42mm_speed_limit;    // 42mm枪口初速上限
  uint16_t chassis_power_limit;             // 底盘功率上限
  uint8_t mains_power_gimbal_output : 1;    // 云台电源输出状态
  uint8_t mains_power_chassis_output : 1;   // 底盘电源输出状态
  uint8_t mains_power_shooter_output : 1;   // 发射机构电源输出状态
} RefereeGameRobotState;

typedef struct {
  uint16_t chassis_voltage;       // 底盘电压（mV）
  uint16_t chassis_current;       // 底盘电流（mA）
  float chassis_power;            // 底盘功率（W）
  uint16_t chassis_power_buffer;  // 底盘功率缓冲（J）
  uint16_t shooter_heat_17mm_1;   // 1号17mm枪口热量
  uint16_t shooter_heat_17mm_2;   // 2号17mm枪口热量
} RefereePowerHeatData;

typedef struct {
  float x;    // 机器人x坐标（m）
  float y;    // 机器人y坐标（m）
  float z;    // 机器人z坐标（m）
  float yaw;  // 机器人朝向（rad）
} RefereeGameRobotPosition;

typedef struct {
  uint8_t power_rune_buff;  // 机器人增益状态
} RefereeBuff;

typedef struct {
  uint8_t attack_time;  // 空中机器人可攻击时间（s）
} RefereeAerialRobotEnergy;

typedef struct {
  uint8_t armor_id : 4;   // 受击装甲板ID
  uint8_t hurt_type : 4;  // 受击类型
} RefereeRobotHurt;

typedef struct {
  uint8_t bullet_type;       // 弹丸类型
  uint8_t shooter_id;        // 发射机构ID
  uint8_t bullet_frequency;  // 射频（Hz）
  float bullet_speed;        // 弹丸初速（m/s）
} RefereeShootData;

typedef struct {
  uint16_t projectile_allowance_17mm;      // 17mm弹丸允许发弹量
  uint16_t projectile_allowance_42mm;      // 42mm弹丸允许发弹量
  uint16_t remaining_gold_coin;            // 剩余金币
  uint16_t fortress_projectile_allowance;  // 堡垒弹丸允许发弹量
} RefereeProjectileAllowance;

typedef struct {
  uint16_t data_cmd_id;  // 交互数据内容ID
  uint16_t sender_id;    // 发送方ID
  uint16_t receiver_id;  // 接收方ID
} RefereeInteractiveHeader;

#pragma pack(pop)

typedef struct {
  uint8_t data[kRefereeMaxInteractiveDataLength];  // 交互数据内容
  uint16_t length;                                 // 交互数据长度
} RefereeInteractiveData;

typedef struct {
  uint8_t buffer[kUartBufferSize];  // 完整裁判系统帧
  uint16_t length;                  // 帧长度
} RefereePacket;

typedef struct {
  RefereePacket packets[kRefereeQueueSize];  // 帧队列缓存
  volatile uint16_t head;                    // 读指针
  volatile uint16_t tail;                    // 写指针
} RefereeQueue;

typedef struct {
  UART_HandleTypeDef *rx_huart;             // 接收串口
  UART_HandleTypeDef *tx_huart;             // 发送串口
  RefereeQueue queue;                       // 完整帧队列
  uint8_t rx_stream[kRefereeRxStreamSize];  // 流式接收缓存
  uint16_t rx_stream_length;                // 流式接收缓存有效长度
  uint8_t tx_sequence;                      // 发送包序号
  uint32_t received_frame_count;            // 已接收有效帧计数
  uint32_t dropped_frame_count;             // 丢弃帧计数

  // 最近一次解析出的各类裁判系统数据。
  RefereeGameState game_state;
  RefereeGameResult game_result;
  RefereeGameRobotHp game_robot_hp;
  RefereeEventData event_data;
  RefereeSupplyProjectileAction supply_projectile_action;
  RefereeGameRobotState game_robot_state;
  RefereePowerHeatData power_heat_data;
  RefereeGameRobotPosition game_robot_position;
  RefereeBuff buff;
  RefereeAerialRobotEnergy aerial_robot_energy;
  RefereeRobotHurt robot_hurt;
  RefereeShootData shoot_data;
  RefereeProjectileAllowance projectile_allowance;
  RefereeInteractiveHeader interactive_header;
  RefereeInteractiveData interactive_data;
} RefereeObject;

extern RefereeObject g_referee;

// API函数声明
void RefereeInit(RefereeObject *referee, UART_HandleTypeDef *rx_huart,
                 UART_HandleTypeDef *tx_huart);
void RefereeRxCallback(RefereeObject *referee, uint8_t *rx_buffer,
                       uint16_t rx_length);
void RefereeUartRxCallback(uint8_t *rx_buffer, uint16_t rx_length);
void RefereeUpdate(RefereeObject *referee);
bool RefereePackFrame(RefereeObject *referee, uint16_t cmd_id,
                      const uint8_t *data, uint16_t data_length,
                      uint8_t *tx_buffer, uint16_t tx_buffer_length,
                      uint16_t *frame_length);
bool RefereeSend(RefereeObject *referee, uint16_t cmd_id, const uint8_t *data,
                 uint16_t data_length);

#endif  // APP_REFEREE_H
