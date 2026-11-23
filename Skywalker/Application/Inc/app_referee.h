#ifndef APP_REFEREE_H
#define APP_REFEREE_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp_stream.h"
#include "bsp_uart.h"
#include "dev_peripheral.h"

// 裁判系统协议模式开关：默认 RMUL（高校联盟赛），可通过 CMake -DREFEREE_MODE=RMUC 切换
#if defined(REFEREE_MODE_RMUC) && (REFEREE_MODE_RMUC != 0) && \
    defined(REFEREE_MODE_RMUL) && (REFEREE_MODE_RMUL != 0)
#error "REFEREE_MODE_RMUC and REFEREE_MODE_RMUL cannot both be enabled."
#endif

#if (!defined(REFEREE_MODE_RMUC) || (REFEREE_MODE_RMUC == 0)) && \
    (!defined(REFEREE_MODE_RMUL) || (REFEREE_MODE_RMUL == 0))
#undef REFEREE_MODE_RMUL
#define REFEREE_MODE_RMUL 1
#endif

enum
{
  kRefereeSof = 0xA5,                         // 裁判系统帧头
  kRefereeHeaderLength = 5,                   // 帧头长度
  kRefereeCmdIdLength = 2,                    // 命令码长度
  kRefereeTailLength = 2,                     // 帧尾CRC16长度
  kRefereeFrameMinLength = 9,                 // 最小帧长度
  kRefereeDataOffset = 7,                     // 数据段起始偏移
  kRefereeQueueSize = 10,                     // 解包后帧队列深度
  kRefereeRxStreamSize = kUartBufferSize * 2, // 流式接收缓存长度
  kRefereeMaxInteractiveDataLength = 112,     // 0x0301 机器人交互最大内容长度
  kRefereeCustomDataMaxLength = 300,          // 0x0310 图传链路自定义数据最大长度
  kRefereeCustomCmdMaxLength = 60,            // 0x0311 图传链路自定义控制最大长度
};

// 裁判系统命令码（RoboMaster 2027 通信协议 V1.0.0）
typedef enum
{
  kRefereeCmdGameState = 0x0001,           // 比赛状态 (11B)
  kRefereeCmdGameResult = 0x0002,          // 比赛结果 (1B)
  kRefereeCmdGameRobotHp = 0x0003,         // 机器人血量 (24B)
  kRefereeCmdEventData = 0x0101,           // 场地事件 (8B, RMUC/RMUL 位域不同)
  kRefereeCmdRefereeWarning = 0x0104,      // 裁判警告 (3B)
  kRefereeCmdDartInfo = 0x0105,            // 飞镖发射相关数据 (3B, RMUC)
  kRefereeCmdGameRobotState = 0x0201,      // 机器人性能体系数据 (17B)
  kRefereeCmdPowerHeatData = 0x0202,       // 实时底盘缓冲能量与枪口热量 (14B)
  kRefereeCmdGameRobotPosition = 0x0203,   // 机器人位置 (12B)
  kRefereeCmdBuff = 0x0204,                // 机器人增益和底盘能量 (7B)
  kRefereeCmdRobotHurt = 0x0206,           // 伤害状态 (7B)
  kRefereeCmdShootData = 0x0207,           // 实时射击数据 (7B)
  kRefereeCmdProjectileAllowance = 0x0208, // 允许发弹量 (8B)
  kRefereeCmdRfidStatus = 0x0209,          // 机器人 RFID 模块状态 (6B, RMUC/RMUL 位域不同)
  kRefereeCmdDartClientCmd = 0x020A,       // 飞镖选手端指令数据 (6B, RMUC)
  kRefereeCmdGroundRobotPosition = 0x020B, // 地面机器人位置数据 (40B)
  kRefereeCmdRadarMarkData = 0x020C,       // 雷达标记进度数据 (2B, RMUC)
  kRefereeCmdSentryInfo = 0x020D,          // 哨兵自主决策信息同步 (14B)
  kRefereeCmdRadarInfo = 0x020E,           // 雷达自主决策信息同步 (8B, RMUC)
  kRefereeCmdStudentInteractive = 0x0301,  // 机器人交互数据 (6 + n <= 118B)
  kRefereeCmdMapCommand = 0x0303,          // 选手端小地图交互数据 (12B)
  kRefereeCmdMapRobotData = 0x0305,        // 选手端小地图接收雷达数据 (48B)
  kRefereeCmdMapData = 0x0307,             // 选手端小地图接收路径数据 (105B)
  kRefereeCmdCustomInfo = 0x0308,          // 选手端小地图接收机器人消息 (34B)
  kRefereeCmdCustomRobotData = 0x0310,     // 机器人可通过图传链路向自定义客户端发送的数据 (<= 300B)
  kRefereeCmdCustomRobotCmd = 0x0311,      // 自定义客户端可通过图传链路向机器人发送的控制指令 (<= 60B)
} RefereeCommandId;

// 0x0301 机器人交互子内容 ID
typedef enum
{
  kRefereeSubCmdDeleteLayer = 0x0100,   // 选手端删除图层 (2B)
  kRefereeSubCmdDraw1Figure = 0x0101,   // 选手端绘制 1 个图形 (15B)
  kRefereeSubCmdDraw2Figures = 0x0102,  // 选手端绘制 2 个图形 (30B)
  kRefereeSubCmdDraw5Figures = 0x0103,  // 选手端绘制 5 个图形 (75B)
  kRefereeSubCmdDraw7Figures = 0x0104,  // 选手端绘制 7 个图形 (105B)
  kRefereeSubCmdDrawCharacter = 0x0110, // 选手端绘制字符图形 (45B)
  kRefereeSubCmdSentryCmd = 0x0120,      // 哨兵自主决策指令 (4B)
  kRefereeSubCmdRadarCmd = 0x0121,       // 雷达自主决策指令 (13B)
} RefereeInteractiveSubCmdId;

#pragma pack(push, 1)

typedef struct
{
  uint8_t sof;          // 起始字节，固定0xA5
  uint16_t data_length; // 数据段长度
  uint8_t sequence;     // 包序号
  uint8_t crc8;         // 帧头CRC8
} RefereeFrameHeader;

// 0x0001 (11B)
typedef struct
{
  uint8_t game_type : 4;      // 比赛类型 (1:RMUC, 2:单项, 3:ICRA, 4:RMUL 3V3, 5:RMUL 步兵)
  uint8_t game_progress : 4;  // 当前比赛阶段
  uint16_t stage_remain_time; // 当前阶段剩余时间 (s)
  uint64_t sync_timestamp;    // UNIX 时间戳
} RefereeGameState;

// 0x0002 (1B)
typedef struct
{
  uint8_t winner; // 0:平局, 1:红方胜, 2:蓝方胜
} RefereeGameResult;

// 0x0003 (24B)
typedef struct
{
  uint16_t ally_1_robot_hp;     // 己方 1 号英雄血量
  uint16_t reserved;            // 保留
  uint16_t ally_3_robot_hp;     // 己方 3 号步兵血量
  uint16_t ally_4_robot_hp;     // 己方 4 号步兵血量
  int16_t damage_difference;    // 己方与对方累积伤害差
  uint16_t ally_7_robot_hp;     // 己方 7 号哨兵血量
  uint16_t outpost_r_hp;        // 红方前哨站血量
  uint16_t ally_base_hp;        // 己方基地血量
  uint16_t outpost_b_hp;        // 蓝方前哨站血量
  uint16_t enemy_base_hp;       // 对方基地血量
  uint16_t ally_control_degree; // 己方控制度
  uint16_t enemy_control_degree;// 对方控制度
} RefereeGameRobotHp;

// 0x0101 (8B)
typedef union
{
  struct
  {
    uint32_t event_data;   // 场地事件数据 1
    uint32_t event_data_2; // 场地事件数据 2
  } raw;
#if defined(REFEREE_MODE_RMUC) && (REFEREE_MODE_RMUC != 0)
  struct
  {
    uint32_t supply_zone : 1;           // bit0: 己方补给区占领状态
    uint32_t small_rune : 2;            // bit1-2: 小能量机关激活状态
    uint32_t large_rune : 2;            // bit3-4: 大能量机关激活状态
    uint32_t fortress_r : 3;            // bit5-7: 红方堡垒占领/增益状态
    uint32_t fortress_b : 3;            // bit8-10: 蓝方堡垒占领/增益状态
    uint32_t base_r : 3;                // bit11-13: 红方基地虚拟护盾/状态
    uint32_t base_b : 3;                // bit14-16: 蓝方基地虚拟护盾/状态
    uint32_t highland_command : 1;      // bit17: 高地制高点状态
    uint32_t center_highland_r : 2;     // bit18-19: 红方中央高地占领状态
    uint32_t center_highland_b : 2;     // bit20-21: 蓝方中央高地占领状态
    uint32_t outpost_r_buff : 3;        // bit22-24: 红方前哨站增益状态
    uint32_t outpost_b_buff : 3;        // bit25-27: 蓝方前哨站增益状态
    uint32_t outpost_r_ctrl : 2;        // bit28-29: 红方前哨站控制状态
    uint32_t outpost_b_ctrl : 2;        // bit30-31: 蓝方前哨站控制状态

    uint32_t base_r_ctrl : 2;           // bit0-1: 红方基地控制状态
    uint32_t base_b_ctrl : 2;           // bit2-3: 蓝方基地控制状态
    uint32_t enemy_dart_hit_time : 9;   // bit4-12: 敌方飞镖最后一次击中己方时间 (0~420s)
    uint32_t enemy_dart_hit_target : 3; // bit13-15: 敌方飞镖最后一次击中目标
    uint32_t reserved : 16;             // bit16-31: 保留
  } bits;
#elif defined(REFEREE_MODE_RMUL) && (REFEREE_MODE_RMUL != 0)
  struct
  {
    uint32_t supply_zone : 1;      // bit0: 己方补给区占领状态
    uint32_t reserved1 : 31;       // bit1-31: 保留

    uint32_t reserved2 : 16;       // bit0-15: 保留
    uint32_t highland_r_buff : 2;  // bit16-17: 红方高地增益区状态 (仅RMUL)
    uint32_t highland_r_ctrl : 2;  // bit18-19: 红方高地控制状态 (仅RMUL)
    uint32_t highland_b_buff : 2;  // bit20-21: 蓝方高地增益区状态 (仅RMUL)
    uint32_t highland_b_ctrl : 2;  // bit22-23: 蓝方高地控制状态 (仅RMUL)
    uint32_t center_ctrl_buff : 2; // bit24-25: 中心增益区增益状态 (仅RMUL)
    uint32_t center_ctrl : 2;      // bit26-27: 中心增益区占领状态 (仅RMUL)
    uint32_t reserved3 : 4;        // bit28-31: 保留
  } bits;
#endif
} RefereeEventData;

// 0x0104 (3B)
typedef struct
{
  uint8_t level;              // 己方最后一次受到判罚的等级
  uint8_t offending_robot_id; // 己方最后一次受到判罚的违规机器人 ID
  uint8_t count;              // 对应判罚等级的违规次数
} RefereeWarning;

#if defined(REFEREE_MODE_RMUC) && (REFEREE_MODE_RMUC != 0)
// 0x0105 (3B, RMUC)
typedef struct
{
  uint8_t dart_remaining_time; // 己方飞镖发射剩余时间 (s)
  uint16_t dart_info;          // 飞镖目标与命中信息位域
} RefereeDartInfo;
#endif

// 0x0201 (17B)
typedef struct
{
  uint8_t robot_id;                             // 本机器人 ID
  uint8_t robot_level;                          // 机器人等级
  uint16_t current_hp;                          // 机器人当前血量
  uint16_t maximum_hp;                          // 机器人血量上限
  uint16_t shooter_barrel_cooling_value;        // 机器人枪口每秒冷却值
  uint16_t shooter_barrel_heat_limit;           // 机器人枪口热量上限
  uint16_t chassis_power_limit;                 // 机器人底盘功率上限
  float bullet_speed_limit;                     // 机器人弹速上限 (m/s)
  uint8_t power_management_gimbal_output : 1;   // gimbal 口输出 (1:24V, 0:无)
  uint8_t power_management_chassis_output : 1;  // chassis 口输出 (1:24V, 0:无)
  uint8_t power_management_shooter_output : 1;  // shooter 口输出 (1:24V, 0:无)
  uint8_t reserved : 5;                         // 保留位
} RefereeGameRobotState;

// 0x0202 (14B)
typedef struct
{
  uint16_t reserved1;               // 保留
  uint16_t reserved2;               // 保留
  float reserved3;                  // 保留
  uint16_t buffer_energy;           // 缓冲能量 (J)
  uint16_t shooter_17mm_barrel_heat;// 17mm 枪口热量
  uint16_t shooter_42mm_barrel_heat;// 42mm 枪口热量
} RefereePowerHeatData;

// 0x0203 (12B)
typedef struct
{
  float x;     // 本机器人位置 x 坐标 (m)
  float y;     // 本机器人位置 y 坐标 (m)
  float angle; // 本机器人测速模块朝向 (度, 正北为 0)
} RefereeGameRobotPosition;

// 0x0204 (7B)
typedef struct
{
  uint8_t recovery_buff;      // 机器人回血增益 (%)
  uint16_t cooling_buff;      // 机器人枪口冷却增益 (%)
  uint8_t defence_buff;       // 机器人防御增益 (%)
  uint8_t vulnerability_buff; // 机器人负防御增益 (%)
  uint16_t attack_buff;       // 机器人攻击增益 (%)
} RefereeBuff;

// 0x0206 (7B)
typedef struct
{
  uint8_t armor_id : 4;            // 受击装甲板 ID
  uint8_t hp_deduction_reason : 4; // 血量变化类型
  uint16_t hit_sequence;           // 击中序号
  int16_t hit_position_x;          // 弹丸击中位置 x 坐标
  int16_t hit_position_y;          // 弹丸击中位置 y 坐标
} RefereeRobotHurt;

// 0x0207 (7B)
typedef struct
{
  uint8_t bullet_type;         // 弹丸类型 (1:17mm, 2:42mm)
  uint8_t shooter_number;      // 发射机构 ID
  uint8_t launching_frequency; // 弹丸射频 (Hz)
  float initial_speed;         // 弹丸初速 (m/s)
} RefereeShootData;

// 0x0208 (8B)
typedef struct
{
  uint16_t projectile_allowance_17mm;     // 17mm 弹丸允许发弹量
  uint16_t projectile_allowance_42mm;     // 42mm 弹丸允许发弹量
  uint16_t remaining_gold_coin;           // 剩余金币数量
  uint16_t projectile_allowance_fortress; // 堡垒增益点提供的储备 17mm 弹丸允许发弹量
} RefereeProjectileAllowance;

// 0x0209 (6B)
typedef union
{
  struct
  {
    uint32_t rfid_status;   // RFID 状态字 1
    uint16_t rfid_status_2; // RFID 状态字 2
  } raw;
#if defined(REFEREE_MODE_RMUC) && (REFEREE_MODE_RMUC != 0)
  struct
  {
    uint32_t supply_zone : 1;        // bit0: 己方补给区
    uint32_t uc_zones_1_10 : 10;     // bit1-10: RMUC 装配/基地/堡垒/中央高地/前哨站/高地等增益点
    uint32_t reserved_ul : 3;        // bit11-13: 保留 (RMUL 专用位)
    uint32_t terrain_cross_low : 17; // bit14-30: 地形跨越增益点低位
    uint32_t reserved : 1;           // bit31: 保留

    uint16_t terrain_cross_high : 9; // bit0-8: 地形跨越增益点高位
    uint16_t reserved2 : 7;          // bit9-15: 保留
  } bits;
#elif defined(REFEREE_MODE_RMUL) && (REFEREE_MODE_RMUL != 0)
  struct
  {
    uint32_t supply_zone : 1;        // bit0: 己方补给区
    uint32_t reserved_uc : 10;       // bit1-10: 保留 (RMUC 专用位)
    uint32_t highland_r : 1;         // bit11: 红方高地增益区 (仅RMUL)
    uint32_t highland_b : 1;         // bit12: 蓝方高地增益区 (仅RMUL)
    uint32_t center_ctrl : 1;        // bit13: 中心增益区 (仅RMUL)
    uint32_t terrain_cross_low : 17; // bit14-30: 地形跨越增益点低位
    uint32_t reserved : 1;           // bit31: 保留

    uint16_t terrain_cross_high : 9; // bit0-8: 地形跨越增益点高位
    uint16_t reserved2 : 7;          // bit9-15: 保留
  } bits;
#endif
} RefereeRfidStatus;

#if defined(REFEREE_MODE_RMUC) && (REFEREE_MODE_RMUC != 0)
// 0x020A (6B, RMUC)
typedef struct
{
  uint8_t dart_launch_opening_status; // 飞镖发射站状态
  uint8_t reserved;                   // 保留
  uint16_t target_change_time;        // 切换击打目标时的比赛剩余时间 (s)
  uint16_t latest_launch_cmd_time;    // 最近一次操作手确定发射指令时的比赛剩余时间 (s)
} RefereeDartClientCmd;
#endif

// 0x020B (40B)
typedef struct
{
  float heavyengineer_x; // 己方工程/重型工程位置 x 坐标 (m)
  float heavyengineer_y; // 己方工程/重型工程位置 y 坐标 (m)
  float reserved1;       // 保留
  float reserved2;       // 保留
  float standard_3_x;    // 己方 3 号步兵位置 x 坐标 (m)
  float standard_3_y;    // 己方 3 号步兵位置 y 坐标 (m)
  float standard_4_x;    // 己方 4 号步兵位置 x 坐标 (m)
  float standard_4_y;    // 己方 4 号步兵位置 y 坐标 (m)
  float reserved3;       // 保留
  float reserved4;       // 保留
} RefereeGroundRobotPosition;

#if defined(REFEREE_MODE_RMUC) && (REFEREE_MODE_RMUC != 0)
// 0x020C (2B, RMUC)
typedef struct
{
  uint16_t mark_progress; // 对方机器人被标记进度
} RefereeRadarMarkData;
#endif

// 0x020D (14B)
typedef struct
{
  uint32_t sentry_info;   // 哨兵自主决策信息 1
  uint16_t sentry_info_2; // 哨兵自主决策信息 2
  uint64_t sentry_info_3; // 哨兵自主决策信息 3
} RefereeSentryInfo;

#if defined(REFEREE_MODE_RMUC) && (REFEREE_MODE_RMUC != 0)
// 0x020E (8B, RMUC)
typedef struct
{
  uint8_t radar_info;        // 雷达自主决策信息 1
  uint8_t radar_info_2;      // 雷达自主决策信息 2
  uint16_t fortify_center_x; // 设防区中心 x 坐标
  uint16_t fortify_center_y; // 设防区中心 y 坐标
  uint16_t fortify_radius;   // 设防区半径
} RefereeRadarInfo;
#endif

// 0x0301 头部 (6B)
typedef struct
{
  uint16_t data_cmd_id; // 子内容 ID
  uint16_t sender_id;   // 发送者 ID
  uint16_t receiver_id; // 接收者 ID
} RefereeInteractiveHeader;

// 0x0301 子命令 0x0100: 删除图层 (2B)
typedef struct
{
  uint8_t delete_type; // 删除操作 (0:空, 1:删除图层, 2:删除所有)
  uint8_t layer;       // 图层数 (0~9)
} RefereeInteractionLayerDelete;

// 0x0301 子命令 0x0101 图形元素 (15B)
typedef struct
{
  uint8_t figure_name[3];     // 图形名
  uint32_t operate_type : 3;  // 图形操作
  uint32_t figure_type : 3;   // 图形类型
  uint32_t layer : 4;         // 图层数
  uint32_t color : 4;         // 颜色
  uint32_t details_a : 9;     // 细节参数 a
  uint32_t details_b : 9;     // 细节参数 b
  uint32_t width : 10;        // 线宽
  uint32_t start_x : 11;      // 起点/圆心 x 坐标
  uint32_t start_y : 11;      // 起点/圆心 y 坐标
  uint32_t details_c : 10;    // 细节参数 c
  uint32_t details_d : 11;    // 细节参数 d
  uint32_t details_e : 11;    // 细节参数 e
} RefereeGraphicDataStruct;

// 0x0301 子命令 0x0102 (30B)
typedef struct
{
  RefereeGraphicDataStruct figures[2];
} RefereeInteractionFigure2;

// 0x0301 子命令 0x0103 (75B)
typedef struct
{
  RefereeGraphicDataStruct figures[5];
} RefereeInteractionFigure5;

// 0x0301 子命令 0x0104 (105B)
typedef struct
{
  RefereeGraphicDataStruct figures[7];
} RefereeInteractionFigure7;

// 0x0301 子命令 0x0110 (45B)
typedef struct
{
  RefereeGraphicDataStruct graphic;
  uint8_t data[30];
} RefereeInteractionCharacter;

// 0x0301 子命令 0x0120 (4B)
typedef struct
{
  uint32_t sentry_cmd; // 哨兵自主决策指令
} RefereeSentryCmd;

// 0x0301 子命令 0x0121 (13B)
typedef struct
{
  uint8_t radar_cmd;         // 雷达自主决策指令
  uint8_t password_cmd;      // 密钥更新指令
  uint8_t password_1;        // 密钥 1
  uint8_t password_2;        // 密钥 2
  uint8_t password_3;        // 密钥 3
  uint8_t password_4;        // 密钥 4
  uint8_t password_5;        // 密钥 5
  uint8_t password_6;        // 密钥 6
  uint16_t fortify_center_x; // 设防区中心 x 坐标
  uint16_t fortify_center_y; // 设防区中心 y 坐标
  uint8_t fortify_flag;      // 设防标识
} RefereeRadarCmd;

// 0x0303 (12B)
typedef struct
{
  float target_position_x; // 目标位置 x 坐标 (m)
  float target_position_y; // 目标位置 y 坐标 (m)
  uint8_t cmd_keyboard;    // 云台手按下的键盘按键通用键值
  uint8_t target_robot_id; // 对方机器人 ID
  uint16_t cmd_source;     // 信息来源 ID
} RefereeMapCommand;

// 0x0305 (48B)
typedef struct
{
  uint16_t coordinates[24]; // 12 组 x/y/预留坐标 (cm)
} RefereeMapRobotData;

// 0x0307 (105B)
typedef struct
{
  uint8_t intention;         // 1:攻击, 2:防守, 3:移动
  uint16_t start_position_x; // 路径起点 x 坐标 (dm)
  uint16_t start_position_y; // 路径起点 y 坐标 (dm)
  int8_t delta_x[49];        // 路径点 x 轴增量数组 (dm)
  int8_t delta_y[49];        // 路径点 y 轴增量数组 (dm)
  uint16_t sender_id;        // 发送者 ID
} RefereeMapData;

// 0x0308 (34B)
typedef struct
{
  uint16_t sender_id;   // 发送者 ID
  uint16_t receiver_id; // 接收者 ID
  uint8_t user_data[30];// 字符消息
} RefereeCustomInfo;

#pragma pack(pop)

// 0x0301 内容缓冲 (<= 112B)
typedef struct
{
  uint8_t data[kRefereeMaxInteractiveDataLength];
  uint16_t length;
} RefereeInteractiveData;

// 0x0310 (<= 300B)
typedef struct
{
  uint8_t data[kRefereeCustomDataMaxLength];
  uint16_t length;
} RefereeCustomRobotData;

// 0x0311 (<= 60B)
typedef struct
{
  uint8_t data[kRefereeCustomCmdMaxLength];
  uint16_t length;
} RefereeCustomRobotCmd;

typedef struct
{
  uint8_t buffer[kUartBufferSize]; // 完整裁判系统帧
  uint16_t length;                 // 帧长度
} RefereePacket;

typedef struct
{
  RefereePacket packets[kRefereeQueueSize]; // 帧队列缓存
  volatile uint16_t head;                   // 读指针
  volatile uint16_t tail;                   // 写指针
} RefereeQueue;

typedef struct
{
  UART_HandleTypeDef *rx_huart;            // 接收串口
  UART_HandleTypeDef *tx_huart;            // 发送串口
  RefereeQueue queue;                      // 完整帧队列
  uint8_t rx_stream[kRefereeRxStreamSize]; // 流式接收缓存
  StreamBuffer rx_stream_buffer;           // 流式接收缓存状态
  uint8_t tx_sequence;                     // 发送包序号
  uint8_t tx_buffer[kUartBufferSize];      // 发送帧组装缓存
  uint32_t received_frame_count;           // 已接收有效帧计数
  uint32_t dropped_frame_count;            // 丢弃帧计数
  PeripheralGuard guard;                   // 统一外设生命周期守护对象

  // 最近一次解析出的各类裁判系统数据
  RefereeGameState game_state;
  RefereeGameResult game_result;
  RefereeGameRobotHp game_robot_hp;
  RefereeEventData event_data;
  RefereeWarning referee_warning;
  RefereeGameRobotState game_robot_state;
  RefereePowerHeatData power_heat_data;
  RefereeGameRobotPosition game_robot_position;
  RefereeBuff buff;
  RefereeRobotHurt robot_hurt;
  RefereeShootData shoot_data;
  RefereeProjectileAllowance projectile_allowance;
  RefereeRfidStatus rfid_status;
  RefereeGroundRobotPosition ground_robot_position;
  RefereeSentryInfo sentry_info;
#if defined(REFEREE_MODE_RMUC) && (REFEREE_MODE_RMUC != 0)
  RefereeDartInfo dart_info;
  RefereeDartClientCmd dart_client_cmd;
  RefereeRadarMarkData radar_mark_data;
  RefereeRadarInfo radar_info;
#endif
  RefereeInteractiveHeader interactive_header;
  RefereeInteractiveData interactive_data;
  RefereeMapCommand map_command;
  RefereeMapRobotData map_robot_data;
  RefereeMapData map_data;
  RefereeCustomInfo custom_info;
  RefereeCustomRobotData custom_robot_data;
  RefereeCustomRobotCmd custom_robot_cmd;
} RefereeObject;

extern RefereeObject g_referee;

enum
{
  kRefereeBaudrate = 115200U, // 裁判系统标准串口波特率（bps）
};

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

#endif // APP_REFEREE_H
