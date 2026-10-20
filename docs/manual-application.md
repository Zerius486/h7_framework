# 应用开发

## 1. 配置外设与机构

通过 `h7_framework.ioc` 配置引脚、时钟、CAN、UART、SPI 和任务资源。在 `app_config.h/.c` 中定义板卡角色、控制来源、任务周期和机构参数，并声明需要共享的设备对象。

普通自定义串口优先使用 UART1、UART7、UART10；UART2/3 用于 RS485，UART5 用于 DT7，按实际硬件用途分配。仅填写串口句柄不会自动注册接收回调。

在 `AppConfigInit()` 中直接调用 BSP、设备和算法初始化接口，注册 CAN/UART 接收回调。此函数在任务创建前执行；需要任务上下文的初始化放入相应 `TaskInit()`。

方向、减速比和零位可以使用 `AppMotorConfig` 表达。应用按具体驱动反馈的轴侧定义，显式完成机构坐标转换；不要对驱动已经换算过的量重复应用减速比。

## 2. 实现周期控制

在 `app_robot.c` 中实现以下四组接口：

```c
uint32_t ManagerTaskInit(void);
void ManagerTaskLoop(void);
uint32_t GimbalTaskInit(void);
void GimbalTaskLoop(void);
uint32_t ChassisTaskInit(void);
void ChassisTaskLoop(void);
uint32_t ShootTaskInit(void);
void ShootTaskLoop(void);
```

初始化返回任务周期（ms）。默认从 `g_app_task_config` 读取，均为 1 ms。算法 `sampling_time` 使用秒，与实际调用周期一致。

典型控制顺序：

```text
读取设备反馈 → 机构坐标转换 → 滤波/估计
→ 读取目标与状态 → 轨迹/运动学 → PID/前馈/DOB
→ 电流限幅/功率分配 → 发送 → 执行量跟踪
```

控制器与滤波器通过 C 对象和函数自由组合。目标、反馈和限幅需对应同一坐标与单位。PID 的 `output_max` 必须为正；`integral_max` 限制误差积分，积分输出预算为 `Ki * integral_max`。

单板可在四个任务中实现各机构；双板分别配置云台板和底盘板，使用协议传递所需目标与状态。`AppBoardRole` 和 `AppControlSource` 不包含自动切换逻辑，切换规则在任务业务中实现。

## 3. 定义通信协议

在 `app_protocol.h` 定义消息结构和公共接口，在 `app_protocol.c` 实现帧同步、校验、字段解析及发送。

现有视觉协议使用 `AB` 帧头、CRC16 和流缓存；`VisionProtocolRxCallback()` 需要绑定到视觉 UART 接收入口。发送通过 `VisionProtocolTransmitGimbalState()`，串口取自 `g_app_communication_config.vision_huart`。默认业务尚未执行这些绑定或周期发送。

可使用 `bsp_stream.h` 处理分包/粘包，使用 `alg_crc.h` 校验。板间消息、调试观测和整定命令均由应用按需求定义。

## 常用 C 接口

| 功能 | 头文件 |
| --- | --- |
| 电机 | `dev_dji_motor.h`、`dev_dm_motor.h`、`dev_lk_motor.h` |
| PID 与扰动补偿 | `alg_pid.h`、`alg_dob.h` |
| IMU 与滤波 | `dev_bmi088.h`、`alg_lpf.h`、`alg_notch.h`、`alg_imu_ekf.h` |
| 三构型运动学 | `alg_kinetics.h` |
| 轨迹、舵轮约束、轮速估计及功率模型 | `alg_motion.h` |
| 辨识与整定 | `alg_sysid.h`、`alg_autotune.h`、`alg_chassis_tune.h` |
| UART/CAN、流缓存 | `bsp_uart.h`、`bsp_fdcan.h`、`bsp_stream.h` |
| 实际用时测量 | `bsp_profile.h` |

性能测量先调用 `ProfileInit()`、`ProfileReset()`，再用 `ProfileBegin()`/`ProfileEnd()` 包围待测代码。结果为 CPU 周期，包含期间的中断和任务抢占，单次测量应短于 32 位计数器回绕周期。

## 构建与验证

```sh
cmake --preset Debug
cmake --build --preset Debug
```

默认任务循环为空，配置中的机械参数也为占位值。实现机构逻辑后，核对反馈方向、限幅、坐标和周期，再通过对应机构验证控制参数。整定的调用顺序见 [自动整定与辨识](autotune.md)。
