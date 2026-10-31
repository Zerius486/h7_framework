# 应用开发

## 工程结构

`Core/` 保存 CubeMX 生成的启动与外设代码，`Drivers/`、`Middlewares/` 提供 HAL、CMSIS-DSP 和 FreeRTOS。Skywalker 分为四层：

| 层 | 职责 |
| --- | --- |
| BSP | UART、FDCAN、SPI、流缓存、DMA 与计时封装 |
| Device | 电机、遥控器和 IMU 的反馈解析、状态与指令 |
| Algorithm | 通用控制、估计、滤波、运动学及辨识算法 |
| Application | 设备配置、通信协议、任务与机器人控制逻辑 |

Device 使用 BSP 通信，也可组合算法；Application 组织设备和控制流程。通用算法不依赖设备型号或应用协议。

启动顺序为外设初始化、内核初始化、`AppConfigInit()`、创建任务、启动调度器，随后执行各任务的 `TaskInit()` 和 `TaskLoop()`。`app_task.c` 用 `osDelayUntil()` 周期调度，当前 RTOS tick 为 1 ms。

常规开发修改 `app_config.h/.c`、`app_robot.c`、`app_protocol.h/.c`，机构复杂时可拆分应用文件。裁判系统解析位于 `app_referee.h/.c`。当前 `AppConfigInit()` 初始化默认关闭的调试记录器；四个任务循环为空，实际设备、控制逻辑和通信接收回调仍需由应用接入。

## 1. 配置外设与机构

通过 `h7_framework.ioc` 配置引脚、时钟、CAN、UART 和 SPI。在 `app_config.h/.c` 中定义板卡角色、控制来源、任务周期和机构参数，并声明需要共享的设备对象。

普通自定义串口优先使用 UART1、UART7、UART10；UART2/3 用于 RS485，UART5 用于 DT7，按实际硬件用途分配。仅填写串口句柄不会自动注册接收回调。

在 `AppConfigInit()` 中直接调用 BSP、设备和算法初始化接口，注册 CAN/UART 接收回调。此函数在任务创建前执行；需要任务上下文的初始化放入相应 `TaskInit()`。

方向、减速比和零位可以使用 `AppMotorConfig` 表达。应用按具体驱动反馈的轴侧定义，显式完成机构坐标转换；不要对驱动已经换算过的量重复应用减速比。

### DM8009P 电机

`dev_dm_motor.h` 支持 J4310 和 J8009P，复用 MIT、位置速度及速度模式的控制帧与反馈解析：

```c
DmMotor motor;
DmMotorInit(&motor, kDmMotorTypeJ8009P, kDmMotorModeMit, 1U, 0x11U);
motor.state.given_torque = 2.0f;
motor.state.kp = 0.0f;
motor.state.kd = 0.0f;

uint8_t tx_data[8];
DmMotorPackMit(&motor, tx_data);
// 使用DmMotorTxId(&motor)发送；使能命令由应用按启动流程发送。
```

J8009P 的协议量程预设为位置 ±12.5 rad、速度 ±45 rad/s、力矩 ±54 Nm。接入前用达妙调试助手读取电机的 PMAX、VMAX、TMAX；若不同，在初始化后修改 `motor.limits.position_max`、`velocity_max`、`torque_max`，三者必须为正。命令编码与所有模式的反馈解码共用这组量程。协议量程不等于额定或峰值力矩，实际输出限幅由应用按电机规格与机构要求设置。

反馈和目标使用输出轴侧的 rad、rad/s、Nm，不再重复换算电机内部减速比。原有无型号参数的 `DmMotorTorqueToCurrentUnits()`、`DmMotorCurrentUnitsToTorque()` 保留 J4310 的换算含义；J8009P 的 MIT 控制直接填写 `given_torque`。

帧格式依据 [DM-J8009P-2EC V1.0 手册](https://github.com/enactic/damiao/blob/main/website/i18n/en/docusaurus-plugin-content-docs/current/products/hardware/dm-j8009p-2ec-v1.0.mdx)，量程可由电机端配置，需与实机一致。力位混控模式是否可用取决于电机固件版本。

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

控制器与滤波器通过 C 对象和函数自由组合。目标、反馈和限幅需对应同一坐标与单位。PID 的 `output_max` 必须为正；`integral_max` 限制误差积分，积分输出预算为 $K_i\cdot\text{integral\_max}$。

### 舵轮驱动速度修正

`alg_motion.h` 提供 `SteerSpeedCorrect()`，按实际舵角误差进行三次余弦修正。先生成舵角和轮速目标，再修正驱动速度：

```c
// chassis与setpoint已初始化，angle为同一坐标中的四轮实际舵角。
float drive_speed[4];
ChassisSetWheelAngleFeedback(&chassis, angle);
ChassisSetVelocity(&chassis, vx, vy, omega);
SteerSetpointUpdate(&setpoint, &chassis, wheel_acceleration, steer_rate,
                    switch_hysteresis, sampling_time);
SteerSpeedCorrect(&chassis.state, angle, angle_scale, drive_speed);
```

将 `drive_speed[i]` 换算为对应电机的目标角速度，舵向位置环使用 `chassis.state.wheel_angle[i]`。不使用协调目标生成时，用 `ChassisUpdateState()` 替换 `SteerSetpointUpdate()`。

`angle_scale` 即可调参数 a，通常从 1 开始，0 表示不衰减。修正后轮速不写回协调规划状态；反馈舵角应每周期更新，且不能用目标舵角代替实际值。

### 任务分工

单板可在四个任务中实现各机构；双板分别配置云台板和底盘板，使用协议传递所需目标与状态。`AppBoardRole` 和 `AppControlSource` 不包含自动切换逻辑，切换规则在任务业务中实现。

状态枚举、转移条件和进入/退出动作按机器人在应用层定义。底盘、云台和发射机构可以分别持有状态，通过目标和反馈协调；框架不要求统一状态树或事件队列。模式切换时的参考重建及控制器交接由应用负责。

## 3. 定义通信协议

在 `app_protocol.h` 定义消息结构和公共接口，在 `app_protocol.c` 实现帧同步、校验、字段解析及发送。

现有视觉协议使用 `AB` 帧头、CRC16 和流缓存；`VisionProtocolRxCallback()` 需要绑定到视觉 UART 接收入口。发送通过 `VisionProtocolTransmitGimbalState()`，串口取自 `g_app_communication_config.vision_huart`。默认业务尚未执行这些绑定或周期发送。

可使用 `bsp_stream.h` 处理分包/粘包，使用 `alg_crc.h` 校验。板间消息、调试观测和整定命令均由应用按需求定义。

裁判系统通过 `RefereePackFrame()` 打包，或通过 `RefereeSend()` 打包并提交 UART DMA 发送。`RefereeSend()` 返回 `true` 表示已提交，不表示传输已完成；串口忙或提交失败时返回 `false`，后续发送时机由应用安排。

## 4. 编译与测量

根目录 `CMakeLists.txt` 递归收集 Skywalker 各层 `Src/` 下的 C 文件，包含各层 `Inc/`。新增或删除源文件后，下次构建会自动重新配置 CMake；替换应用时确保每组任务只有一份强定义。

编译命令及工具链见[README](../README.md#编译)。Debug 和 Release 输出分别位于 `build/Debug/`、`build/Release/`。

测量用时使用 `bsp_profile.h`：先调用 `ProfileInit()`、`ProfileReset()`，再用 `ProfileBegin()`/`ProfileEnd()` 包围待测代码。结果为 CPU 周期，包含期间的中断和任务抢占；单次测量应短于 32 位计数器回绕周期。

控制算法见[算法说明](algorithms.md)，整定调用见[自动整定与辨识](autotune.md)。
