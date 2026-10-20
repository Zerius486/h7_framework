# 自动整定与辨识

整定、辨识和自适应均在 H7 上运行，通过普通 C 接口使用。算法负责模型、激励与验证；应用负责采样、控制权切换、执行指令、开始/取消入口和结果保存。默认应用尚未接入这些流程。

## 单电机整定

接口位于 `alg_autotune.h`，辨识基础位于 `alg_sysid.h`。使用静态 `AutotuneObject` 保存状态，通过 `AutotuneConfig` 配置：

| 字段 | 含义 |
| --- | --- |
| `sampling_time` | 调用周期，s |
| `current_limit` | 测试电流上限，A |
| `speed_limit` | 测试角速度上限，rad/s |
| `position_min/max` | 同一连续角度坐标中的运动范围，rad |
| `speed_offset` | 速度工作点，rad/s |
| `response_time` | 期望响应时间，s；0 为自动选择 |
| `tune_position` | 是否整定位置外环 |
| `adaptive` | 应用参数后是否允许慢速自适应 |

### 调用顺序

1. `AutotuneInit()` 初始化对象并检查返回值。
2. `AutotuneStart()` 传入当前角度、角速度和可选 DOB；测试接管该电机的电流指令。
3. 活跃期间，每周期调用 `AutotuneUpdate(tune, position, omega, actual_current, rotor_omega)`，返回测试电流。
4. 应用限幅或功率分配后发送电流，并用 `AutotuneTrackOutput()` 反馈最终执行电流指令。
5. 通过 `state`、`error` 和 `result` 获取进度、原因与结果。仅 `kAutotuneComplete` 表示完成验证。
6. 调用 `AutotuneApply()` 将完成验证的参数及控制状态写入运行中的速度 PID、可选位置 PID 和 DOB。结果也可另行保存后用于正式应用。

`AutotuneIsActive()` 用于判断是否继续接管；`AutotuneCancel()` 结束测试。退出测试后的输出与业务切换由应用处理。

`position` 使用连续角度，不能把跨越 ±π 的原始编码器值直接作为有限行程。`omega` 与控制器使用相同的反馈坐标；`rotor_omega` 为转子侧角速度，供 DOB 使用。电流均为 A，实际反馈电流与最终执行电流指令是两个不同输入。

### 模型与增益

内部电流环相对机械运动足够快时，采用电流坐标机械模型：

```text
i = M * dω/dt + B * ω + F * sign(ω) + L
M = J/Kt, B = b/Kt
```

对一个采样窗口积分：

```text
∫i dt = M * Δω + B * ∫ω dt + F * ∫sign(ω) dt + L * Δt
```

固定四维 RLS 辨识惯性、黏性、库仑摩擦和恒定负载，避免直接对速度微分。辨识使用归一化和模型质量条件。独立使用时，按顺序调用 `SysidInit()`、`SysidUpdate()`、`SysidGetModel()`。

速度 PI 以设计自然频率 `ωn` 为基础：

```text
Kp = 2 * ωn * M - B
Ki = M * ωn²
```

实现还会检查可用增益、降低验证未通过的带宽，并按配置继续验证位置外环及 DOB。`result.bandwidth` 单位为 rad/s，`dob_cutoff_hz` 单位为 Hz。`PidObject.integral_max` 表示误差积分限幅，若积分电流预算为 `I_max`，应按 `I_max / Ki` 换算。

该模型假设实际电流能代表转矩输入。若目标机械带宽接近内电流环动态，需要再评价内环滞后；初始辨识不要求加入完整电气状态。

## 正常运行中的自适应

成功 `AutotuneApply()` 后，配置启用 `adaptive` 时，每周期调用 `AutotuneAdapt()`，传入运行速度 PID、反馈角速度和实际反馈电流。其返回值为增量加速度前馈电流，由应用纳入总电流请求。

更新依赖有效动态激励、模型质量与惯量范围，增益按较慢节奏受限变化。DOB/积分仍处理摩擦和剩余负载；不能再把辨识模型中的全部负载重复叠加为补偿。

保存模型、设计自然频率和控制参数后，可用 `AutotuneRestore()` 接入已经验证的模型及运行对象。该函数不读写 Flash，持久化由应用实现。

## 底盘整体落地整定

接口位于 `alg_chassis_tune.h`。支持全向、麦轮和舵轮，要求已有可用电机闭环、运动学、轮速反馈和 IMU 角速度。它测试车体响应并整定角速度外环，不替代单电机整定。

`ChassisTuneConfig` 配置采样时间、平移速度、旋转速度、距离范围、角度范围和响应时间。状态由静态 `ChassisTuneObject` 保存。

1. `ChassisTuneInit()` 初始化，`ChassisTuneStart()` 开始测试。
2. 活跃期间周期调用 `ChassisTuneUpdate()`，输入轮速估计的车体速度、IMU 角速度及上一周期最终下发的运动学角速度。
3. 将返回的 `ChassisVelocity` 交给已有底盘运动学与电机闭环；所有限幅后实际采用的角速度作为下一周期 `executed_omega`。
4. 使用 `ChassisTuneIsActive()`、`state`、`error` 判断运行结果；`ChassisTuneCancel()` 取消。
5. 完成后从 `yaw.result` 读取角速度外环参数，从 `forward`、`lateral` 读取平移响应统计。正式控制接入与参数保存由应用实现。

整车旋转模型的输入是运动学角速度指令；其 PI 输出同样是角速度指令，不能当成电机电流环增益。平移距离与响应来自轮速里程计，共同滑移无法由轮速独立识别，不能将其解释为真实地面位移标定。

算法实现可独立编译和仿真；最终增益、内环近似、负载变化及结构共振仍需用对应机构的数据验证。
