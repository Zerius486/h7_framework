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
| `identification` | `kSysidExponential` 为原指数遗忘 RLS；`kSysidSubspace` 为 SIFt-RLS |
| `optimize` | 模型整定后，是否增加速度 PI 性能优化与最终验证 |

两个新增字段均可省略：零初始化时保持原辨识方法，关闭性能优化。这些扩展属于 2.0 实验功能。

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

### SIFt-RLS 辨识

设置 `identification = kSysidSubspace` 即可接入；独立辨识时设置 `SysidConfig.method`。机械模型、积分窗口与质量检查保持一致。

每个积分窗口是一条标量观测，使用归一化回归向量 `x`、信息矩阵 `R`、协方差 `P`：

```text
R_next = R - (1 - λ) * (R*x)*(R*x)' / (x'*R*x) + x*x'
P_next = inverse(R_next)
θ_next = θ + P_next*x*(y - x'*θ)
```

只沿当前信息方向遗忘，弱激励时保留其他方向的信息；未激励的参数仍不能被辨识。四维信息矩阵采用 double 和固定尺寸 Cholesky 求逆，避免单精度下共线方向的信息丢失，不使用堆分配、通用矩阵库或 SVD。

`memory_time` 控制遗忘系数，不等于参数收敛时间；系数仍有原实现的 0.98 下限。短积分窗口下惯性对应的回归分量可能很小，惯量变化的跟踪会明显慢于指数遗忘。它是可选对照方案，并非所有工况下更快。

### 速度 PI 性能优化

设置 `optimize = true`，调用顺序与原整定接口相同。流程为：辨识 → PI 初值 → 重复速度测试与参数优化 → 最优参数的独立速度验证 → 可选位置/DOB 验证。

实现位于 `alg_pid_tune.h/.c`，借鉴 DiffTune+ 的敏感度传播与沿梯度预测步长，仅优化 `log(Kp)`、`log(Ki)`。这是本工程的低维实现，不是完整论文工具链的移植。名义机械响应按一阶模型精确离散，积分、误差过零衰减、条件积分及内部分段限幅均参与敏感度计算。

每轮使用相同的参考频率、幅度和测试时长，前四分之一作为过渡段，不计入损失。归一化损失为：

```text
loss = mean((error / feedback_scale)^2
          + output_weight * (output / output_max)^2
          + change_weight * (delta_output / output_max)^2)
```

内置整定的两个输出代价权重均为 0.01。根据累计残差梯度与二次近似曲率选择步长；每次增益变化限制在 0.8～1.25 倍，总范围为初值的 0.25～4 倍，最多评价 6 轮。实际损失变差时恢复已测得的最佳参数，结束优化后再验证。`optimizer.result` 提供初始损失、最佳损失、最佳增益和已评价轮数。

速度优化与最终速度验证期间不叠加加速度前馈或 DOB；位置与 DOB 在后续阶段单独验证。当前优化的是速度内环，位置环仍使用原整定方法，没有对位置、DOB 或滤波器传播完整闭环梯度。

外部功率分配或限幅的导数未知，必须调用 `AutotuneTrackOutput()`。本轮出现执行指令与请求不一致时，优化器标记 `kPidTuneLimited`，恢复已验证参数，再进行普通速度验证；整定本身仍可继续。`kPidTuneInvalidSample` 同样结束参数优化并恢复已验证增益。是否整定成功以 `AutotuneObject.state` 为准，不能只看优化器状态。

### 独立使用 PI 优化器

1. 用 `PidTuneInit()` 配置名义模型、周期、过渡时间、误差尺度和两个代价权重。
2. 用 `PidTuneStart()` 接入已初始化的正增益 PI。要求有限输出/积分限幅，启用抗饱和，关闭微分、前馈、死区和积分分离。
3. 每周期用 `PidTuneCalculate()` 代替本轮的 `PidCalculate()`；发送最终输出后调用 `PidTuneTrackOutput()`。
4. 一个固定测试窗口结束后调用 `PidTuneFinish()`；返回 true 时以新增益重复同一测试，false 时查看 `state` 与 `result`。

独立使用时，调用方负责重复参考轨迹、保持可比工况，并在每轮前安排一致的控制器/机构初始状态或足够的过渡段。测试期间不修改 PI 增益、模式或内部状态。该接口使用输入到反馈的一阶名义模型；若接入其他外环，必须提供该环坐标下的有效模型，不能直接套电机模型。

## 单电机性能约束与帕累托整定

`alg_motor_tune.h/.c` 提供 2.0 实验接口，面向 M2006、M3508 和 GM6020 的电流指令机械控制。内部复用 `alg_autotune` 建立模型及基准控制器，再完成机构特征测量、有限网格搜索和独立轨迹验证。所有计算与测试状态均在板端，无需上位机搜索器。使用静态 `MotorTuneObject`，不分配堆内存。

流程为：

```text
模型辨识及基准验证
  → 回到初始位置
  → 基准控制器的方向/低速/换向/工作速度测试
  → 可选三个角度的保持电流测量
  → 候选训练 → 满足约束后独立验证 → 下一候选
  → 从所有通过验证的候选中筛选帕累托集合
```

每轮测试前使用基准控制器回到同一初始位置并静止，避免将不同起始状态当成参数差别。每条轨迹有 16 段，每段前半程用三次平滑插值改变目标，后半程保持。速度训练包含双向 12% 低速、双向工作速度和停转；位置训练包含小幅往返、全幅往返和保持。验证改用不同的顺序及 25%、40%、85% 幅度。每个可行候选都独立验证，最终才比较支配关系。

### 物理参数与性能范围

`MotorTuneConfig` 中 `plant` 复用采样、电流、速度、连续角度限位和辨识方法。此流程要求 `speed_offset = 0`，内部关闭原 `adaptive`、`optimize`，也不叠加 DOB；位置模式复用基准位置验证，候选则重新评价整条串级控制链。

| 配置 | 含义 |
| --- | --- |
| `bandwidth` | 自然频率 `ωn` 的上下限、点数，rad/s；上限为 `0.08 / sampling_time` |
| `damping` | 阻尼比 `ζ` 的上下限、点数，允许 0.2～3 |
| `feedforward` | 加速度前馈比例 `kff` 的上下限、点数，允许 0～1.5 |
| `speed_amplitude` | 希望测试的最大速度，不超过 `0.8 * speed_limit` |
| `position_amplitude` | 位置测试幅度；启动位置至较近边界的距离至少是其 `1/0.3` 倍 |
| `segment_time` | 单段时长，0.1～10 s，且至少 20 个采样周期 |
| `settling_band` | 相对测试幅度的稳定带，0～0.5 之间 |
| `limits` | 跟踪 RMS、超调、调节时间、保持波动、电流变化率、饱和比例的上限；单项 0 表示不约束 |

每个搜索维度取 1～3 个等间隔点，共最多 27 个候选。单点范围的上下限必须相等。候选使用以下可解释的参数关系：

```text
Kp = 2 * ζ * ωn * M - B
Ki = M * ωn²
速度 PID 的导数前馈系数 = kff * M
位置外环 Kp = 0.15 * ωn
```

非正或无效的 PI 增益标记为 `kMotorTuneInvalidDesign`，不会偷偷修改成另一组物理参数。速度 PID 使用现有的目标差分前馈、积分限幅和抗饱和；导出的 `PidConfig` 与测试配置一致。位置模式评价的是位置误差，速度模式评价的是速度误差，位置外环比例随速度带宽变化，本版不再单独搜索外环比例。

速度测试幅度会按初始行程缩小，实际采用值在 `result.profile.tested_speed`。因此狭小位置机构的速度特征不代表电机全速特征。`segment_time` 应足够容纳目标响应：若所有候选都来不及稳定，结果会明确失败，不放宽用户给定的指标。

### 指标和特征含义

候选的 `training`、`validation` 分别保留两条轨迹的性能：

- `rms_error`：全轨迹误差除以测试幅度后的均方根。
- `overshoot`：保持阶段相对该次目标变化量的最大超调比例。
- `settling_time`：从进入保持阶段起，到最后一次离开稳定带后的最坏时间；不含前半程目标插值时间。
- `idle_ripple`：各保持段最后四分之一窗口的误差标准差最大值，单位为 rad/s 或 rad，不代替静差指标。
- `rms_slew`：最终发送电流指令的变化率均方根，A/s。
- `saturation`：内部电流饱和或外部分配改变指令的采样比例。

只有所有保持段的最后 10% 保持在稳定带内，才将 `settled` 置为 true。约束在训练、验证中分别检查。最后在通过两次检查的候选中，同时最小化上述六项验证指标；任一项更好且其余均不差才构成支配。帕累托下标按枚举顺序保存，不暗含推荐排名。这是有限实测集合的非支配解，不是连续参数空间的全局最优证明。

`result.profile` 保存基准闭环下的双向误差、低速误差、工作速度响应比例、启动响应时间及电流、换向响应时间及位移；位置模式还保存负位置、中心、正位置的平均反馈电流与平均角度。启动阈值为实际速度幅度的 2%，延迟包含测试参考的插值过程。`startup_observed`、`reversal_observed` 为 false 时，对应时间只是观测窗口下界；计数为 0 时表示没有有效观测。保持电流仅在位置与速度均满足条件时采集。

这些特征用于定位单一惯性/阻尼/摩擦模型未解释的现象，保留给应用选择工况与后续模型扩展；本版不根据某次不对称观测自动改模型或判定故障。仅用转子反馈无法分离齿隙与闭环换向动态，`reversal_travel` 不能当成减速箱回程间隙。不同姿态保持电流是样本，不直接拟合重力模型。

### 接入示例

下面的数值只演示配置结构，应按机构允许的运动范围、电流和期望响应填写：

```c
#include "alg_motor_tune.h"

static MotorTuneObject motor_tune;

MotorTuneConfig config = {
    .plant = {.sampling_time = 0.001f,
              .current_limit = 2.0f,
              .speed_limit = 20.0f,
              .position_min = -50.0f,
              .position_max = 50.0f,
              .response_time = 0.2f},
    .bandwidth = {.lower = 12.0f, .upper = 24.0f, .count = 3},
    .damping = {.lower = 0.8f, .upper = 1.2f, .count = 3},
    .feedforward = {.lower = 0.0f, .upper = 1.0f, .count = 3},
    .limits = {.rms_error = 0.15f,
               .overshoot = 0.1f,
               .settling_time = 0.3f,
               .saturation = 0.1f},
    .speed_amplitude = 5.0f,
    .segment_time = 1.0f,
    .settling_band = 0.05f};

if (MotorTuneInit(&motor_tune, &config))
{
  (void)MotorTuneStart(&motor_tune, position, omega);
}
```

周期任务的顺序：

```c
if (MotorTuneIsActive(&motor_tune))
{
  float request = MotorTuneUpdate(&motor_tune, position, omega, actual_current);
  float executed = ApplyCurrentLimit(request); // 应用自身的限幅/功率分配
  SendMotorCurrent(executed);                  // 应用自身的发送接口
  if (MotorTuneIsActive(&motor_tune))
  {
    (void)MotorTuneTrackOutput(&motor_tune, executed);
  }
}
```

`ApplyCurrentLimit` 和 `SendMotorCurrent` 是应用示意接口，不是框架新增函数。每次活跃的 `Update` 后必须反馈一次最终执行量；漏掉反馈会返回 `kMotorTuneOutputMissing`，重复反馈不重复计数。下一次 `Update` 用新反馈评价上一周期参考，完成窗口时也会等待最后一个执行量对应的反馈。

`actual_current` 必须是对应采样区间的电机电流反馈，不能用请求电流代替。应用现有滤波可继续放在输入侧，但搜索与正式运行必须保持相同反馈处理、采样周期、坐标和输出链。这里不自动配置 CAN 或接收回调；GM6020 需要启用电流控制模式。

完成后读取 `result.pareto_count`，通过 `result.pareto[i]` 查看具体候选。选择候选后调用 `MotorTuneExport(&motor_tune, i, &speed_config, &position_config)` 导出配置；速度模式可省略位置输出。应用负责保存、初始化 PID 和接管时的控制状态，默认任务不会自动启动整定。正式控制存在外部限幅时，也应调用 `PidTrackOutput(&speed_pid, executed - request, candidate.bandwidth)`，保持与候选测试相同的抗饱和链。`MotorTuneCancel()` 可结束测试；失败原因在 `error`，原始辨识失败细节在 `baseline.error`。训练失败、验证失败和无效设计均保留在候选状态中，不会作为可用参数导出。

一次训练或验证为 `16 * segment_time` 秒，特征测量另需一轮；27 组候选均通过时最多再执行 54 轮，另加辨识与各轮回中时间。可先减少网格点数再缩小范围细化。此版本输出实测参数集合与模型设计关系，尚不拟合随负载、速度、姿态变化的调度函数，也不把单电机结果直接用于整车外环。

## 正常运行中的自适应

成功 `AutotuneApply()` 后，配置启用 `adaptive` 时，每周期调用 `AutotuneAdapt()`，传入运行速度 PID、反馈角速度和实际反馈电流。其返回值为增量加速度前馈电流，由应用纳入总电流请求。

更新依赖有效动态激励、模型质量与惯量范围，增益按较慢节奏受限变化。DOB/积分仍处理摩擦和剩余负载；不能再把辨识模型中的全部负载重复叠加为补偿。

启用 `optimize` 时，应用或恢复参数会记录已验证增益相对模型设计值的比例；正常自适应保留该比例，避免把性能优化结果逐渐覆盖回原设计值。运行自适应仍只做慢速模型更新，不重复主动测试轨迹。

保存模型、设计自然频率和控制参数后，可用 `AutotuneRestore()` 接入已经验证的模型及运行对象。该函数不读写 Flash，持久化由应用实现。

## 底盘整体落地整定

接口位于 `alg_chassis_tune.h`。支持全向、麦轮和舵轮，要求已有可用电机闭环、运动学、轮速反馈和 IMU 角速度。它测试车体响应并整定角速度外环，不替代单电机整定。

`ChassisTuneConfig` 配置采样时间、平移速度、旋转速度、距离范围、角度范围和响应时间。状态由静态 `ChassisTuneObject` 保存。

同样可设置 `identification = kSysidSubspace` 与 `optimize = true`，作用于车体角速度外环；三种底盘共享此流程。上一周期 `executed_omega` 同时用于辨识和执行量跟踪，内部在开始下一周期前结束待处理的优化窗口。

1. `ChassisTuneInit()` 初始化，`ChassisTuneStart()` 开始测试。
2. 活跃期间周期调用 `ChassisTuneUpdate()`，输入轮速估计的车体速度、IMU 角速度及上一周期最终下发的运动学角速度。
3. 将返回的 `ChassisVelocity` 交给已有底盘运动学与电机闭环；所有限幅后实际采用的角速度作为下一周期 `executed_omega`。
4. 使用 `ChassisTuneIsActive()`、`state`、`error` 判断运行结果；`ChassisTuneCancel()` 取消。
5. 完成后从 `yaw.result` 读取角速度外环参数，从 `forward`、`lateral` 读取平移响应统计。正式控制接入与参数保存由应用实现。

整车旋转模型的输入是运动学角速度指令；其 PI 输出同样是角速度指令，不能当成电机电流环增益。平移距离与响应来自轮速里程计，共同滑移无法由轮速独立识别，不能将其解释为真实地面位移标定。

算法实现可独立编译和仿真；最终增益、内环近似、负载变化及结构共振仍需用对应机构的数据验证。

## 方法来源

- [SIFt-RLS: Subspace of Information Forgetting Recursive Least Squares](https://arxiv.org/html/2404.10844v1)，2024：当前按单条标量观测实现四维信息子空间遗忘。
- [DiffTune+](https://proceedings.mlr.press/v211/cheng23b.html)，L4DC 2023：当前借鉴低维敏感度传播及沿梯度二次预测步长，使用本工程的 PI、损失函数和增益边界。
