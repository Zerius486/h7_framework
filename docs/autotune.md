# 自动整定与辨识

整定在 H7 上执行。应用提供采样、控制权切换、电流发送、开始/取消和结果保存。

| 需求 | 接口 |
| --- | --- |
| 只辨识机械模型 | `alg_sysid.h` |
| 由模型生成一组参数，可选慢速自适应 | `alg_autotune.h` |
| 按性能约束搜索多组参数 | `alg_motor_tune.h` |
| 优化已有速度 PI | `alg_pid_tune.h` |
| 为闭环频响提供多正弦参考 | `alg_multisine.h` |
| 离线对照与候选建议 | [tools/tuning](../tools/tuning/README.md) |

当前主动整定需要有限行程边界，测试包含正反运动。SIFt-RLS、PI 优化和帕累托搜索按实验功能使用；当前未提供连续旋转、单向机构模式、整定结果的结构化导出及 NN 训练。原始串口波形和基础运行元数据可用本地[采集工具](../tools/tuning/README.md)保存。

## 保存一次整定记录

板端日志通道由 `g_app_debug_log_config.enabled` 控制，默认关闭。开启后，`AppTuneLogInit()` 在 `AppConfigInit()` 中初始化记录器。当前默认使用 UART7、921600 baud；串口波特率必须和 CubeMX 外设配置相同，并连接 USB-UART 适配器到采集电脑。

应用需在对应电机控制周期中调用整定日志适配器。`Autotune` 在每次 `AutotuneUpdate()`、最终限幅与发送量跟踪完成后调用 `AppTuneLogAutotune()`；`MotorTune` 在每次 `MotorTuneUpdate()` 和 `MotorTuneTrackOutput()` 之后调用 `AppTuneLogMotorTune()`。每个适配器只传对应的整定对象。首次检测到整定活跃时开始记录，检测到结束状态时缓存结果事件并请求停止采样。两个适配器都会处理缓存发送，结束后仍须周期调用；若不再调用适配器，则改为在同一任务周期调用 `AppTuneLogProcess()`，直到剩余事件与结束摘要发送完成。应用若不调用适配器，就不会产生整定记录。示例参数须取同一控制周期的实际值，尤其 `executed_current` 应是应用最终发送的电流指令。

采集程序和输出文件格式见[整定数据采集与离线分析](../tools/tuning/README.md)。该采集器会保存串口收到的原始字节，并统计坏 CRC 与帧序号缺口。固件端 UART 忙时会跳过日志帧；元数据中的序号缺口可用于发现丢帧，但不能恢复丢失的波形。默认关闭记录器不会增加业务串口发送。

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
| `identification_min_speed` | 滑动辨识最低速度，rad/s；0 使用速度上限的 1% |
| `identification_current_delay_samples` | 辨识时使用的电流历史采样数，0～32；0 关闭 |
| `identification_window` | 积分窗口，s；0使用20ms与4个采样周期中的较大值，显式配置范围为采样周期～0.1s |
| `identification_timeout` | 辨识总时长上限，s；0使用20s，显式配置至少12s |

`identification` 和 `optimize` 零初始化时，使用指数遗忘 RLS，关闭 PI 性能优化。

### 调用顺序

1. `AutotuneInit()` 初始化对象并检查返回值。
2. `AutotuneStart()` 传入当前角度、角速度和可选 DOB；测试接管该电机的电流指令。
3. 活跃期间，每周期调用 `AutotuneUpdate(tune, position, omega, actual_current, rotor_omega)`，返回测试电流。
4. 应用限幅或功率分配后发送电流，并用 `AutotuneTrackOutput()` 反馈最终执行电流指令。
5. 通过 `state`、`error` 和 `result` 获取进度、原因与结果。仅 `kAutotuneComplete` 表示完成验证。
6. 调用 `AutotuneApply()` 将参数及控制状态写入运行中的速度 PID、可选位置 PID 和 DOB。

`AutotuneIsActive()` 用于判断是否继续接管；`AutotuneCancel()` 结束测试。退出测试后的输出与业务切换由应用处理。

`position` 使用连续角度，不能把跨越 $\pm\pi$ 的原始编码器值直接作为有限行程。`omega` 与控制器使用相同的反馈坐标；`rotor_omega` 为转子侧角速度，供 DOB 使用。电流均为 A，实际反馈电流与最终执行电流指令是两个不同输入。

### 模型与增益

内部电流环相对机械运动足够快时，采用电流坐标机械模型：

$$
i = M\frac{d\omega}{dt} + B\omega + F\operatorname{sign}(\omega) + L
$$

$$
M = \frac{J}{K_t}, \qquad B = \frac{b}{K_t}
$$

这里的 $J$、$b$ 和 $K_t$ 必须对应所用角速度的同一轴侧。返回字段 `inertia` 是电流坐标参数 $M$，单位为 $\mathrm{A\,s^2/rad}$，不是单位为 $\mathrm{kg\,m^2}$ 的转动惯量；`viscous` 为 $B$，单位为 $\mathrm{A\,s/rad}$，`friction` 和 `bias` 的单位均为 A。DOB 配置使用转子侧物理惯量，不能直接填入 `model.inertia`。

对一个采样窗口积分：

$$
\int i\,dt = M\Delta\omega + B\int\omega\,dt + F\int\operatorname{sign}(\omega)\,dt + L\Delta t
$$

四维 RLS 用窗口积分辨识惯性、黏性、库仑摩擦和恒定负载，避免直接差分速度。独立使用时依次调用 `SysidInit()`、`SysidUpdate()`、`SysidGetModel()`，模型读取须通过质量检查。

速度 PI 以设计自然频率 $\omega_n$ 为基础：

$$
K_p = 2\zeta\omega_n M - B, \qquad K_i = M\omega_n^2
$$

该关系匹配名义闭环特征多项式，$\zeta$ 为设计阻尼比，初始取 1。`Autotune` 将 $K_p$ 下限设为 $0.1\omega_nM$，以保持正增益。

实现会在固定轨迹上比较初始自然频率的 1、0.6、1.6 倍和阻尼比 1、1.4，共六组参数；自然频率仍受采样周期限制。保存通过基准跟踪条件且误差评分最低的一组，再重复验证，并按配置继续验证位置外环及 DOB。重试不改变参考频率、幅度或时长。`result.bandwidth` 保存设计自然频率，单位为 rad/s，不是实测的闭环 −3 dB 带宽；`dob_cutoff_hz` 单位为 Hz。`PidObject.integral_max` 表示误差积分限幅，若积分电流预算为 $I_{\max}$，应按 $I_{\max}/K_i$ 换算。

目标机械带宽接近内电流环动态时，需要评价内环滞后对该模型的影响。

### 分阶段激励

主动辨识按以下顺序进行，进度在`identification_stage`中：

1. `kAutotuneProbe`：正反向各0.75s电流斜坡，记录首次达到滑动速度阈值的实际电流。
2. `kAutotuneQuasistatic`：正反向各1.5s慢斜坡，补充阻尼与摩擦数据。
3. `kAutotuneDynamic`：至少4s变长随机脉冲，补充惯性激励；幅度随实际运动调整。
4. `kAutotuneModelValidation`：冻结模型，另采2s电流和速度，做自由速度预测。

各阶段共用速度和行程限制，斜坡会叠加速度约束，不能把首次滑动电流当作严格的静摩擦测量值。`startup_current[0/1]`分别对应正/负方向，`startup_observed`说明是否观测到。阶段切换清除未完成的积分窗口，已拟合参数保留。起动、低速、滑动和动态数据按用途处理，不要求滑动模型解释静摩擦全过程。

### 数据配对与模型检查

`identification_current_delay_samples` 只用于辨识，将历史反馈电流与当前速度配对。按固定调用周期计算延迟：$T_d=N\Delta t$。例如实测需要 4 ms 配对时，1 ms 周期设置 4，2 ms 周期设置 2。启动和重新初始化辨识器时先填满历史；PID、DOB 的实时反馈不经过这条历史队列。

延迟应由同一套采样链测量，不按电机型号默认写死。RPM 与编码器速度的比较可以提供线索，但不能独自证明电流与 RPM 的确切延迟。

一个积分窗口必须全程高于最低滑动速度且没有换向，才参与机械模型辨识。最低速度应结合反馈分辨率和实际滑动状态设置；零初始化的 1% 是兼容默认值。静止、起动和过零窗口不用于这个滑动摩擦模型，`sysid.rejected_windows` 记录被排除的窗口数。原始样本仍由应用保留。

读取模型时枚举阻尼、摩擦的非负约束，并重新求解其余参数。惯量不取绝对值，也不钳成正数。`fitness` 用返回模型重新计算，不能沿用约束前的拟合度。

匀速窗口参与参数拟合；每40个具有速度变化的有效窗口形成一个不重叠的数据段，用于惯量检查。不重叠不代表噪声在统计上独立。相邻两段分别做约束拟合，检查惯量是否显著大于零、两段估计的三倍标准差区间是否相容，并互相预测电流。`validation_fitness` 是两个方向中较差的交叉拟合度，要求至少 0.75；整体模型还需通过原有激励和拟合检查。标准差来自局部线性回归残差，是辨识筛选依据，不能替代实际机构验证。`result.model.status` 区分样本不足、动态激励不足、非物理模型、数据段不一致和模型可用；`AutotuneError` 的原有编号保持不变。至少需要两段有效数据，弱激励或不一致的数据继续采集，主动辨识默认最长20s，可通过`identification_timeout`修改。

模型通过上述检查后，还要在后续数据上验证速度预测：每个连续滑动片段仅在起点注入实测速度，随后用实际电流积分模型；片段最长250ms，过零、低速后重新开始。至少累计0.5s有效预测、4个片段和足够速度变化，要求`velocity_fitness >= 0.75`。`velocity_rmse`单位为rad/s。预测失败返回新增的`kAutotunePredictionError`，不生成PI。独立使用时依次调用`SysidPredictionInit/Update/GetResult`；`GetResult`仅更新传入模型的两个速度指标，模型参数保持不变。

`kAutotuneComplete` 表示模型和基准轨迹验证完成。若需要限制超调、调节时间等性能，使用下面的 `MotorTuneConfig.limits`，并在所需工作轨迹上确认结果；当前测试参考含过渡段，不是理想阶跃。

### SIFt-RLS 辨识

设置 `identification = kSysidSubspace` 即可接入；独立辨识时设置 `SysidConfig.method`。机械模型、积分窗口与质量检查保持一致。

每个积分窗口是一条标量观测，使用归一化回归向量 `x`、信息矩阵 `R`、协方差 `P`：

$$
R_{k+1}=R_k-(1-\lambda)\frac{(R_kx)(R_kx)^\mathsf{T}}{x^\mathsf{T}R_kx}+xx^\mathsf{T}
$$

$$
P_{k+1}=R_{k+1}^{-1}, \qquad \theta_{k+1}=\theta_k+P_{k+1}x(y-x^\mathsf{T}\theta_k)
$$

SIFt 只沿当前信息方向遗忘，弱激励时保留其他方向的信息。实现使用 double 四维信息矩阵和固定尺寸 Cholesky 求逆。

`memory_time` 控制遗忘系数，下限为 0.98。短积分窗口下，惯性回归分量较小，惯量变化的跟踪可能慢于指数遗忘；选用前应以同一段数据对照。

### 速度 PI 性能优化

设置 `optimize = true`，调用顺序与原整定接口相同。流程为：辨识 → PI 初值 → 重复速度测试与参数优化 → 最优参数的后续速度验证 → 可选位置/DOB 验证。

`alg_pid_tune` 借鉴 DiffTune+ 的敏感度传播与沿梯度预测步长，优化 $\log K_p$、$\log K_i$。机械模型按一阶响应精确离散；积分、过零衰减、条件积分和内部分段限幅参与敏感度计算。

每轮使用相同的参考频率、幅度和测试时长，前四分之一作为过渡段，不计入损失。归一化损失为：

$$
\mathcal{L}=\operatorname{mean}\left[\left(\frac{e}{s_y}\right)^2
+w_u\left(\frac{u}{u_{\max}}\right)^2
+w_{\Delta u}\left(\frac{\Delta u}{u_{\max}}\right)^2\right]
$$

其中 $e$ 为跟踪误差，$s_y$ 对应 `feedback_scale`，$u$ 为输出，$u_{\max}$ 对应 `output_max`，$\Delta u$ 为相邻输出之差；$w_u$、$w_{\Delta u}$ 分别对应 `output_weight`、`change_weight`。

内置整定的两个输出代价权重均为 0.01。根据累计残差梯度与二次近似曲率选择步长；每次增益变化限制在 0.8～1.25 倍，总范围为初值的 0.25～4 倍，最多评价 6 轮。实际损失变差时恢复已测得的最佳参数，结束优化后再验证。`optimizer.result` 提供初始损失、最佳损失、最佳增益和已评价轮数。

速度优化及最终速度验证期间不叠加前馈或 DOB；位置与 DOB 随后单独验证。

调用 `AutotuneTrackOutput()` 反馈最终执行量。出现外部限幅时，优化器标记 `kPidTuneLimited`，恢复已验证增益并转入普通速度验证。`kPidTuneInvalidSample` 同样结束优化。整定是否完成以 `AutotuneObject.state` 为准。

### 独立使用 PI 优化器

1. 用 `PidTuneInit()` 配置名义模型、周期、过渡时间、误差尺度和两个代价权重。
2. 用 `PidTuneStart()` 接入已初始化的正增益 PI。要求有限输出/积分限幅，启用抗饱和，关闭微分、前馈、死区和积分分离。
3. 每周期用 `PidTuneCalculate()` 代替本轮的 `PidCalculate()`；发送最终输出后调用 `PidTuneTrackOutput()`。
4. 一个固定测试窗口结束后调用 `PidTuneFinish()`；返回 true 时以新增益重复同一测试，false 时查看 `state` 与 `result`。

每轮使用相同参考轨迹、可比工况和初始状态，或安排足够过渡段；测试期间不修改 PI 状态。优化器使用输入到反馈的一阶名义模型，接入其他外环时需提供对应坐标下的模型。

## 单电机性能约束与帕累托整定

`alg_motor_tune` 面向 M2006、M3508 和 GM6020，复用 `alg_autotune` 建立模型与基准控制器，再测量机构特征、搜索候选并重复验证。对象使用静态 `MotorTuneObject`。

流程为：

```text
模型辨识及基准验证
  → 回到初始位置
  → 基准控制器的方向/低速/换向/工作速度测试
  → 可选三个角度的保持电流测量
  → 候选训练 → 满足约束后搜索内验证 → 下一候选
  → 从所有通过验证的候选中筛选帕累托集合
```

每轮用基准控制器回到初始位置并静止。每条轨迹 16 段，前半段三次插值、后半段保持。速度训练包含双向 12% 低速、工作速度和停转；位置训练包含小幅、全幅往返和保持。验证采用不同顺序及 25%、40%、85% 幅度。

### 物理参数与性能范围

`MotorTuneConfig` 中 `plant` 复用采样、电流、速度、连续角度限位和辨识方法。此流程要求 `speed_offset = 0`，内部关闭原 `adaptive`、`optimize`，也不叠加 DOB；位置模式复用基准位置验证，候选则重新评价整条串级控制链。

| 配置 | 含义 |
| --- | --- |
| `bandwidth` | 自然频率 $\omega_n$ 的上下限、预算因子`count`，rad/s；上限为 $0.08/\Delta t$，其中 $\Delta t$ 对应 `sampling_time` |
| `damping` | 阻尼比 $\zeta$ 的上下限、预算因子`count`，允许 $0.2$～$3$ |
| `feedforward` | 加速度前馈比例 $k_{\mathrm{ff}}$ 的上下限、预算因子`count`，允许 $0$～$1.5$ |
| `speed_amplitude` | 希望测试的最大速度，不超过 $0.8\,\text{speed\_limit}$ |
| `position_amplitude` | 位置测试幅度；启动位置至较近边界的距离至少是其 $1/0.3$ 倍 |
| `segment_time` | 单段时长，$0.1$～$10\,\mathrm{s}$，且至少 20 个采样周期 |
| `settling_band` | 相对测试幅度的稳定带，$0$～$0.5$ 之间 |
| `limits` | 跟踪 RMS、超调、调节时间、保持波动、电流变化率、饱和比例的上限；单项 0 表示不约束 |

`count`仍取1～3，1表示固定维度，固定时上下限必须相等；三个`count`的乘积作为默认预算，不再表示等间隔网格。`search_budget`可显式指定1～27组。先测中心与各活动维度的上下边界，再围绕已验证的非支配候选细化，步长依次为该维范围的1/4、1/8、1/16、1/32。没有已验证可行点时，围绕约束归一化违反程度最小的候选细化。重复参数不占新预算；预算较小时按带宽、阻尼、前馈的顺序填入初始边界。候选使用以下可解释的参数关系：

$$
K_p = 2\zeta\omega_n M - B, \qquad K_i = M\omega_n^2, \qquad K_{d,\mathrm{ff}} = k_{\mathrm{ff}}M
$$

$$
K_{p,\mathrm{position}} = 0.15\omega_n
$$

非正或无效增益标记为 `kMotorTuneInvalidDesign`。速度 PID 使用目标差分前馈、积分限幅和抗饱和；位置外环比例随速度带宽生成。导出的 `PidConfig` 与测试配置一致，评价误差取当前测试模式的反馈坐标。

速度测试幅度按初始行程缩小，实际值在 `result.profile.tested_speed`。`segment_time` 应足够容纳目标响应，否则候选会因未稳定被拒绝。

### 指标和特征含义

候选的 `training`、`validation` 分别保留两条轨迹的性能：

- `rms_error`：全轨迹误差除以测试幅度后的均方根。
- `overshoot`：保持阶段相对该次目标变化量的最大超调比例。
- `settling_time`：从进入保持阶段起，到最后一次离开稳定带后的最坏时间；不含前半程目标插值时间。
- `idle_ripple`：各保持段最后四分之一窗口的误差标准差最大值，单位为 rad/s 或 rad，不代替静差指标。
- `rms_slew`：最终发送电流指令的变化率均方根，A/s。
- `saturation`：内部电流饱和或外部分配改变指令的采样比例。

所有保持段最后10%均在稳定带内，才置`settled = true`。训练满足约束后，默认重复验证2次，`validation_repeats`允许1～3次，0使用默认值。所有重复均通过才标记可用，`validation`保留各指标最坏值，`validation_runs`记录次数。

`objectives`为`MOTOR_TUNE_OBJECTIVE_*`位掩码，0默认比较跟踪误差与电流变化率，其余指标仍按`limits`约束。`comparison_tolerance`为各指标的最小有意义改进量：所有活动目标均不差，且至少一项改善超过容差才构成支配。帕累托集合只针对已测候选，下标不代表推荐排名。

本文将 `training` 称为训练，`validation` 称为搜索内验证。两者都参与候选选择；最终确认另用未参与搜索的轨迹或负载，结果由应用记录。

`result.profile` 保存基准闭环下的双向误差、低速误差、工作速度响应比例、启动响应时间及电流、换向响应时间及位移；位置模式还保存负位置、中心、正位置的平均反馈电流与平均角度。启动阈值为实际速度幅度的 2%，延迟包含测试参考的插值过程。`startup_observed`、`reversal_observed` 为 false 时，对应时间只是观测窗口下界；计数为 0 时表示没有有效观测。保持电流仅在位置与速度均满足条件时采集。

`reversal_travel` 同时受闭环响应影响，不能当作减速箱回程间隙。姿态保持电流作为样本保留，当前不据此自动改变模型。

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

一次训练或验证为$16\,T_{\mathrm{segment}}$秒，特征测量另需一轮。默认每组通过的候选共用1轮训练与2轮验证；27组全部通过时为81轮，另加辨识与回中时间。`search_budget`限制候选数，不能据此当作运行时间上限。

## 闭环多正弦参考

`alg_multisine`只生成参考，不持有电机或控制器。应用将参考接入已有速度或位置控制器，记录实际电流和速度，离线分析电流到速度的频响。它不替代主动整定。

```c
#include "alg_multisine.h"

static MultisineObject excitation;
MultisineConfig config = {
    .sampling_time = 0.001f,
    .fundamental_hz = 1.0f,
    .amplitude = 2.0f,
    .offset = 0.0f,
    .lower = -3.0f,
    .upper = 3.0f,
    .harmonic = {1, 3, 7, 13},
    .tone_count = 4,
    .settling_periods = 2,
    .measurement_periods = 4};

if (MultisineInit(&excitation, &config))
{
  (void)MultisineStart(&excitation);
}
```

这些数值只说明接口。周期任务先采集上一周期参考及其`sample/period/measuring`对应的反馈，再调用`MultisineCalculate()`生成下一次参考，交给已有控制器。首周期平滑增幅；其后输出重复的Schroeder相位多正弦。幅值按分量幅值之和限制，整数谐波须互异且低于Nyquist频率。`period_ticks`为实际周期长度，`fundamental_hz`为量化后的实际基频。至少保留1个过渡周期和2个测量周期。

最后一个测量周期输出后，下一次调用返回`offset`并退出；后续业务参考的过渡由应用处理。取消使用`MultisineCancel()`。任一控制限幅都应记录为`limited`，不能只记录参考是否越界。数据格式、工具变量频响及命令见[离线工具](../tools/tuning/README.md)。

## 离线对照

`tools/tuning/tune.py` 提供 `identify`、`frequency`、`propose`、`replay` 四个命令，分别用于窗口/延迟及 LS/IV 对照、完整多正弦周期分析、贝叶斯候选建议和实测候选表回放。输入分别为时序记录或逐次候选表；采集器不会自动生成候选表。命令读取数据并输出报告，H7 控制链不依赖 Python。

保留原始波形、失败试验、工况和逐次候选指标，供复盘及后续本地 NN 训练使用。IV 与闭环频响需要独立外部参考；搜索策略比较需要实测候选表和最终确认数据。已有记录的辨识对照见[工具说明](../tools/tuning/README.md#已有记录的对照)，新激励方案仍需台架验证。

## 正常运行中的自适应

成功 `AutotuneApply()` 后，配置启用 `adaptive` 时，每周期调用 `AutotuneAdapt()`，传入运行速度 PID、反馈角速度和实际反馈电流。其返回值为增量加速度前馈电流，由应用纳入总电流请求。

更新检查动态激励、模型质量与惯量范围，增益缓慢受限变化；摩擦和剩余负载由积分/DOB 处理。

启用 `optimize` 时，自适应保留已验证增益相对模型设计值的比例，不重复主动测试轨迹。

保存模型、设计自然频率和控制参数后，可用 `AutotuneRestore()` 接入已经验证的模型及运行对象。该函数不读写 Flash，持久化由应用实现。

## 方法来源

- [SIFt-RLS: Subspace of Information Forgetting Recursive Least Squares](https://arxiv.org/html/2404.10844v1)，2024：当前按单条标量观测实现四维信息子空间遗忘。
- [DiffTune+](https://proceedings.mlr.press/v211/cheng23b.html)，L4DC 2023：当前借鉴低维敏感度传播及沿梯度二次预测步长，使用本工程的 PI、损失函数和增益边界。
- [WPILib SysId测试流程](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/creating-routine.html)：参考准静态与动态分段测试。
- [机器人动力学工具变量辨识](https://doi.org/10.1109/TCST.2013.2246163)：离线实现采用线性两阶段最小二乘对照，未移植完整机器人辨识方法。
- [闭环多正弦辨识研究](https://doi.org/10.1016/j.conengprac.2025.106457)：参考闭环外部激励与频率分析，当前只实现参考生成和频域比值。
- [Noisy Multi-Objective Bayesian Optimization](https://arxiv.org/abs/2105.08195)：离线工具采用有限候选池、单点后验采样的简化对照。
