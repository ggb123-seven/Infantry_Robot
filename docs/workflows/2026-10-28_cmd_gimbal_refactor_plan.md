# 命令层拆分与云台模块精简方案

状态：步骤 1～3 已完成并通过软件验证；用户已确认结构体草案、rejected_version 和非法配置拒绝策略，正弦测试留在原文件使用编译开关隔离

实施记录：[命令层拆分与云台模块精简验证记录](2026-10-28_cmd_gimbal_refactor.md)

## 0. 实施前必读

- 先完整阅读根目录 `AGENTS.md`、`agent.md`，按其中排版、注释、分步验证和本地提交规则执行
- 源码修改授权：用户已同意本方案涉及 `User/task`、`User/module`、`User/component` 的修改与新增，以及 CMake、Keil 工程文件同步；不得修改 `User/bsp`、`User/device`、`Src`、`Inc`、`.ioc`
- 本文第 3 节所有结构体名称与字段均为候选草案。按 `AGENTS.md`，新建或改动结构体前必须先向用户确认，未确认不得定稿
- 不修改 `gimbal_governor.c` 中的状态机、阈值和电流包络逻辑，只允许调整它读写目标角的接口（见步骤 4）
- 本方案总原则：除第 2 节明确列出的变化外，云台和底盘的实际控制行为必须与当前提交 `f6801cf` 一致

## 1. 背景与目标

现状问题：

- `task/motor_gimbal.c` 承担 DR16 映射、死区、加减速平滑、遥控使能状态机和正弦测试，并直接读取 `gimbal.yaw.target_angle_deg`、`gimbal.yaw.state` 等模块内部状态来累加目标角
- 目标角同时被 task（遥控累加）和 governor（制动、跟随时改写）修改，交接处是近期多次 bugfix 的来源
- `motor_chassis.c` 与 `motor_gimbal.c` 各有一份通道归一化和死区实现
- `module/gimbal.c` 中 `period_s` 与反馈有效性在多处重复校验；`Gimbal_RunAxis` 混合串级 PID、三处机械限位、积分回退和 governor 钩子
- 每周期逐字段比较 PID 参数检测在线调参，需要 `applied_*` 副本

目标：

1. 新增命令层，遥控数据只在命令层解析，控制任务只消费统一命令
2. 云台命令改为“目标来源 + 角速度”，目标角由云台模块内部唯一持有
3. 云台模块内部精简：入口统一校验、`Gimbal_RunAxis` 固定流水线、配置版本号触发采纳
4. 正弦测试与正式控制路径隔离

## 2. 行为约束

### 必须保持不变

- 遥控手感：满杆 6 rad/s、5% 死区、加减速 60000 °/s²、4 ms 平滑、0.1 °/s 收敛阈值、方向符号均为 -1
- 遥控增量累加基于上周期限幅后的目标角，顶住机械限位继续推杆不得在背后积累目标
- 首次进入 ANGLE、单轴失联恢复、模式切换时捕获当前姿态并清零前馈
- 单轴电机反馈超时只停该轴，右拨杆保持上位时反馈恢复自动起控
- 遥控失联、帧龄超过 `DR16_RECEIVER_OFFLINE_TIMEOUT_US`、时间回退：云台 RELAX，须右拨杆回中再拨上才重新使能
- 任一轴 FAULT 或 IMU FAULT：两轴均零输出，须右拨杆回中（命令 RELAX）再拨上才重新使能
- 云台输出邮箱发布失败：两轴停机，须回中再拨上
- `GIMBAL_MODE_SPEED` 速度环整定模式行为不变
- governor 全部状态、阈值、制动与恢复时序不变
- 底盘拨杆映射、100 rpm 尺度、自旋 0.30、离线停机行为不变

### 允许的变化

- 调试器在线修改 `gimbal.config` 后，需额外递增配置版本号才生效（见步骤 5）
- 正弦测试改为绝对目标来源，开关变量可改名或移到调试文件
- 单轴失联等待或正弦测试期间，命令层的遥控平滑速度不再强制归零；恢复首周期仍由模块捕获姿态并清零前馈，之后摇杆速度不再从 0 重新爬升。按 60000 °/s² 和 4 ms 平滑，差异在数毫秒内，需在验证记录中说明

## 3. 目标结构

### 3.1 数据流

```text
DR16 任务 --(两个现有 DR16 邮箱，拓扑不变)--> 底盘任务 / 云台任务
                                                  |
                                    module/cmd 解析为 Chassis_Command_t / Gimbal_Command_t
                                                  |
                                    module/chassis、module/gimbal 控制
```

- 本轮不新增命令任务，`module/cmd` 的解析函数由两个控制任务各自调用，邮箱和任务拓扑不变，降低 RTOS 改动风险
- 参考工程 `F:\RM\未命名文件夹\User\task\command.c` 的独立命令任务结构留作后续接入键鼠、上位机时再评估；其用 `osKernelLock()` “防止中断”的写法禁止照搬
- 参考工程把具体拨杆业务放在 `component/cmd`，本工程按分层放在 `module/cmd`；只有与业务无关的通道整形进入 `component`

### 3.2 新增 component：遥控输入整形（文件名候选 `component/input_shaping.c/.h`）

只放无业务语义的纯函数，不含状态结构体：

- 通道归一化加死区：输入原始计数、满量程、死区比例，输出 [-1, 1]。合并 `MotorChassis_NormalizeDR16Channel`、`MotorChassis_ApplyDeadzone` 与 `MotorGimbal_NormalizeChannel`。两份实现在死区边界处结果相同（边界值经公式也为 0），需用主机测试逐点对比确认
- 速度指令平滑：原样迁移 `MotorGimbal_UpdateRemoteRate`（换向先过零、加减速上限、一阶平滑、小误差收敛），参数改为函数入参

### 3.3 新增 module：命令解析（文件名候选 `module/cmd.c/.h`）

结构体草案，名称和字段须先经用户确认：

```c
/*
 * 云台遥控使能状态，原样迁移 MotorGimbal_RemoteState_t：
 * - WAIT_NEUTRAL：启动、遥控失联、云台锁存故障或输出发布失败后，等待右拨杆中位
 * - READY：已确认中位，输出 RELAX
 * - ACTIVE：右拨杆上位，输出 ANGLE
 */
typedef enum
{
    CMD_GIMBAL_WAIT_NEUTRAL = 0,
    CMD_GIMBAL_READY,
    CMD_GIMBAL_ACTIVE,
} Cmd_GimbalState_t;

/*
 * 云台命令解析上下文，由云台任务独占：
 * - state：遥控使能状态
 * - yaw_rate_deg_s、pitch_rate_deg_s：上周期平滑后的遥控角速度，单位度每秒
 */
typedef struct
{
    Cmd_GimbalState_t state;
    float yaw_rate_deg_s;
    float pitch_rate_deg_s;
} Cmd_Gimbal_t;
```

接口草案：

- `Cmd_ParseChassis(const DR16_State_t *rc, bool rc_valid, uint64_t now_us, Chassis_Command_t *out)`：原样迁移 `MotorChassis_ApplyDR16Input` 的拨杆映射与离线判定
- `Cmd_ParseGimbal(Cmd_Gimbal_t *ctx, const DR16_State_t *rc, bool rc_valid, uint64_t now_us, float period_s, bool gimbal_fault, Gimbal_Command_t *out)`：迁移 `MotorGimbal_ApplyDR16Input` 的离线判定、使能状态机和摇杆平滑，只输出模式和角速度，不再读取云台内部目标或轴状态
- `Cmd_ResetGimbal(Cmd_Gimbal_t *ctx)`：输出发布失败时由任务调用，回到 WAIT_NEUTRAL 并清零平滑速度
- `gimbal_fault` 由任务通过新增的只读查询 `Gimbal_HasLatchedFault(const Gimbal_t *)` 取得（任一轴 FAULT 或 IMU FAULT）。必须保留该输入：现有逻辑在单轴故障时会让两轴一起 RELAX，若只依赖模块锁存，pitch 故障时 yaw 会继续运行，违反第 2 节约束
- DR16 邮箱读取与缓存仍留在任务内，命令层只处理已取得的快照
- 遥控通道方向、灵敏度、加减速等宏随解析逻辑迁入 `module/cmd.c`，数值不变

### 3.4 云台命令语义（`module/gimbal.h`）

结构体草案，须经用户确认：

```c
/*
 * 单轴目标来源：
 * - INCREMENT：模块以上周期限幅后的目标角为基准累加 rate × period，用于遥控、键鼠
 * - ABSOLUTE：直接采用 angle_deg，用于自瞄和正弦测试
 */
typedef enum
{
    GIMBAL_TARGET_INCREMENT = 0,
    GIMBAL_TARGET_ABSOLUTE,
} Gimbal_TargetSource_t;

/*
 * 单轴命令：
 * - source：目标来源，按轴独立，便于正弦测试只接管 yaw
 * - rate_rad_s：INCREMENT 下为累加速度兼前馈，ABSOLUTE 下为前馈，SPEED 模式下为速度目标
 * - angle_deg：仅 ABSOLUTE 使用，当前 INS 坐标，yaw 为累计角
 */
typedef struct
{
    Gimbal_TargetSource_t source;
    float rate_rad_s;
    float angle_deg;
} Gimbal_AxisCommand_t;
```

`Gimbal_Command_t` 保持 `mode + yaw + pitch` 形态，`command` 参数改为 `const`，模块不再回写调用方命令。

模块内部实现要点：

- 新增内部设定值类型（沿用现在 `target_angle_deg + target_speed_rad_s` 的形态），`Gimbal_Control` 先把公开命令解析成内部设定值，再执行现有的姿态捕获、`GimbalGovernor_PrepareYaw`、`Gimbal_ControlAxis`。governor 继续改写内部设定值，其状态机和接口语义不变
- 解析顺序必须是：命令解析 → 起控、失联恢复、模式切换时的姿态捕获 → governor → 单轴控制，保证捕获和 governor 覆盖优先于累加
- INCREMENT 的累加仅在轴 RUNNING 且 `axis->target_angle_deg` 有限时进行，与现有 task 条件一致；否则沿用上周期内部设定值，由捕获逻辑处理
- 累加基准取 `axis->target_angle_deg`（上周期限幅后的目标），governor 锁存期间该值即恢复姿态，行为与现在 task 读取 `gimbal.yaw.target_angle_deg` 一致
- 内部设定值的持久化位置从 task 全局 `gimbal_command` 移入 `Gimbal_Axis_t`（字段名待确认），`gimbal_command` 只作为观测用的命令副本

### 3.5 正弦测试

- 移到 `task/gimbal_debug.c/.h`（候选名）或在 `motor_gimbal.c` 中用 `#if` 编译开关隔离，用户择一
- 改用 yaw `ABSOLUTE` 来源：`angle_deg = 中心 + A·sin(φ)`，`rate_rad_s = 前馈开关 ? A·ω·cos(φ) : 0`，保留“有无前馈”对比能力
- 中心仍取启动测试时的 `gimbal.yaw.angle_deg`，调试工具在任务层只读该观测量
- 启用条件不变：命令为 ANGLE、yaw RUNNING、IMU READY；退出时 yaw 回到 INCREMENT，以最后限幅目标为基准

## 4. 实施步骤

每步独立完成复核、构建、测试并本地提交，提交信息使用简体中文。

### 步骤 1：通用输入整形

- 新增 3.2 的 component 文件，CMake 与 Keil 工程同步加入
- 底盘、云台任务改调新函数，删除各自私有实现
- 主机测试：新旧归一化函数在 [-700, 700] 全部整数逐点相等；平滑函数用随机序列与旧实现逐步相等

### 步骤 2：命令层（底盘部分 + 云台使能状态机）

- 新增 `module/cmd`，迁移底盘拨杆解析和云台使能状态机
- 此步云台仍输出旧语义命令：命令层输出模式和平滑速度，目标角累加暂留在 `motor_gimbal.c`，保证本步行为可单独对比
- 新增 `Gimbal_HasLatchedFault`

### 步骤 3：云台命令语义切换

- 按 3.4 修改 `Gimbal_Command_t` 与 `Gimbal_Control`，删除 task 内的目标角累加
- 按 3.5 迁移正弦测试
- 同步更新 `gimbal.h`、`motor_gimbal.h` 的接口说明

### 步骤 4：模块内部精简（纯重构）

- 统一校验：`Gimbal_Control` 入口一次性校验 `period_s`，结果作为参数下传；`Gimbal_RunAxis` 与 `Gimbal_ControlAxis` 的 OFFLINE 分支删除重复的周期检查，只保留反馈和命令的有限值检查。`gimbal_governor.c` 内部自带的检查不动
- `Gimbal_RunAxis` 拆成固定流水线，每段一个静态函数：目标角限幅 → 角度环 → 速度目标限幅（机械限速 + governor 限速）→ 速度环 → 输出限幅（governor 电流 + 机械电流阻断 + 积分反算）
- 积分回退集中处理：入口记录两环旧积分，各段只返回“是否受限”，末尾统一回退，回退条件与现有三处逐一对应
- 输出必须与重构前逐周期一致，见第 5 节

### 步骤 5：配置版本号

- 用户已确认采用版本号方案，字段设计如下：
  - `Gimbal_t` 保留 `config` 作为调试器可写的待提交配置，新增 `config_version`、`active_config`、`active_version`、`config_reject_count`
  - 非法新配置保留旧配置并记录拒绝次数；同一版本只处理一次，改正后须再递增版本号
  - 用户已确认增加 `rejected_version`，记录最近拒绝的版本，使 `active_version` 始终表示正在运行的配置版本；处理条件为 `config_version != active_version && config_version != rejected_version`
  - 调试器须先改完 `config` 各字段再递增 `config_version`；版本号为 `uint32_t`，用 `!=` 比较以容忍回绕，`config_version` 加 `volatile` 并注明由调试器写入、云台任务读取
  - 初始化时 `config`、`active_config` 均复制默认配置，版本号与拒绝计数置 0；初始配置非法时 `Gimbal_Init` 仍返回失败
  - `d_cutoff_freq` 本轮仍只在初始化时生效
- 版本递增时按修改内容分三类处理：
  - 配置非法（非有限值、限幅非正、`min_angle_deg > max_angle_deg`、PID `range` 非 0 等）：整份拒绝，保留旧配置，拒绝计数加 1，运行中的轴继续运行。这是相对现有“运行中配置非法即 FAULT”的唯一行为变化
  - 合法且零点或 `reversed` 改变：采纳，运行中或失联等待的轴进入 FAULT，与现状一致，因为机械坐标已失效
  - 合法且仅 PID、限幅等改变：采纳并清空对应积分，轴继续运行，与现状一致
- 版本变化时校验整份 `config`（含两个超时参数），合法则复制到 `active_config`；零点或方向变化以新配置与 `active_config` 比较判定
- 模块所有运行时读取改用 `active_config`；PID 参数指针指向 `active_config` 中的两环参数。删除 `applied_zero_encoder_deg`、`applied_reversed`、`applied_angle_pid`、`applied_speed_pid` 与 `Gimbal_IsPidChanged`
- `GimbalGovernor_TrackCurrent` 读取的 `axis->applied_speed_pid` 改为 `axis->speed_pid.param`，这是对 governor 唯一允许的改动
- `Gimbal_EnterRunning` 中读取的 `applied_speed_pid.k` 同步改为新位置
- 底盘 `Chassis_ApplyPidConfig` 有同类逐字段比较，本轮不改，记录为后续项

### 步骤 6：收尾

- `gimbal.h` 只保留接口契约，实现细节（欧拉角速度公式、历史枚举值说明等）移到 `gimbal.c` 对应位置
- 更新 `agent.md` 进度、`docs/workflows/INDEX.md`，验证记录写入 `docs/workflows/2026-10-28_cmd_gimbal_refactor.md`

## 5. 验证要求

- 每步运行：`python .auto-embedded/scripts/check.py`；`python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug`；实际 ARM 编译器 `-Wall -Wextra -Werror -fsyntax-only`；`git diff --check`
- 行为对比（重点）：用主机 GCC 分别编译 `f6801cf` 的旧 `motor_gimbal.c` 遥控与正弦逻辑、`gimbal.c`、`gimbal_governor.c` 与新实现，替身化邮箱和时间，喂入同一组合成序列，逐周期比较两轴输出电流、目标角、目标速度、轴状态与 governor 状态
- 合成序列至少覆盖：上电拨杆在上位、中位确认后起控、满杆与换向、pitch 顶限位持续推杆后回拉、遥控失联与恢复、帧龄边界 100000/100001 us、单轴反馈超时与恢复、pitch 单轴故障、IMU 故障、输出发布失败、ANGLE 与 SPEED 切换、悬空残差触发制动到 HOLD、RESTORING、GUARDED、MONITOR 全流程、正弦测试开关与前馈开关
- 步骤 1、2、4 要求逐周期结果一致（浮点允许 1e-6 相对误差，需说明来源）；步骤 3 只允许第 2 节列出的差异，需逐项标注；步骤 5 仅在配置修改场景出现差异
- 临时测试源码和可执行文件放在 `build/` 下，验证后清理
- 上板项目由用户执行，软件验证不得替代：遥控手感对比、pitch 限位、悬空疯转保护与放回地面自动恢复、遥控失联停机与恢复、在线调参递增版本号后生效
