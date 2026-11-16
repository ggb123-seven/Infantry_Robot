# Yaw 二次摩擦补偿实现与软件验证

日期：2026-11-16

用户已确认修改 `User/module/gimbal.c/.h`，并在 `Gimbal_Config_t` 末尾追加一次项、二次项和拟合速度上限三个字段。本轮不修改 `config.c`，不烧录、不写目标板 RAM

## 实现

仅 Yaw 在速度 PID 输出后叠加摩擦前馈，合成电流继续经过现有轴电流限幅和 `Gimbal_DumpOutput` 状态闸门。Pitch 不使用该前馈

近水平底座下，使用相对机械目标速度近似：

```text
u = motor_speed_rad_s + (target_speed_rad_s - speed_rad_s)
P(v) = a0 + a1*v + a2*v²
I_ff(u) = P(w) * u/w                 |u| < w
          sign(u) * P(min(|u|, vmax)) |u| >= w
```

`target_speed_rad_s` 为外环合成、速度限幅与机械边界减速后的本周期目标；`motor_speed_rad_s` 是相对底座的编码器速度，`speed_rad_s` 是上部的大地系 Yaw 欧拉角速度

底座静止时，编码器与惯性反馈近似相同，`u` 近似目标速度，因此未起步时也能得到摩擦补偿。保持世界朝向且底座自旋时，世界目标和反馈接近零，`u` 保留机械相对转速。该关系在底座倾斜或存在明显横滚、俯仰耦合时只是近似，尚无此工况的实物验证

`vmax=0` 明确关闭补偿；启用时要求 `vmax>=w>0`。零速过渡连续且零点电流为零，超上限使用边界模型值。原先 `a0=0` 关闭的操作应改为 `vmax=0`；一次项和二次项非零时单独将 a0 置零不表示关闭

本次追加字段使配置类型和包含它的实例布局变化，需要使用新固件及对应 ELF 重新解析调试变量。保留原 RAM config 副本、每周期 Control 入口快照、PID 持久参数引用及输出闸门

## 后续 RAM 验证候选值

下表列出 `gimbal.config` 下的字段，尚未写入目标板

| 字段 | 候选值 | 依据 |
| --- | ---: | --- |
| yaw_friction_current_a | 0.2648070877 A | 稳态反馈电流二次拟合 a0 |
| yaw_friction_linear_a_per_rad_s | -0.0055278781 A/(rad/s) | 拟合 a1，负值合法 |
| yaw_friction_quadratic_a_per_rad_s2 | 0.00195745266 A/(rad/s)² | 拟合 a2 |
| yaw_friction_smoothing_rad_s | 0.2 rad/s | 现有默认值，线性过渡宽度为未实测起点 |
| yaw_friction_max_speed_rad_s | 3.4 rad/s | 实际标定速度绝对值约 0.408～3.421 rad/s，取范围内上限 |

拟合证据见 [多速度采样与拟合报告](yaw_friction_multispeed_20261116.md)。独立段均值预测 RMSE 为 0.001267 A，段内反馈电流标准差约 0.023～0.035 A，不能将均值误差解释为瞬时补偿精度。0.408 rad/s 以下模型与 0.2 rad/s 过渡均未实测；0.2～0.408 rad/s 使用的是低速外推

候选模型在 w=0.2 处约 0.263780 A，在 vmax=3.4 处约 0.268640 A。近零区的斜率约 1.319 A/(rad/s)，编码器量化及两种反馈不同步可能影响启停手感，必须上板比较后确认过渡宽度

后续验证应在保存两轴现有 RAM 调参、核对新 ELF 并保持 RELAX 后进行。先保持 vmax=0 写入其余字段，读回确认后最后设置 vmax；关闭时先将 vmax 写为零。逐项写入不是多字段原子事务，Control 每周期只保证本周期使用同一个已校验副本

`config.c` 未设置新增字段，按 C 初始化规则为零，因此新固件复位后补偿仍关闭，拟合值没有固化进默认配置

## 软件验证

基线为 `e267da7b1fb941cfb745e2e26f18e7c50db3914f`。主机测试使用实际 `gimbal.c`、`config.c`、`pid.c`、`filter.c`、`user_math.c`，仅将未调用的硬件总线类型换为主机标识。原模块作为独立翻译单元重命名四个公共函数，进行默认关闭时的逐周期对照

```text
gcc -std=gnu11 -fgnu89-inline -O0 -g -Wall -Wextra -Werror
    -IUser/task/.yaw_friction_host_20261116 -IUser -IUser/component
    User/task/.yaw_friction_host_20261116/test.c
    User/task/.yaw_friction_host_20261116/baseline.c
    User/module/config.c User/component/pid.c User/component/filter.c
    User/component/user_math.c -lm -o User/task/.yaw_friction_host_20261116/test.exe

PASS checks=70072 grid=20001 baseline_cycles=2000 worst_model_error=3.11991651e-008 A
```

覆盖：

- ±10 rad/s 的 20,001 点参考式扫描、正反奇对称、零点、过渡和上限的相邻浮点数连续性、FLT_MAX 输入保持上限值
- 静止起步、零世界目标时底盘自旋、目标与底盘速度抵消、最终目标与摩擦模型上限、Pitch 隔离
- 本周期配置快照、DumpOutput 使用已采纳方向、下周期拒绝非法 RAM 快照并输出两轴零电流
- 五个摩擦字段的 NaN/Inf、负上限、零宽度、上限小于宽度、端点溢出、内部负极值及内部超电流极值
- 先除后二分可能溢出的极大系数顶点案例、合法负一次项和零二次项
- 明确关闭、PID 与前馈合成后的最终限流、机械边界减速后的目标、反向输出符号
- 默认关闭与原模块连续 2,000 周期两轴输出、状态及配置有效性完全一致，含正反速度、角度模式、待机、单轴反馈离线及恢复

临时主机测试源码 SHA-256：`4684c862b0989019e330336bb9593d059e98b6abb15c4b4f724aebe5dda64777`，按仓库规则在记录后清理临时测试目录

固件构建使用 `aemb-build-cmake` 技能：

```text
python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --list-presets
python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug
```

Debug 配置与构建通过，产物 `build/Debug/Infantry_Robot.elf`。使用 compile_commands.json 中的实际 ARM 工具链与包含路径，对 gimbal.c 附加 `-fsyntax-only -Wextra -Werror -Wmissing-prototypes -Wstrict-prototypes -Wmisleading-indentation`，严格检查通过

本次 ELF SHA-256：`5f844f536b2c8a1f368a01b613ab5b42aee4d0bfcefc99d009bfb650433ca3b5`

`arm-none-eabi-size`：text=132088 B、data=208 B、bss=50464 B，即 FLASH 装载约 132296 B、RAM 静态约 50672 B

## 栈与实时性边界

| 项目 | Debug 静态栈 |
| --- | ---: |
| Task_motor_gimbal | 368 B |
| Gimbal_Control | 304 B |
| Gimbal_ControlAxis | 48 B |
| Gimbal_RunAxis | 104 B |
| PID_Calc | 72 B |
| Gimbal_IsConfigValid | 16 B |
| Gimbal_IsFrictionConfigValid | 32 B |
| Gimbal_CalcYawFriction | 24 B |
| Gimbal_FrictionMagnitude | 16 B |

云台仍为 3072 B 静态任务栈。上述表按调用路径嵌套解释，PID 与摩擦前馈为先后调用，不能将表内所有函数简单累加作为真实栈占用。库函数、异常浮点上下文、其他任务阶段均需要纳入运行时验证

本轮没有烧录新固件，因此没有新固件的 FreeRTOS 栈高水位、控制周期最大执行时间和 deadline 实测，不能宣称完成栈安全或 500 Hz 实时性验收。控制计算取消原 tanhf，使用浮点多项式与简单分段；集中参数校验新增曲线端点及内部顶点检查

## 防御性检查自查

| 本轮新增或迁移检查 | 信任边界与真实触发场景 | 上游是否已检查 |
| --- | --- | --- |
| a0 有限及 [0,max_current]、w 有限且正 | 原检查迁移到摩擦校验函数，Init / Control 快照入口，RAM 写错会触发 | 本入口执行一次 |
| a1、a2 有限 | Init / Control 配置边界，调试写入 NaN/Inf | 无 |
| vmax 有限且非负 | 同一配置边界，调试写入非法上限 | 无 |
| 启用时 vmax>=w | 同一配置边界，避免过渡区超过模型区间 | 无 |
| P(w)、P(vmax) 有限、非负且不超过轴电流上限 | 同一配置边界，有限但过大系数仍可使计算溢出、反向或超限 | 系数单独有限不能保证曲线合法 |
| 非零 a2 的区间内顶点检查 | 同一配置边界，端点合法但内部负值或超限 | 端点不能覆盖内部极值 |
| friction_target_rad_s 有限 | 控制浮点合成链出口，反馈与目标差的计算可能溢出，且 fminf 限速可能掩盖异常 | 目标单独有限不能保证该合成结果有限 |

vmax==0 是明确启停语义，fabs(u)<w 是连续函数分段，均不重复检查已校验的字段。计算帮助函数不重复判空、isfinite 或参数范围校验。未新增判空或下游配置保护性 return，没有待删除的重复防御项

人工复核保持 module 层依赖方向；未新增 RTOS/HAL 对象、ISR 共享状态或故障状态。Control 快照、既有状态机与输出闸门位置均未改变。新增注释为简体中文且无句末句号，花括号、120 列与行尾空白扫描通过

`User/module/config.c` SHA-256 保持 `3228F43578255F33CF5B2A5A7830605EA88C07584F61C171A7E75F3F2AC7A677`

## 架构门禁与最终复核

初次门禁 HW、SPEC 通过，ARCH 报告 `gimbal.c` 971 行超过既有单文件 900 行上限。用户随后明确选择仅将 gimbal.c 上限提高到 1000 行、保持实现集中；已同步 arch-check.ps1、architecture 与 conventions，其他 C 文件仍限 800 行

临时测试目录已逐项清理并确认不存在，未删除仓库正式测试资产。最终执行：

```text
python .auto-embedded/scripts/check.py
ARCH: PASS
HW: PASS
SPEC: PASS
ALL PASS
```

调整门禁后再次通过 Debug 构建，ELF 哈希与上文一致，本轮确认上限后没有再修改业务源码。70,072 条主机断言、ARM 严格语法与静态栈证据沿用上述同一份实现的验证结果

三层人工复核：验证门已核对构建和主机证据，实测边界已单列；硬件层未改 hw-lock、引脚、外设或 CubeMX，HW 门禁通过；代码质量层未改 main.c、未新增 volatile 共享状态或临界区，保留原 Control 入口 volatile RAM 读取，配置副本及持久快照和输出闸门保持原职责

通过后已用 `task.py promote conventions convention` 沉淀补偿目标坐标、关闭语义、校验位置与实物验证边界。本地提交仅纳入本轮源码、验证文档、行数约定及该条沉淀；仓库既有未提交内容单独保留，不推送远端
