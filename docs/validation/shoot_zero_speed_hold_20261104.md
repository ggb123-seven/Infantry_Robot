# 摩擦轮下位回中位零速制动验证

日期：2026-11-04

## 本次行为

- 右拨杆下位：两轮 enabled=true，目标均为 8000 rpm
- 从下位回中位：两轮 enabled=true，目标均为 0 rpm；中位持续零速闭环，不按时间或停稳自动撤驱
- 中位返回下位：恢复 8000 rpm
- 上位：失能，目标为零，并清除运行记忆
- 上电即在中位，或上位、遥控失效后直接回中位：保持失能，须先经过下位
- 中位拨弹仍为 FEEDER_CMD_RELAX；摩擦轮不再与拨弹运行条件共用使能
- 缺新命令、反馈过期与模块故障沿用既有 Shoot_Control / Shoot_DumpOutput 规则；保持期间锁存故障不会因中位零目标解除，使用上位失能解除

## 修改范围和接口

生产行为只修改 User/task/motor_trigger.c，User/task/user_task.h 同步邮箱集中说明。任务内私有枚举保存撤驱、运行和零速保持，MotorTrigger_ParseShootCommand 集中处理状态转换。复用 Cmd_ParseFeeder 输出的已校验档位，仅在有效待机档通过同一快照的右拨杆区分中位与上位，不重复在线、帧龄或协议合法性检查

Shoot_Command_t、全部结构体布局、RTOS 邮箱和线程、config.c、可调 config 副本和周期快照、PID、BSP、设备驱动、CubeMX 均未修改。函数返回现有命令类型，不新增结构体。命令阶段只在拨弹任务内访问，不引入 ISR 共享变量或临界区

## 软件验证

### 机械门禁与构建

初次直接运行 python .auto-embedded/scripts/check.py 因本机 PowerShell 未签名脚本策略阻止 ARCH 脚本执行，HW、SPEC 通过。仅设置检查进程及其子进程的 PSExecutionPolicyPreference=Bypass 后重新运行，ARCH、HW、SPEC 全部 PASS；未修改机器或用户持久执行策略

```powershell
$env:PSExecutionPolicyPreference = 'Bypass'
python .auto-embedded/scripts/check.py
python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug
git diff --check
```

采用 aemb-check 和 aemb-build-cmake 技能，复用 Debug 预设和原有构建目录。CMake 配置和构建成功，固件为 build/Debug/Infantry_Robot.elf。使用 build/Debug/compile_commands.json 中 motor_trigger.c 的正式 ARM 命令，去掉输出目标和 -c，添加 -Wextra -Werror -fsyntax-only，退出码 0，无诊断

### 真实生产代码主机回归

临时测试直接包含正式 motor_trigger.c 与 motor_shoot.c，链接正式 cmd.c、feeder.c、shoot.c、config.c、pid.c、filter.c、user_math.c，使用正式 DR16、反馈和任务运行时结构。仅时钟与 CMSIS 邮箱函数使用替身；BSP_CAN_t 替身隔离 HAL，FreeRTOS 替身只提供 1000 Hz 节拍宏，未重写生产命令状态机或 PID

主机 GCC 编译参数：-std=gnu11 -fgnu89-inline -O0 -g -Wall -Wextra -Werror，链接 -lm。PowerShell 使用参数数组传递包含点号的 -IUser/task/.shoot_brake_check 路径，避免原生参数拆分。编译和执行退出码均为 0

```text
PASS 1 startup-middle-disabled
PASS 2 down-middle-and-persistent-zero-hold
PASS 3 restart-up-disarm-and-clear-memory
PASS 4 invalid-cache-offline-timeout-clock-switch-state-clear-memory
PASS 5 remote-100ms-boundary
BRAKE currents=-3.000,-3.000 A gain=0.199703708 A/rpm step=-1597.630 A
PASS 6 8000-to-zero-negative-current-with-feedforward-clamped
PASS 7 zero-speed-stays-enabled-and-rejects-both-sign-disturbances
PASS 8 zero-to-8000-reaccelerates-up-zero-current
PASS 9 missing-command-disables
PASS 10 either-feedback-expired-stops-both-and-recovers-hold
PASS 11 fault-latched-in-middle-up-clears
PASS 12 real-trigger-task-mailbox-publish-failure-cache-and-read-error
PASS 13 real-shoot-task-zero-target-missing-command-recovery
ALL PASS: 13 groups
```

连续执行 10000 次中位命令转换仍保持 enabled=true、目标为零。遥控测试覆盖无缓存、离线、100000/100001 us 边界、源时间超前、非法拨杆和非法私有状态；异常后中位不会恢复旧记忆

速度环采用 config.c 原始参数。先给两轮 8000/7000 rpm 合成反馈、8000 rpm 目标，再将目标变为零：输出均为 -3 A，状态保持 SHOOT_RUNNING。随后正转低速反馈保持负电流；两轮零反馈时输出为零但仍为 SHOOT_RUNNING；+5/-5 rpm 扰动得到相反方向电流。中位回下位重新产生正向电流，上位输出归零

现有目标差分前馈系数为约 0.199703708 A/rpm，8000→0 的首周期前馈候选约 -1597.630 A，叠加 PID 后仍由现有限幅压到每轮 -3 A。该数值为限幅前软件候选，不能当作真实电机电流；参数未修改，电流响应和回生等硬件现象本轮未测量

实际拨弹任务的邮箱回归包含下→中、持续中位、中→下、上位、上位后中位、空邮箱缓存、读取错误以及拨弹电流发布失败；中位拨弹输出为零，拨弹发布失败不阻断摩擦轮零速命令。实际摩擦轮任务回归包含零速命令、缺少本周期新命令失能、收到新零速命令后恢复

主机测试文件、替身头和可执行程序均位于本次专用的 User/task/.shoot_brake_check，证据记录后全部清理，未删除正式测试资产

## 三层复核及校验自查

- 验证门：真实生产代码主机回归 13 组、ARM 严格语法检查和 Debug 构建通过；尚未烧录、运行电机或写入目标 RAM
- 硬件合规：没有新增或修改引脚、CAN ID、DMA、中断、时钟和硬件资源，HW 门禁通过
- 代码质量：保持任务编排和模块控制边界，命令状态只保存操作顺序；实际速度环、电流限幅及唯一输出闸门继续由 Shoot 模块负责。未修改 main.c、RTOS/HAL 对象创建或 ISR，无新增共享状态或临界区
- 生产改动新增判空、isfinite、数值范围检查和保护性 return 均为零；删除的重复防御检查为零
- 新增两个 switch 的 default 分支分别负责非法命令阶段、离线或未识别命令的撤驱映射，不再逐字段复验遥控数据。上游只负责输入合法性，本地私有枚举的非法状态此前无人处理
- Allman、同行 else、120 列和行尾空白扫描以及 git diff --check 通过；人工核对控制块花括号、可单行表达的语句、中文阶段注释、Doxygen 参数和返回值、集中参数说明

## 实物验证边界

本轮只完成软件实现与验证，未烧录、复位、写 RAM 或操作真实电机。需在具备上板条件时记录两轮实际转速、目标、模块状态及实际电流，测量下位回中位至停稳的时间，并检查零速附近抖动、反转、再加速和上位撤驱。主机合成反馈不代表机械惯量、负载、电源或 CAN 调度时延
