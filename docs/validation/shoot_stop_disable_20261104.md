# 摩擦轮停稳后关闭速度环验证

日期：2026-11-04

## 当前行为与实现范围

本次替代 shoot_zero_speed_hold_20261104.md 中的持续零速保持行为。用户要求先以零目标主动刹停，再关闭速度环，避免停稳后继续输出维持电流

- 下位运行：双轮目标 8000 rpm，复用原有速度 PID 和每轮 3 A 电流限幅
- 下位回中位：先保持 enabled=true、双轮目标 0 rpm，进行速度闭环制动
- 停稳确认：两轮转子速度绝对值均不超过 1 rpm，各自连续新反馈的源时间跨度均至少 20 ms
- 达到确认条件后：下一控制周期提交 enabled=false，Shoot_Control 清除 PID 历史，Shoot_DumpOutput 输出两轮 0 A 指令；名义 500 Hz 下增加一个 2 ms 周期
- 持续中位：保持已撤驱，即使外力带转或偶发缺少命令也不重新开启闭环；下位非零目标重新使能
- 上位或遥控失效：沿用原映射立即失能；上电即在中位仍不使能
- 制动期间任一轮超出低速范围、缺新反馈、反馈被模块拒绝、控制未进入 SHOOT_RUNNING 或缺命令，重新确认停稳，不用缓存零速帧累计时间
- 配置故障不被停稳检测伪装成正常完成，仍按原 Shoot 模块规则失能解除

±1 rpm 和 20 ms 是本轮实现选取的初始停稳判据，尚无实物测量依据，集中在 motor_shoot.c 顶部说明。这里的“零速”是容差判定，不要求浮点值精确等于零。关闭后不再主动抵抗外力，不能同时保证电机永远维持 0 rpm

源码改动限定于 User/task：motor_shoot.c 增加任务独占的停稳命令状态及两个源时间起点；motor_trigger.c 将原 HOLDING 阶段命名改为 BRAKING，继续只发布零速制动请求；motor_shoot.h 和 user_task.h 同步集中说明。没有新增或修改结构体，没有新增 RTOS 对象，没有改 config.c、模块、PID、设备驱动、BSP、CubeMX 配置

任务的停稳状态仅编排提交给既有模块的命令；模块继续独占 PID、配置快照、反馈有效性、运行状态和最终电流闸门。原始零目标请求与实际提交的失能命令区分：停止后 shoot_command.enabled 为 false，下一周期邮箱再次收到零目标不清除停止记忆。只在明确失能或非零目标命令到来时退出已撤驱阶段

## 命令与结果

使用 aemb-check 与 aemb-build-cmake 技能，复用 Debug 预设和既有构建目录

```powershell
$env:PSExecutionPolicyPreference = 'Bypass'
python .auto-embedded/scripts/check.py
python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug
git diff --check
```

门禁 ARCH/HW/SPEC 全 PASS。执行策略仅作用于本次检查进程及子进程，未修改持久系统设置。CMake 配置与构建退出码 0，固件为 build/Debug/Infantry_Robot.elf，使用 Ninja 和项目原有 ARM 工具链

从 build/Debug/compile_commands.json 读取 motor_shoot.c、motor_trigger.c 的正式 ARM 编译参数，去掉 -o、-c 和 -fstack-usage，添加 -Wextra -Werror -fsyntax-only：两份文件退出码均为 0，无诊断

## 主机回归

直接包含正式 motor_shoot.c、motor_trigger.c，链接正式 cmd.c、feeder.c、shoot.c、config.c、pid.c、filter.c、user_math.c。模拟时钟和 CMSIS 邮箱，使用正式反馈与任务运行时类型；仅以 BSP_CAN_t 替身隔离 HAL，以 FreeRTOS 替身提供 1000 Hz 节拍。执行正式摩擦轮任务 while 循环，在 osDelayUntil 替身保存每周期状态并结束用例

主机 GCC 参数：-std=gnu11 -fgnu89-inline -ffunction-sections -fdata-sections -O0 -g -Wall -Wextra -Werror，链接参数 -Wl,--gc-sections -lm。编译退出码 0，测试执行退出码 0

```text
RUN 1 brake-settle-20ms-disable-next-cycle
RUN 2 residual-integral-cleared-on-disable
RUN 3 plus-minus-one-rpm-inclusive
RUN 4 one-wheel-not-stopped-keeps-braking
RUN 5 speed-excursion-restarts-confirmation
RUN 6 zero-crossing-never-counts-as-settled
RUN 7 cached-zero-feedback-cannot-complete
RUN 8 missing-new-frame-restarts-confirmation
RUN 9 duplicate-source-time-not-settled
RUN 10 offline-feedback-recovery-new-confirmation
RUN 11 each-wheel-source-time-must-cover-20ms
RUN 12 stopped-remains-disabled-after-disturbance-and-empty-command
RUN 13 nonzero-command-rearms-and-reaccelerates
RUN 14 explicit-disable-immediate-zero-output
RUN 15 missing-command-during-braking-restarts-confirmation
RUN 16 fault-not-cleared-by-stop-detector
RUN 17 low-32-bit-feedback-timestamp-wrap
RUN 18 invalid-stop-state-same-cycle-disables
RUN 19 output-publish-failure-resets-controller
ALL PASS: 20 groups
```

第 20 组使用实际 Cmd_ParseFeeder 和 MotorTrigger_ParseShootCommand，断言上电中位失能、下位双轮 8000 rpm、回中位双轮零速请求且拨弹 RELAX、上位失能

首组高速反馈为双轮 6000 rpm，目标由 8000 改为 0 后两轮实际导出指令均为 -3 A。第 2 周期首次收到双轮零速，第 11 周期未足 20 ms 不撤驱，第 12 周期确认满 20 ms，第 13 周期模块进入 SHOOT_RELAX，实际发布电流均为 0 A

残留积分测试只在主机实例 RAM 副本设置 Ki=0.1 并注入正负非零积分，撤驱前仍存在正负非零电流指令，撤驱后输出 0 A，两个 PID 的 i 和 last.out 均为零；只读 shoot_config.speed_pid.i 始终为零。没有写板上 RAM 或修改 config.c

较慢反馈源时间测试使第二轮时间每周期只推进 1 ms，第一轮推进 2 ms，必须等第二轮满 20 ms 才确认。缺帧、重复源时间、过期反馈、过零反转与一轮未停均未被误判完成。停稳后注入 ±200 rpm 扰动与空命令，输出始终为零；重新给 8000 rpm 命令恢复 +3 A 限幅输出

## 三层复核

1. 验证门：20 组真实生产代码主机回归、两份 C 文件 ARM 严格检查、Debug 构建及格式检查通过；主机反馈是合成数据，不代表真实惯量、电流或制动时长
2. 硬件合规：没有改变 CAN ID、引脚、时钟、DMA、中断及硬件资源，HW 门禁通过
3. 代码质量：保持 task -> module -> component/device 依赖；main.c、RTOS/HAL 对象创建、ISR 均未修改，不引入临界区或跨任务裸全局访问。新状态和源时间数组由摩擦轮任务独占，不需要 volatile。已验证非法新状态同周期提交失能，实际电流仍只从 Shoot_DumpOutput 导出

Allman、120 列、行尾空白和 git diff --check 检查通过；人工检查控制块花括号、可单行语句、中文处理阶段注释、Doxygen 参数返回值及集中参数说明

## 校验与状态条件自查

本轮生产代码未新增判空、isfinite、参数有效性范围检查或保护性 return，未新增重复配置校验；因此无此类重复检查删除项。新增数值比较均为停稳业务条件，具体如下

| 新增判断 | 实际场景与分类 | 上游是否已判断同一条件 |
| --- | --- | --- |
| 已收到新命令、enabled 和两轮目标均为零 | 区分零速制动、明确撤驱、重新加速和空邮箱，命令操作语义 | 未判断停稳后的命令处理，沿用原邮箱返回结果 |
| 模块状态为 SHOOT_RUNNING | 只在本周期已通过模块配置、反馈和控制校验后采纳停稳结果 | 复用模块结果，不重复各字段校验 |
| 本周期两轮均有新反馈 | 防止 100 ms 有效缓存被反复当作停稳证据，停稳取样条件 | 模块允许缓存闭环，不保证本周期新样本；复用两次邮箱读取结果 |
| 两轮转速绝对值不超过 1 rpm | 区分一轮仍在转动、单点过零与低速停稳窗口，业务范围条件 | 无，模块只负责速度控制及反馈合法性 |
| 两路源时间分别覆盖至少 20000 us | 防止短暂过零或一轮源时间较慢时提前撤驱，业务持续时间 | 无，模块只保证新帧源时间推进及帧龄，不判断停稳持续时间 |
| 非法停稳枚举的 default | 状态损坏时当前周期失能并转入停止，状态完整性安全分支 | 无，新私有枚举仅由本任务解释 |

原始源时间回绕与单调性由 Shoot_UpdateFeedback 检查，实测 period_s 和配置由 Shoot_Control 检查；本次只使用这些结果，不增加下游重复校验

## 清理与实物边界

临时测试目录为 User/task/.shoot_stop_check，测试源码、替身头和可执行程序在保存证据后全部清理，正式测试资产未删除

本轮未烧录、复位或操作真实电机。停稳判据、真实停机耗时及电流衰减仍需上板验证。软件保证的是最终电流指令归零和 PID 历史复位，不宣称物理电流瞬间为零；停稳撤驱后也不再保证受到外力时保持零速
