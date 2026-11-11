# pitch PID 参数移植与整圈环绕验证

日期：2026-11-11

## 授权与实现

用户明确授权修改 User/module/config.c 及必要的 User/module/gimbal.c 校验，将提供的两组参数用于 GM6020 pitch；确认速度环输出单位为 A，最终选择角度环启用 2π 环绕

| 参数 | pitch 角度环 | pitch 速度环 |
| --- | --- | --- |
| k | 2 | 0.25 |
| p | 5 | 1 |
| i | 2.5 | 0 |
| d | 0 | 0 |
| i_limit | 0 | 1 |
| out_limit | 10 rad/s | 1 A |
| d_cutoff_freq | -1 | -1 |
| range | M_2PI | -1 |

两环参数与用户片段一致，沿用现有 PID 算法。i_limit=0 阻止非零积分状态累计，但 PID_Calc 仍把本周期候选积分加入输出，因此角度环零历史时的未限幅输出为误差乘以 (10+5×dt)，不能当作常规持续积分

配置校验仅允许 pitch 角度环的正周期为 M_2PI；yaw 角度环和两轴速度环保持关闭环绕。有限的零值和负值沿用底层 CircleError 的关闭语义，兼容原片段的速度环 range=-1。参数变更检测新增 range 比较，使 RAM 在线切换时同步持久 PID 参数并沿用积分重置

保留 RAM config、Control 入口周期快照、持久参数引用和 DumpOutput 输出闸门。对照任务开始时备份，yaw、其他模块参数、pitch 机械零点和行程、最终电流上限、超时和摩擦补偿均未改变；集中注释同步了工作树既有 pitch 行程。未新增结构体、状态机或硬件资源

工作树已有其他用户改动；本次提交按差异块隔离 PID 参数、必要校验及记录，不纳入既有 yaw、拨弹、机械行程或其他源码改动。整机 ELF 由当前工作树构建，不等同于只包含本次提交的独立构建

## 验证证据

- python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug：配置与构建通过，生成器 Ninja，产物 build/Debug/Infantry_Robot.elf
- 从 build/Debug/compile_commands.json 提取正式 ARM 命令，去除 -o、-c、-fstack-usage，追加 -Wextra -Werror -fsyntax-only：config.c、gimbal.c 均通过
- 主机 GCC 使用 -std=gnu11 -fgnu89-inline -O0 -Wall -Wextra -Werror，直接编译正式 gimbal.c、pid.c、filter.c、user_math.c；默认云台配置从实际 config.c 提取，反馈类型复用正式 MOTOR_Feedback_t，只隔离未调用的 CAN/HAL 头依赖
- 主机回归 386696 条断言通过：默认参数和初始化；PID 持久指针；range=0/M_2PI 在线双向同步；NaN、正负无穷、其他正周期的初始化拒绝与运行期拒绝；非法快照不覆盖已应用参数且 DumpOutput 两轴归零；yaw 和速度环不接受 M_2PI；有限负值关闭环绕
- 角度网格覆盖目标与反馈各 -179° 至 +179°，与 atan2(sin(差值),cos(差值)) 交叉核对误差及限幅输出；恰好半圈时核对绝对误差为 π 和输出饱和幅值，不将等长方向差异误判为错误。另验证跨 ±179° 的相反纠偏方向，以及速度环 ±1 A 饱和
- Python 对照任务开始前备份，确认非云台配置、yaw 配置、pitch PID 之前的机械配置及云台超时和摩擦配置均未改变
- python .auto-embedded/scripts/check.py：ARCH、HW、SPEC 全 PASS；PSExecutionPolicyPreference=Bypass 仅作用于当前检查进程及子进程
- Allman、120 列、行尾空白及 git diff --check 通过；人工检查新增 Doxygen、集中说明及配置流向，无新增业务分支状态流程

ELF SHA256：f9b98d8940d3214f0b89c5d61d4d48f2bc103357ea7a043a150750e0f892646e

ELF size：text=130216 B，data=208 B，bss=50200 B，静态 RAM=50408 B

## 栈与三层复核

motor_gimbal 现有静态任务栈为 3072 B；当前 ARM .su 报告：Task_motor_gimbal=368 B、Gimbal_Init=216 B、Gimbal_Control=288 B、Gimbal_RunAxis=112 B、PID_Calc=72 B、Gimbal_IsPidConfigValid=24 B、Gimbal_IsAxisConfigValid=16 B，相关函数开销与修改前报告一致

1. 验证门：上述构建、严格语法和真实算法主机回归通过；未烧录、未写目标 RAM、未运行真实电机，尚无新固件运行时栈高水位，动态响应和栈余量未完成实测
2. 硬件合规：HW 门禁通过，未修改 CAN、引脚、DMA、中断、时钟或 CubeMX 配置，原有机械限位继续生效
3. 代码质量：实现位于 module 层，未改 main.c、任务编排、RTOS/HAL 对象创建、ISR 共享状态或临界区，保留既有输出安全状态流程

## 校验自查

- 新增 isfinite(pid->range)：属于模块初始化与 Control 在线配置的信任边界，实际触发场景为 EmberProbe 写入 NaN 或正负无穷；RAM 副本上游没有同条件校验
- 替换原 range==0 约束：仅在同一配置入口检查有限非正值，或具有许可的 pitch 角度环 M_2PI；实际场景为合法移植值和调试器误写其他正周期。该条件没有在下游重复检查
- range 的变更比较只负责在线参数同步，沿用 Control 入口执行位置，不是额外的防御检查
- 未新增判空或保护性 return；未删除其他边界检查，没有新增下游重复校验

## 临时文件

本任务临时主机源码、默认配置摘录、头依赖隔离文件和可执行文件位于 User/task/.pitch_pid_port_host，记录结果后删除；任务开始时的临时备份亦在结束时删除，不处理正式测试资产
