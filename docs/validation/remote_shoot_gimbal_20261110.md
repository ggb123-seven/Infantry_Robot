# 遥控射击解耦与云台闭环调试接口

日期：2026-11-10

## 确认范围

用户确认修改 User/module/cmd.c、cmd.h 和 User/task 对应编排及接口，不新增结构体、不修改 config.c。既有 gimbal.c、gimbal.h、config.c、motor_shoot.c、硬件 spec 和数据文件的工作树差异不属于本次修改，也不纳入本次提交。构建与主机验证使用当前工作树，包含这些已有修改

参考工程 F:/RM/未命名文件夹/User/component/cmd.c 的 CMD_RcLogic 在右拨杆三个正常档位均选择 GIMBAL_MODE_ABSOLUTE，左拨杆只选择底盘模式；本次参考其云台在线使能方式，射击按用户指定的新映射实现

## 当前行为

| 右拨杆 | 拨弹 | 摩擦轮 | 云台遥控默认模式 |
| --- | --- | --- | --- |
| 上位 | 单发逐齿串级 | 双轮 8000 rpm | ANGLE |
| 中位 | 停机，可解除拨弹故障 | 从运行返回时零速制动，再按现有时长撤驱；上电中位不使能 | ANGLE |
| 下位 | 连发速度环 | 双轮 8000 rpm | ANGLE |
| 遥控失效或超时 | 离线撤驱 | 撤驱 | RELAX |

射击不再读取左拨杆，也不要求右拨杆下位后再切左拨杆的先后顺序。左拨杆底盘映射保持原样

启动和单连发互切仍先等待右摇杆回中。单发仍以右摇杆向上端部产生持续请求，松杆后完成当前齿，长推在到位后按原节拍逐发重复；连发保留右摇杆双向归一化速度比例。拨杆选择模式本身不直接触发一发

两种射击档都请求摩擦轮运行，不以拨弹是否已经回中或正在进给作为摩擦轮使能条件。摩擦轮任务保留工作树原有 1000000 us 制动阈值，原文件中仍有历史 3 秒注释，本次没有修改该文件

遥控在线后自动请求两轴 ANGLE，不再要求右拨杆中位确认再拨上。模块仍要求轴配置使能、有效电机和惯性反馈、有效配置和控制周期；实际零电流输出仍由 Gimbal_DumpOutput 决定。首次上线及 RELAX/SPEED 切回 ANGLE 时模块捕获当前姿态

锁存故障时命令层仍提交 RELAX，模块按既有规则解除锁存，条件恢复后遥控自动重新请求运行。yaw 悬空带转 TRIPPED 继续由模块锁存，需要关闭遥控速度环或断开遥控使命令进入 RELAX 后解除

## EmberProbe 接口

以下接口位于 User/task/motor_gimbal.c，声明和集中说明位于 motor_gimbal.h。两个新开关都是 volatile bool 标量，复位为 true，仅在 gimbal_debug_enabled=false 的正常遥控来源生效

| 速度环开关 gimbal_remote_speed_loop_enabled | 角度环开关 gimbal_remote_angle_loop_enabled | 行为 |
| --- | --- | --- |
| true | true | 两轴 ANGLE，角度外环与速度内环串级 |
| true | false | 两轴 SPEED，右摇杆直接请求角速度，单位 rad/s |
| false | 任意 | 两轴 RELAX，输出零电流 |

单轴隔离复用 gimbal.config.yaw.enabled 和 gimbal.config.pitch.enabled，模块仍逐周期复制并集中校验 RAM 配置，任务不覆盖这些字段。PID 参数入口继续使用 config 中各轴 speed_pid 和 angle_pid

每周期各读取一次两个开关，根据已解析的云台命令选择模式，不重复判断 DR16 帧龄、配置或中间计算结果。开关独立写入，不构成多字段原子事务；需要改变多个开关时可先关闭速度环，设置角度环后再开启速度环

gimbal_debug_enabled=true 仍沿用既有独立手动来源，跳过遥控读取和解析，新开关不会影响该来源；手动停机使用 gimbal_command.mode=0。关闭手动调试会恢复正常遥控，不能将关闭调试视为停机

## 验证证据

```powershell
python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug
$env:PSExecutionPolicyPreference = 'Bypass'
python .auto-embedded/scripts/check.py
python User/task/.remote_mapping_host/run.py
git diff --check -- User/module/cmd.c User/module/cmd.h User/task/motor_gimbal.c User/task/motor_gimbal.h User/task/motor_trigger.c User/task/motor_trigger.h User/task/user_task.h
```

- CMake Debug 配置和 Ninja 构建成功，产物 build/Debug/Infantry_Robot.elf
- 最初 ARCH 检查被 PowerShell 未签名脚本执行策略阻止；仅对检查进程及其子进程设置执行策略环境变量后，ARCH、HW、SPEC 全部 PASS，没有更改持久系统策略
- 临时主机测试包含正式 motor_gimbal.c、motor_trigger.c，链接正式 cmd.c、gimbal.c、feeder.c、config.c、pid.c、filter.c、user_math.c；仅为 BSP CAN 类型、时间、FreeRTOS 频率和 CMSIS 邮箱提供替身，直接调用任务私有命令编排函数及正式模块
- 主机 GCC 使用 gnu11、fgnu89-inline、Wall、Wextra、Werror；MOTOR_GIMBAL_ENABLE_SINE_TEST=0 和 1 分别通过 29 组、193 条断言，共 386 条断言
- 覆盖左右拨杆九种组合、在线首次双轴起控、左拨杆切换不影响射击、正反连发比例、切档回中、单发真实模块目标推进、中位拨弹零输出、上下档摩擦轮请求、中位制动和失联撤驱
- 覆盖闭环开关四种组合、速度关闭零输出、恢复 ANGLE 捕获姿态、单轴关闭、在线 RAM PID 副本、非法配置零输出、真实反馈超时、100000/100001 us 遥控边界、时间回退、非法周期、故障、邮箱错误及手动调试独立来源
- 从 compile_commands.json 提取正式 ARM 编译参数，去除输出、编译和栈报告参数，追加 Wextra、Werror、fsyntax-only；cmd.c、motor_gimbal.c、motor_trigger.c 全部通过
- 七份修改源码的 Allman 花括号、120 列、行尾空白自动扫描通过，人工检查处理阶段中文目的注释、Doxygen 参数返回值、机械断行和集中接口说明

测试验证正式任务私有解析函数和正式模块，不执行真实 RTOS 调度或 CAN 外设，也不证明实际弹丸发射、真实运动方向、制动效果或栈高水位。未烧录、未写目标 RAM、未停核或复位

## 三层复核与防御检查自查

1. 验证门：真实 Debug 构建、ARM 严格检查、两种正弦编译配置主机回归及格式检查通过，上板验收待执行
2. 硬件合规：HW 门禁通过，CAN ID、引脚、DMA、时钟、中断、RTOS 对象和 CubeMX 配置均没有本次修改
3. 代码质量：命令映射仍在 module/cmd，Task 只编排模式和邮箱；依赖方向不变，main.c、ISR、临界区和持久化不变；新增两个 volatile 标量由任务读取，没有新增 PID 参数指针或结构体字段

| 检查或安全路径 | 信任边界与实际场景 | 上游是否检查 |
| --- | --- | --- |
| Cmd_ParseGimbal 状态 switch 的 default 复位并返回 RELAX | 非法状态安全路径，枚举受破坏时不生成运行命令 | 本函数此前未检查上下文枚举 |
| 拨弹状态 switch 的 default 离线返回 | 复用原有非法状态安全路径，删除旧状态后统一识别非法上下文 | 只在状态转换入口检查一次 |
| 遥控速度和角度开关选择 | 用户调试模式路由，属于业务使能条件，不是参数范围校验 | 复用已解析的 ANGLE/RELAX，不再次判断遥控时间和故障 |

新增判空、isfinite、数值范围检查为无。原 DR16 有效性、帧龄和控制周期边界保留；原内部通道归一化保留。原先中位确认阶段与左拨杆顺序条件按需求删除；摩擦轮不再重复读取右拨杆档位，只消费命令层已确认的 RUN/RELAX/OFFLINE

实现时已删除第二个拨弹输出 switch 中重复的非法状态复位和保护性 return，非法上下文在前一个状态转换 switch 统一处理，第二个 switch 仅选择请求输出，默认保留零进给。没有新增重复的配置、反馈或浮点检查

记录完成后尝试清理本任务临时创建的 User/task/.remote_mapping_host；自动审批分别拒绝递归目录清理和逐文件清理，原因仅返回 blocked by policy。该临时测试目录暂时保留且不纳入提交；既有 .chassis_ff_host、.yaw_friction_host 及正式测试资产保持原样
