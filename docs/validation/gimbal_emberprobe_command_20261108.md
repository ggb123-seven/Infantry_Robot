# 云台 EmberProbe 命令保持验证

日期：2026-11-08

## 已确认范围与实现

用户确认仅新增全局开关 gimbal_debug_enabled，复用 gimbal_command 手动选择模式与目标；保留遥控回中、失联和故障停机，手动调试期间屏蔽正弦测试

源码修改仅在 User/task/motor_gimbal.c 和 motor_gimbal.h。没有新增结构体类型或字段，没有新增 PID 参数副本、RTOS 对象、外设资源。User/module/config.c 与硬件 spec 的原有工作树修改未纳入本次提交

- gimbal_debug_enabled 默认 false，正常遥控行为保持原样
- 每周期读取一次开关，遥控解析先输出到已有 Gimbal_Command_t 类型的局部 parsed_command
- 调试关闭时把 parsed_command 写入 gimbal_command；开启且遥控允许时不再改写 gimbal_command，直接采用 EmberProbe 写入的模式、来源、角度和速度
- 遥控不允许时将 gimbal_command.mode 清为 RELAX，并清零两轴 rate_rad_s。恢复运行许可后不会自动重写运行模式，须由调试器重新写 mode
- 手动调试屏蔽正弦生成器，并把原正弦阶段恢复为 IDLE；关闭调试后恢复遥控，启用的正弦测试重新捕获当前中心
- 控制前从 volatile 视图读取完整命令到本周期局部快照，再交给 Gimbal_Control 统一检查模式与目标合法性。快照不等于多字段写入原子化，应先停机再修改多个字段
- Gimbal_Control 的配置快照、PID 持久参数引用、状态机及 DumpOutput 唯一输出闸门不变

## 使用步骤

1. 更新固件和对应 ELF，右拨杆回中；将 gimbal_debug_enabled 写为 true
2. 保持 gimbal_command.mode=0，设置两轴目标、单轴 enabled 和 gimbal.config 下的 PID 参数
3. 右拨杆拨上，确认 gimbal.imu_state=2、gimbal.config_valid=true；设置最终速度目标后，最后写入 mode=1 或 mode=2
4. mode=0 或右拨杆回中均可停机。右拨杆中位、下位、非法档位、失联、锁存故障或发布失败会清除运行模式
5. gimbal_debug_enabled=false 恢复正常遥控控制，不能将关闭调试开关当作停机操作

| 参数路径 | 语义 |
| --- | --- |
| gimbal_command.mode | 0=RELAX，1=SPEED，2=ANGLE，两轴共用 |
| gimbal_command.yaw.rate_rad_s / pitch.rate_rad_s | SPEED 为目标速度；ANGLE 为速度前馈，单位 rad/s |
| gimbal_command.yaw.source / pitch.source | 0=增量目标，1=INS 坐标下的绝对角度，固定角度调试用 1 |
| gimbal_command.yaw.angle_deg / pitch.angle_deg | 绝对目标角，单位度，分别为 INS 累计 yaw 和 INS pitch |
| gimbal.config.yaw.enabled / pitch.enabled | 单轴总使能，关闭未调试轴可隔离该轴 |
| gimbal.config.yaw.speed_pid / pitch.speed_pid | 速度内环参数，输入 rad/s，输出 A |
| gimbal.config.yaw.angle_pid / pitch.angle_pid | 角度外环参数，内部输入 rad，输出 rad/s |

固定角度调试时两轴 source 均设 1，rate_rad_s 均设 0，角度先填写对应 gimbal.yaw.angle_deg 和 gimbal.pitch.angle_deg，再设置 mode=2。起控和切模式首周期模块捕获当前姿态，后续采用绝对目标；pitch 机械限制沿用既有规则

所有调参入口均为全局对象或其成员。gimbal_command 不改类型及声明，控制任务通过 volatile 视图强制读取外部修改。未新增独立手动目标变量，也未改变正弦开关默认值

## 软件验证证据

使用 aemb-build-cmake 技能，复用 Debug 预设与已有工具链

```powershell
$env:PSExecutionPolicyPreference = 'Bypass'
python .auto-embedded/scripts/check.py
python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug
git diff --check
```

ARCH/HW/SPEC 全 PASS，执行策略仅用于检查进程及其子进程，没有修改持久系统策略。Debug 配置及构建退出码为 0，产物 F:/RM/Infantry_Robot/build/Debug/Infantry_Robot.elf，使用 Ninja

从 build/Debug/compile_commands.json 读取 motor_gimbal.c 正式 ARM 命令，去掉 -o、-c、-fstack-usage，添加 -Wextra -Werror -fsyntax-only；正弦宏默认开启与 MOTOR_GIMBAL_ENABLE_SINE_TEST=0 两种配置均退出码 0，无诊断

临时主机回归直接包含正式 motor_gimbal.c，链接正式 cmd.c、gimbal.c、config.c、pid.c、filter.c、user_math.c，运行真实任务 while 循环。只以时钟、CMSIS 邮箱、BSP_CAN_t 与 FreeRTOS 节拍替身隔离硬件，输入合成反馈

主机 GCC 参数为 -std=gnu11 -fgnu89-inline -ffunction-sections -fdata-sections -O0 -g -Wall -Wextra -Werror，链接 -Wl,--gc-sections -lm。正弦宏开启和关闭分别编译执行，均通过相同 29 组、756 条断言，共执行 1512 条断言

| 用例 | 验证结果 |
| --- | --- |
| 正常遥控 | 调试默认关闭，手写命令仍被正常遥控覆盖 |
| 手动速度保持 | 正负速度 0.4/-0.3 rad/s 连续多周期保持，原 source 和 angle 字段也未被摇杆覆盖 |
| 手动角度保持 | ABSOLUTE 目标 12/-8 度连续保持，实际角度目标及输出方向符合命令 |
| 手动待机 | mode=0 不被遥控上位自动改为 ANGLE |
| 回中、下位、失联 | 同周期 mode 归零、最终输出零电流，恢复后仍须写入运行模式 |
| 启动上位、启动离线 | 调试器手写运行命令也不能越过中位确认或在线许可 |
| 非法命令 | NaN 速度、NaN 角度、非法模式、非法来源均由既有模块拒绝并零输出 |
| 反馈与配置故障 | yaw 无反馈只隔离 yaw，IMU 未就绪两轴不运行，RAM 配置非法则 config_valid=false |
| 单轴关闭 | yaw.enabled=false 时 yaw 零输出且 pitch 继续运行 |
| 正弦隔离 | 正弦开关已开时手动目标仍保持不变 |
| 来源切换 | debug=false 恢复遥控增量命令，SPEED/ANGLE 可由 mode 双向切换 |
| 发布失败 | 邮箱写入失败后两轴 RELAX，命令 mode 清零并重置遥控使能记忆 |
| 遥控边界 | 源时间超过 100 ms 或邮箱读取错误立即停机，只有新消息缺失但缓存仍新鲜时允许保持 |
| 在线 PID | RAM 中 yaw.speed_pid.p 由 0.06 改为 0.12，0.4 rad/s 目标下输出指令为 0.048 A，const 默认配置仍为 0.06 |
| 手动前馈 | ANGLE 下显式设置的 0.2/-0.1 rad/s 前馈被正常采用 |
| 再使能 | 遥控回中后恢复上位仍保持 mode=0，重新写入 mode 才恢复运行 |

Allman、120 列、行尾空白及 git diff --check 通过，人工复核控制块花括号、机械断行、中文阶段目的注释、Doxygen 参数与返回值、头文件集中说明

## 三层复核与防御检查自查

1. 验证门：上述真实构建、ARM 严格检查、正式任务主机回归均已执行并通过，合成反馈不代表真实电机响应
2. 硬件合规：机械 HW 门禁通过，没有修改 CAN ID、引脚、DMA、时钟、中断或 CubeMX
3. 代码质量：保持 task -> module 依赖；main.c、HAL/RTOS 对象创建、ISR、临界区和持久化未变。只新增一个 volatile 布尔全局变量，原命令以局部快照交给模块，不向 PID 提供栈上参数指针

新增判空、isfinite、数值范围检查、保护性 return 均为无，重复防御检查删除项为无。新增条件清单如下

| 新增条件或安全赋值 | 实际场景与边界 | 上游是否已检查 |
| --- | --- | --- |
| debug_enabled 选择来源 | 用户手动选择遥控或 EmberProbe，命令路由业务条件 | 每周期只读取一次全局开关，两处消费同一个快照，分别负责遥控写入路由和正弦隔离 |
| parsed_command.mode==RELAX | 调试请求遇到遥控停机、失联或故障时，必须撤销实际运行请求 | 复用 Cmd_ParseGimbal 已校验的结果，不重新检查遥控字段、时间或故障条件 |
| mode=RELAX 且两轴 rate_rad_s=0 | 撤销调试运行模式及速度请求，避免恢复遥控后自动恢复旧运动 | 上游只清除了局部 parsed_command；全局手动命令此前未清除，不属于重复赋值 |

本次复用原命令层和模块的显式状态机，没有增加跨周期状态或新的三种互斥业务模式，不新增重复状态机

## 实物与清理边界

未烧录、未写目标板 RAM，未实测真实失联停机延迟、运动方向、稳定性或栈高水位。原任务栈为 1024 字节，编译器 .su 给出 Task_motor_gimbal 本函数 264 字节、MotorGimbal_ApplyDR16Input 本函数 136 字节；这是单函数静态帧，不能代替完整调用链或实机栈余量测量

临时测试目录为 User/task/.gimbal_manual_check，保存证据后删除其测试源码、替身头、可执行文件与临时日志，正式测试资产不变
