# 云台重新整定入口与验证

日期：2026-11-08

## 调试入口

本次只修改 User/task/motor_gimbal.c 和 motor_gimbal.h，添加七个 volatile 全局调试变量，不新增结构体、RTOS 对象或硬件资源。module/config.c 的既有工作树修改不属于本次变更，未修改或暂存

| 全局变量 | 用途 |
| --- | --- |
| gimbal_debug_override_enabled | 开启手动目标覆盖，默认 false |
| gimbal_debug_speed_loop_enabled | 速度内环开关，默认 false |
| gimbal_debug_angle_loop_enabled | 角度外环开关，默认 false |
| gimbal_debug_yaw_target_speed_rad_s | yaw 速度目标，默认 0 rad/s |
| gimbal_debug_pitch_target_speed_rad_s | pitch 速度目标，默认 0 rad/s |
| gimbal_debug_yaw_target_angle_deg | INS 累计 yaw 角度目标，默认 0 度 |
| gimbal_debug_pitch_target_angle_deg | INS pitch 角度目标，默认 0 度 |

开启覆盖后，速度开/角度关对应 SPEED；两个都开对应 ANGLE；速度关时始终 RELAX。仍要求遥控在线、右拨杆先中位再上位，回中或失联停机。关闭覆盖恢复现有遥控目标，不能把关闭覆盖当作停机操作

手动 ANGLE 模式只使用绝对姿态目标，速度变量保留但不作为前馈，前馈固定为零。手动覆盖期间不执行正弦生成器，并将其阶段重置到待机，退出覆盖后启用的正弦测试从当前姿态重新开始

调参前先关闭两环，设置目标与参数后再使能，避免多字段异步写入的过渡组合。角度目标可先设为 gimbal.yaw.angle_deg / gimbal.pitch.angle_deg，再逐步增减。起控、模式切换及反馈恢复的首周期由模块捕获当前姿态，之后采纳手动绝对目标

## PID 与观测路径

gimbal 是非 static 全局实例；其 config、yaw、pitch 是全局对象成员，并非局部变量，可通过完整路径观察或修改

| 控制器 | 可写参数前缀 | 当前源码 k / p / i / d | i_limit | out_limit |
| --- | --- | --- | --- | --- |
| yaw 速度内环 | gimbal.config.yaw.speed_pid | 1 / 0.06 / 0 / 0 | 0.5/0.177，约 2.82486 | 0.5 A |
| pitch 速度内环 | gimbal.config.pitch.speed_pid | 1 / 0.06 / 0.177 / 0 | 0.5/0.177，约 2.82486 | 0.5 A |
| yaw 角度外环 | gimbal.config.yaw.angle_pid | 1 / 15 / 0 / 0 | 1 rad·s | 约 33.5103 rad/s |
| pitch 角度外环 | gimbal.config.pitch.angle_pid | 1 / 15 / 0 / 0 | 1 rad·s | 约 33.5103 rad/s |

上述前缀后均有 k、p、i、d、i_limit、out_limit、d_cutoff_freq、range。四个 PID 的 d_cutoff_freq 和 range 当前都为 0。range 保持 0；d_cutoff_freq 的在线更新由 gimbal.c 的配置同步逻辑重新初始化滤波器，头文件中“只在初始化时生效”的旧说明与源码不同

角度输入虽然以度显示，PID 内部输入为弧度、输出 rad/s；速度 PID 输入 rad/s、输出 A。i_limit 限制积分累积量，不等于积分输出电流。修改 PID 参数由 Control 集中采纳并清积分，不修改私有 applied 参数

gimbal.config.yaw.enabled / pitch.enabled 是单轴总开关。max_speed_rad_s 两轴约 33.5103，max_current_a 两轴 0.5。pitch 编码器机械限制 [-30, 30] 度，与 INS 绝对俯仰角坐标不同

推荐观察两轴 target_speed_rad_s、speed_rad_s、target_angle_deg、angle_deg、output_current_a、mechanical_angle_deg、state，以及 gimbal.mode、config_valid、imu_state。速度模式 target_angle_deg 为 NaN 属于未使用角度目标的正常表达；速度反馈采用 INS 欧拉角速度，motor_speed_rad_s 仅用于编码器机械观测与保护

gimbal_command 是全局命令观测，每周期被任务重建，不能直接写它作为持久输入。PID 运行历史位于 gimbal.c 的 static gimbal_internal，是文件私有静态对象，不是对外全局调参入口

本表是当前工作树初始化值，未读取板上 RAM，不证明板上固件参数相同。当前 pitch.angle_pid.i 实际为 0.00，配置文件上方旧说明仍写 0.005，应以实际字段值为准

## 验证命令与结果

使用 aemb-build-cmake 技能，复用 Debug 预设

```powershell
$env:PSExecutionPolicyPreference = 'Bypass'
python .auto-embedded/scripts/check.py
python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug
git diff --check
```

ARCH、HW、SPEC 全 PASS。最初直接运行 check.py 时架构脚本受 PowerShell 签名策略阻止；仅通过本次进程环境及子进程设置执行策略后通过，未更改持久策略

Debug 配置及构建退出码 0，产物 build/Debug/Infantry_Robot.elf。从 compile_commands.json 复用正式 motor_gimbal.c 的 ARM 参数，移除 -o、-c、-fstack-usage，添加 -Wextra -Werror -fsyntax-only，正弦编译开关 1 和 0 两种均退出码 0、无诊断

正式 motor_gimbal.c、cmd.c、gimbal.c、config.c、pid.c、filter.c、user_math.c 在主机 GCC 下执行，模拟 CMSIS 邮箱、时钟与反馈，直接运行真实任务 while 循环。主机参数 -std=gnu11 -fgnu89-inline -ffunction-sections -fdata-sections -O0 -g -Wall -Wextra -Werror，链接 -Wl,--gc-sections -lm。18 组、245 条断言通过

| 用例 | 验证内容 |
| --- | --- |
| 0 | 关闭调试覆盖仍采用遥控增量目标 |
| 1 | 速度模式正负目标 0.4/-0.3 rad/s 及对应正负电流 |
| 2 | 角度串级绝对目标 12/-8 度、零前馈 |
| 3/4 | 两环关闭及只开角度环均停机 |
| 5/6 | 速度或角度 NaN 被模块入口拒绝，故障周期零输出 |
| 7/8 | 遥控离线及上电直接上位均不能使能 |
| 9 | 已运行后遥控失联，零输出 |
| 10 | yaw 单轴配置失能不影响 pitch |
| 11 | RAM 配置非法，config_valid=false 且两轴零输出 |
| 12/13 | 单轴反馈缺失或 IMU 无效不能绕过保护 |
| 14 | 正弦开关已开时手动绝对目标仍优先 |
| 15 | 关闭覆盖后正弦从当前姿态重新捕获中心 |
| 16 | 角度串级切速度模式 |
| 17 | 输出邮箱发布失败时模块复位待机并清遥控使能记忆 |

## 人工复核与校验自查

验证门：上述实际构建、ARM 检查、主机回归通过。硬件合规：无引脚、DMA、CAN ID、IRQ、时钟或 CubeMX 修改。质量：保持 task -> module 依赖、模块配置快照、私有 PID 指针及 DumpOutput 唯一输出闸门；main.c、HAL/RTOS 创建、ISR、临界区未改。新增变量由调试器写、任务读，volatile 保证读取可见性，不承诺多个变量原子同步

Allman、120 列、行尾空白检查通过，人工核对完整控制块、目的注释、Doxygen、集中参数说明与无机械断行

无新增判空、isfinite 或数值范围检查。新增一个提前 return 表示未选调试覆盖或上游命令为 RELAX 时保留已有命令，是命令路由条件，不是保护性兜底；使用 Cmd_ParseGimbal 已确认的运行许可，没有重新校验遥控字段或时间。开关组合只在任务入口映射一次，目标数值合法性仍由原 Gimbal_ResolveSetpoint 检查，未增加重复校验。复核删除了 mode 的重复赋值，重复防御检查删除项为无

临时测试源码、替身头、可执行程序位于 User/task/.gimbal_debug_check，保存本证据后全部清理，未删除正式测试资产

未烧录、未写板上 RAM、未执行真实电机整定。真实稳定性、参数优劣、失联时延及机械动作仍需上板验证
