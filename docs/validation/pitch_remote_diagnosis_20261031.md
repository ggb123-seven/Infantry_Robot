# pitch 遥控无响应板上只读定位

## 现场与结论

用户反馈 pitch 无法通过遥控器控制，并确认方向正确。本次按此信息排查使能、命令与机械限位，不改方向和 PID

板上直接观测到右拨杆为上位，但云台命令上下文仍在 WAIT_NEUTRAL，因此当前输出为 RELAX；这足以解释本次采样期间 pitch 无电流。不能仅凭采样断言用户从未拨过中位，上电、遥控失联或故障也会重新要求中位确认

另有独立问题：当前编码器换算的 pitch 机械角约 +125.9 度，而配置启用的行程为 [-60, +60] 度。该坐标超出上限，重新使能后限位计算会约束目标和向外电流，并可能产生向内纠偏请求。零点是否失准需在实物水平姿态下核对，不能直接用本次编码器读数覆盖配置

## 板上证据

使用 mcu-variables read.js，只读复用 active-sampling，未暂停、复位、烧录或写入变量

首次读取返回 firmwareConsistency.status=match，表示当前 ELF 与 EmberProbe 最近烧录记录匹配，不等同于本次重新读取 Flash 校验

- ELF：build/Debug/Infantry_Robot.elf
- SHA-256：75d6cf0424987f9ccf10582bd5713a1b7c2540534c5d6aef68244cf2f71539e3
- 记录的最近烧录时刻：2026-10-31T11:46:41.820Z
- 首帧时间戳：1793447370130 ms

| 字段 | 读数及含义 |
| --- | --- |
| motor_gimbal.c::remote_cache_valid | 1，有有效遥控缓存 |
| motor_gimbal.c::remote_cache.data.sw_r | 1，右拨杆上位 |
| motor_gimbal.c::remote_command.state | 0，WAIT_NEUTRAL |
| gimbal.mode | 0，RELAX |
| gimbal.yaw.state / gimbal.pitch.state | 均为 0，RELAX |
| gimbal.imu_state | 2，READY |
| gimbal.config_valid | 1 |
| gimbal.config.pitch.enabled | 1 |
| gimbal_command.pitch.rate_rad_s | 0 |
| gimbal.pitch.output_current_a | 0 A |
| gimbal.pitch.target_angle_deg | NaN，待机状态的既有表示 |
| gimbal.pitch.encoder_deg | 243.98439025878906 度 |
| gimbal.config.pitch.zero_encoder_deg | 118.08016204833984 度 |
| gimbal.pitch.mechanical_angle_deg | 首次 125.90423583984375 度，趋势范围 125.86026000976562 至 125.90423583984375 度 |
| gimbal.pitch.angle_deg | 首次 4.527642250061035 度，INS 姿态角，不是编码器机械角 |
| gimbal.config.pitch.mechanical_limit_enabled | 1 |
| gimbal.config.pitch.min_angle_deg / max_angle_deg | -60 / +60 度 |

随后以 200 ms 间隔采集 10 帧：sw_r、remote_command.state、gimbal.mode、两轴状态、IMU 状态及电流均保持上述值。ch_r_y 在此窗口始终为 0，没有取得用户推动摇杆期间的样本，不据此宣称通道损坏或映射错误。运行中多字段读取不是原子快照，稳定趋势用于支持本窗口观察

## 源码对应

- User/module/cmd.c:176：WAIT_NEUTRAL 只有检测到右拨杆 MIDDLE 才进入 READY；保持 UP 不会起控
- User/module/cmd.c:205：非 ACTIVE 清零两轴命令速度并保持 RELAX
- User/module/cmd.c:220：pitch 输入为右摇杆上下 ch_r_y
- User/module/gimbal.c:427：按编码器机械角与上下限共同约束姿态目标
- User/module/gimbal.c:453：机械范围约束目标速度，越界时保留向内运动
- User/module/gimbal.c:470：上限处阻断正向电流，下限处阻断负向电流，并非越界即禁止全部方向
- User/module/gimbal.c:294：非 RUNNING 的轴输出零电流

## 验证与范围

- python .auto-embedded/scripts/check.py：ARCH、HW、SPEC 全部 PASS
- CMake Debug 配置与增量构建通过，未烧录
- 本次获得真实目标变量与 10 帧趋势，未运行真实起控动作验收，尚未验证中位确认后的响应
- 没有修改生产源码、结构体、config.c、.ioc、main、RTOS/HAL 对象、ISR 共享数据或临界区；分层依赖、配置副本、周期快照与唯一输出闸门保持不变
- 新增判空、isfinite、范围检查、保护性 return 和删除检查项均为零；没有临时测试源码或测试程序

## 下一步

### 用户补充更换新电机后的复核

用户补充 pitch 完全无响应，可能与更换新电机有关。再次只读采集 10 帧、间隔 200 ms，pitch.feedback_time_us 从 222175263 增加至 224193260 微秒，证明配置的 pitch 通道持续取得反馈，不仅是注册标志有效；CANDevices 当前 pitch 参数为 CAN2、反馈 0x20A。存在同 ID 多设备时，仍须结合实际接线确认反馈对应哪台实物电机，未进行物理身份验证

本窗口右拨杆始终为 UP，命令状态始终 WAIT_NEUTRAL，云台及两轴均为 RELAX，pitch 输出为 0 A，云台发送状态为 OK。当前无输出直接受中位确认联锁影响，不能据此判断新电机失效或驱动力不足，也未验证电机收到非零命令后的响应

编码器为 243.9404296875 至 243.98439025878906 度，旧零点仍为 118.08016204833984 度，机械角为 +125.86026000976562 至 +125.90423583984375 度。更换电机后应在实物水平姿态重新核对零点，不把方向正确当作零点正确，也不将任意当前角度直接设为水平零点。本次未修改 RAM、config.c 或方向；ARCH/HW/SPEC 再次全通过

先保持 RELAX，在实物确认水平姿态并读取 pitch.encoder_deg、pitch.mechanical_angle_deg，核对既有水平零点与当前安装关系；不要直接删除限位或修改用户最终配置。坐标与实际行程确认安全后，让右拨杆经过中位确认再拨上，观察 remote_command.state=ACTIVE、gimbal.mode=ANGLE，并记录右摇杆动作时的 pitch 命令、目标、电流与反馈
