# 拨弹盘下位持续转动与回中抖动诊断

日期：2026-11-11

用户确认异常部件为 M2006 拨弹盘，表现为右拨杆下位持续转动、回中后观察到类似五连发及往复抖动

## 当前源码与验证结果

本轮未修改生产源码、配置或目标 RAM，未烧录、复位或主动驱动电机。工作树原有的 cmd、feeder、config、设备方向等差异保留

真实 `cmd.c`、`feeder.c`、`config.c`、`pid.c`、`filter.c`、`user_math.c` 的主机编译参数为 `-std=gnu11 -fgnu89-inline -O0 -g -Wall -Wextra -Werror`，两轮编译和运行退出码均为 0。第二轮直接包含真实 feeder.c 观察模块私有计数，没有复制生产状态机

隔离 HAL 依赖的替身仅含从正式 bsp/can.h 提取的 BSP_CAN_t 枚举，使用正式业务结构体，没有新增生产结构体。主机诊断禁用云台正弦测试，合成反馈只验证控制分支，不模拟实物动力学

| 条件 | 实测主机结果 | 含义 |
| --- | --- | --- |
| 中位后下位，正常合成逐齿到位反馈，持续开火 | 首齿目标 180°→135°，剩余 4；最终执行 5 齿后进入 WAIT_RELEASE，剩余 0 | 正常反馈下，持续下位没有无限追加目标 |
| 回中后施加 +5° 位置偏差 | RUNNING，single_fire=0，DumpOutput=-1.308997 A | 中位取消进给但仍保持位置闭环 |
| 回中后施加 -5° 位置偏差 | DumpOutput=+1.308997 A，剩余 0 | 回中后的反向纠偏不等于新触发五发 |
| 开火后 3.2 秒没有新反馈 | RUNNING、EXECUTING，剩余 4，DumpOutput=-10 A | 当前代码已移除反馈帧龄检查，继续按旧反馈计算电流 |
| 下位不释放，交替输入角度无效帧和有效帧 | 连续 8 次恢复均重新进入 EXECUTING，剩余重新变成 4，电流请求 -10 A | 反馈恢复直接重新建立五发预算，可以累计进给超过五齿 |
| 上位安全 | DumpOutput=0 A | 已有安全档仍通过唯一输出闸门撤驱 |

恢复测试中首齿目标为 135°，八次恢复后的目标依次为 90°、45°、0°、315°、270°、225°、180°、135°。每次恢复均以当时实测位置重新捕获并再推进 45°，整个测试期间 single_fire 始终为 true

上述电流均为软件输出请求，不是实测相电流

## 源码解释

- `Cmd_ParseFeeder`：中位输出 RUN 且 single_fire=false，下位输出 RUN 且 single_fire=true，复位 fire_mode=2 为五连发。当前映射没有在回中时主动触发五连发
- `Feeder_UpdateSingle`：回中只捕获当前位置、清空剩余齿数并重置 PID，随后仍进入位置控制路径；惯性或外力改变位置后会输出纠偏电流
- `Feeder_Control`：角度无效进入 OFFLINE，恢复进入 RUNNING 时重置执行阶段为 WAIT_CENTER；当前 WAIT_CENTER 在 single_fire=true 时直接启动整组，形成恢复重触发
- `Feeder_UpdateFeedback`：now_us 已不参与时效校验，没有新帧时保留上次有效角度与速度。通信任务只发布新帧，不替代模块的超时边界检查
- 当前默认角度环开、速度环关，位置 PID 直接输出 A；源码配置 P=15、I=0、D=0.005，电流边界 10 A。45° 静止误差下主机输出达到 -10 A。该输出和弱速度阻尼是机械过冲的候选因素，尚未用实物曲线确认
- 设备层的 reverse 同时作用于反馈和输出，不能仅凭 reverse=true 推断闭环成为正反馈

## 板上读取限制

EmberProbe 配置指向 `build/Debug/Infantry_Robot.elf`，文件修改时间为 2026-11-11 12:53:57，SHA-256 为 `A4B760510C03B1903D377AF212C7D8513122EDC7A2CA8C5332E69F9BC2530061`

按工作区配置尝试 variables_read，两次均返回 `OPENOCD_START_FAILED`，阶段为 openocd_start；首次错误 retryable=true，查询 sampling_status 确认 stopped 后只读重试一次仍失败。OpenOCD 日志末尾为使用 Horco CMSIS-DAPv2，未取得目标 RAM 或固件一致性结果

因此不能确认现场物理档位对应的 sw_r、板上 fire_mode 和环路开关，也不能确认实际反馈是否发生无效恢复或停止更新。主机已复现软件缺陷，但不能把这些条件当作本次现场已发生的事实

## 待确认的修复行为

后续用户已确认修改行为：未完成的组反馈恢复后自动重新拨五齿，已完成的组保持撤驱，回中取消当前组并撤驱；取代下方早期等待回中的方案。实现及验证见 ../validation/feeder_group_recovery_20261111.md

修复需要根目录 AGENTS.md 授权范围之外的 User/module 修改，已向用户明确请求模块修改范围和行为确认，确认前保持生产源码不变

1. 下位触发一组五发，完成后保持下位不重复
2. 回中取消剩余发数，拨弹退出位置保持并由 DumpOutput 输出零
3. 反馈失效撤驱，恢复后等待回中重新触发，不自动重建整组
4. 恢复原有反馈时效信任边界和无符号时间戳回绕处理
5. 保留当前单角度环接口与 config.c 参数，摩擦轮中位和下位请求运行的编排独立保留
6. 不新增结构体字段，不增加单齿执行超时或故障锁存

## 三层复核

- 验证门：真实生产链两轮主机编译与上述行为断言通过；cmd.c、feeder.c、motor_trigger.c 使用当前 compile_commands.json 的 ARM 编译参数执行 -fsyntax-only -Wextra -Werror，三份文件退出码均为 0；未完成实物复现和新固件运行时栈验证
- 硬件合规：初次 check.py 受 PowerShell 脚本签名策略阻止；使用仅进程级 PSExecutionPolicyPreference=Bypass 后，ARCH、HW、SPEC 全 PASS，未改系统策略或硬件配置
- 代码质量：本轮未改 main.c、ISR、临界区、RTOS 对象和生产代码；拨弹运行状态由任务/模块独占，在线入口为 volatile 标量，配置通过持久周期快照读取，输出仍由 Feeder_DumpOutput 统一放行
- 检查自查：没有新增或删除生产判空、isfinite、范围检查与保护性 return
- 临时测试源码、替身头文件和可执行文件均由 User/task 下的 TemporaryDirectory 清理，两轮均确认临时目录不存在，没有删除正式测试资产
