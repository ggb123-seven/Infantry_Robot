# 拨弹速度环变量统一

日期：2026-11-01，工作区：F:/RM/Infantry_Robot

## 用户确认与实现

- 用户要求目标和实际 rpm 放入速度控制实例，并明确参考底盘变量写法
- 延续已讨论的局部 Feeder_t 方案，修改 User/module/feeder.c/.h 与 User/task/motor_trigger.c/.h
- feeder.requested_speed_rpm：可写请求，volatile float，上电为 0，单位减速后输出轴 rpm
- feeder.target_speed_rpm：限幅后的内环目标，控制周期更新，保留前馈差分历史
- feeder.measured_speed_rpm：实际输出轴 rpm，由反馈入口更新，无有效反馈为 NaN，与底盘约定一致
- 删除独立全局 feeder_speed_target_rpm；Feeder_Control 不再接收速度入参，而在入口只读一次实例请求
- 速度 PID 和恢复时微分预置读取 measured_speed_rpm，请求字段不被限幅、停机或改参覆盖
- 保留右拨杆下位使能、位置环旁路、配置快照、已有 300 rpm/4 A 边界和 DumpOutput 输出闸门
- 不改通用 KPID_t、config.c、设备驱动、BSP、CubeMX 或摩擦轮

## 验证证据

| 命令或检查 | 结果与范围 |
| --- | --- |
| python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug | Debug 配置、编译与链接成功，产物 build/Debug/Infantry_Robot.elf |
| python build/validate_feeder_rpm_fields_20261101.py | 4039 项模块断言、54 项真实任务循环断言通过，总计 4093 项 |
| 复用 compile_commands.json，追加 -Wextra -Werror -fsyntax-only | feeder.c、motor_trigger.c ARM 严格检查通过 |
| 同一验证脚本的格式扫描 | 四份源码 Allman、控制块花括号、120 列、行尾空白、注释末尾及赋值机械拆分检查通过 |
| python .auto-embedded/scripts/check.py | ARCH/HW/SPEC 全通过 |
| git diff --check | 本次差异无空白错误 |

- 主机编译真实 feeder.c、motor_trigger.c、config.c、pid.c、filter.c、user_math.c，仅使用临时 HAL 类型与 RTOS 邮箱/时间替身
- 覆盖初始化零请求、无反馈 NaN、正负目标、300 rpm/4 A 限幅、请求与目标隔离、实际 rpm 进入 PID 误差链
- 覆盖 10 ms 反馈缓存边界、离线撤驱及恢复、无效角度标志、32 位时间回绕、500 周期正负请求交替
- 覆盖 NaN/Inf 请求故障锁存、停机档解除、在线 PID 修改和持久参数指针
- 真实任务连续执行七周期，覆盖零目标、正反目标、超限请求、停机保留请求和重新使能，验证电流邮箱结果
- 测试源码、可执行文件和替身头由 TemporaryDirectory 清理，临时验证脚本在证据保存后删除

## 校验自查与人工复核

| 本次改变项 | 信任边界与实际场景 | 上游是否已检查 |
| --- | --- | --- |
| isfinite 的输入从速度入参改为 requested_speed_rpm 的周期快照 | 调试器可能写入 NaN/Inf，沿用 Control 唯一入口检查 | 无，任务不再读取或校验目标 |
| 初始化实测 rpm 为 NaN，反馈无效时按已有 feedback_valid 输出 NaN | 反馈输入边界，无反馈或超时不能呈现为静止零转速 | 复用已有有效性结果，不重复校验协议或时间 |

- 未新增判空、数值范围检查、保护性 return 或重复 isfinite；没有需删除的重复检查
- 调试请求单字段 volatile，控制入口只读取一次；不把 volatile 描述为多字段一致快照
- config/cycle_config 保留，PID 参数仍指向实例内持久快照，任务不承担速度计算
- 没有新增 RTOS/HAL 对象、ISR 共享状态或临界区，main.c 与总线统一输出链保持
- 每份源码修改后重新读取 AGENTS.md 与 agent.md，人工核对中文集中字段说明、阶段目的注释、Doxygen 和状态机
- 新 ELF 尚未烧录，不能按新布局解释板上旧固件变量；未写目标 RAM，真实运动响应仍待上板
