# 校验边界修复记录

基线：main / ac3a7c8，日期：2026-10-30

用户授权按前述范围修复，SMC 不处理；保留 config.c 全部内容和 EmberProbe 可写实例 config。优先复用已有变量，确需新增时按现有命名集中注释并在交付中说明，不再逐项确认

## 实施清单

1. BSP 时间：从运算起点使用 64 位，扩展 32 位 tick 回绕，补偿挂起 SysTick，最小中断临界区保护共享计时状态
2. 启动入口：检查 LED 线程与 Task_Init 创建结果，失败进入现有 Error_Handler
3. 四个电机模块：Control 快照并集中校验，参数指针继续引用持久 applied 参数；DumpOutput 仅按状态导出
4. 任务与命令链：删除固定对象生命周期重复检查，编译期验证固定节拍约束，保留实际消息失败、周期和输入时效检查
5. 同步 EmberProbe 名称，逐项记录新增与删除检查、构建和复现结果，清理临时测试并本地提交

## 步骤 1～2：时间与启动入口

修改 User/bsp/time.c 与 Src/freertos.c 用户代码区，无 .ioc 或 FreeRTOS 配置修改

- 时间在约 71.6 分钟边界由 4294966500 正确推进至 4294967500 us
- 32 位 tick 回绕由 4294967295500 正确推进至 4294967296500 us
- SysTick 挂起时补记一拍，处理完中断后读取连续；100000 次样本非递减，原 PRIMASK 为 0 或 1 均准确恢复
- 相邻读取必须小于一圈 32 位 tick，内核节拍必须已启动；本工程周期任务持续调用满足该契约。临界区只采样寄存器和更新高位，64 位单位换算在恢复中断后执行
- 实际启动函数四个线程创建失败注入和正常启动均通过，失败进入现有 Error_Handler，不新增运行时字段

验证：

```text
python build/validation_boundary_fix/run.py time       PASS
python build/validation_boundary_fix/run.py startup    PASS
python build/validation_boundary_fix/run.py strict     PASS（本步骤两个 C 文件）
python .auto-embedded/scripts/check.py                 ARCH/HW/SPEC PASS
python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug
Debug 构建 PASS
git diff --check -- User/bsp/time.c Src/freertos.c      PASS
```

主机测试直接编译生产 time.c / freertos.c，替身提供 SysTick、SCB、内核接口、线程创建和 Error_Handler。启动测试替身首轮缺少弱符号及 USB/延时桩，补齐后重新通过；生产启动代码既有弱任务参数未使用，严格检查仅对 freertos.c 关闭 unused-parameter，未关闭其他警告

本步骤新增检查：SysTick 挂起标志属于硬件时间采样边界；tick 低位回绕比较属于时间边界，均无上游同条件检查。启动线程判空属于对象创建边界，此前没有检查。原内核状态下禁止等待的返回保留，没有新增内部实例判空、isfinite 或参数兜底

本步骤删除：原时间采样两次比较的分支由原子采样和明确挂起补偿替代；固定换算要求改为编译期断言。未上板，不以主机寄存器替身证明真实 IRQ 延迟或硬件调度时序

## 步骤 3：四模块快照与输出闸门

- 底盘、拨弹、摩擦轮使用入口局部快照；云台因反馈先于 Control 执行，在已有 config 之外增加 control_config 与 config_valid，反馈读取最近一次校验通过的配置，Control 采纳新快照
- 所有 PID 指针继续引用实例内 applied 参数，未指向局部快照；运行中参数变化直接同步参数、重置历史并按需重建滤波器，不重复 PID_Init
- DumpOutput 只依据运行状态放行候选电流，云台方向来自本周期已采纳字段；Control 后的 EmberProbe 写入不能改变同一周期的输出方向
- motor_rm.c 的 angle_valid 未检查原始编码器范围，云台反馈入口因此保留单圈范围检查；整数反馈换算不重复 isfinite

新增或迁移检查自查：

| 位置与检查 | 实际触发场景 / 信任边界 | 上游是否已检查 |
| --- | --- | --- |
| 四模块 Control 快照 config_valid | EmberProbe 在线写入非法配置 / 在线调参边界 | 否，Init 后仍可修改；集中一次替代下游检查 |
| 云台完整配置频率、超时最小 1 us 与上限、截止频率小于 Nyquist | 防止无效超时转换或滤波器参数 / Init 与在线调参边界 | 下层 PID_Init 有初始化校验，但运行中不调用；本周期入口负责 |
| 云台命令模式、速度有限、角度来源和目标计算有限 | 非法公共命令、增量运算溢出 / 命令入口及计算出口 | 否，起控捕获可能掩盖非法命令，故在捕获前检查 |
| Chassis_Control initialized 提前返回 | Init 失败后任务仍周期调用 / 初始化失败边界 | Init 记录结果，此处只消费结果，不重复配置校验 |
| 云台编码器单圈范围 | 原始编码器超过 8191 / CAN 反馈协议边界 | 否，angle_valid 仅检查连续角跟踪；保留旧范围校验 |
| PID 截止频率变化检测 | 在线改变滤波器频率 / 参数同步入口 | 否，过去只比较增益与限幅，新增字段比较 |

删除清单：固定实例与输出指针判空、整数 CAN 反馈换算后的 isfinite、子函数重复配置和周期检查、PID 返回值直接 isfinite、DumpOutput 配置校验/重复限幅、Reset 时按 param 或滤波系数猜测初始化、运行中重复 PID_Init 及其兜底分支、模式切换重复复位、混控后重复目标限幅、保护层已绑定对象判空与重复周期检查。消息缺失 NULL、反馈时效/协议校验、实测周期范围及后续计算出口检查保留

验证：

```text
python build/validation_boundary_fix/run.py modules
5000 周期：四模块电流、目标和状态与 ac3a7c8 逐字节一致 PASS
四模块计算中注入非法 config：当周期隔离、下周期停机 PASS
Control 后改方向：当周期输出不变、下周期采纳并锁存故障 PASS
反馈超时、自动恢复、PID 持久参数指针 PASS
保留非零候选电流时 DumpOutput 按停止状态输出零 PASS
非法周期、起控 NaN 命令、编码器越界但 angle_valid=true PASS
合法截止频率在线调参、四模块 Init 失败后输出零 PASS
python build/validation_boundary_fix/run.py strict    五个修改模块 C 文件 PASS
python .auto-embedded/scripts/check.py               ARCH/HW/SPEC PASS
python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug
Debug PASS
```

模块源码与头文件 Allman、120 列、行尾空白扫描通过；生成 freertos.c 的既有非用户代码排版未改。未上板；快照只保证复制后本周期使用一致，不提供调试器多字段写入事务的原子性

## 步骤 4：任务与命令链精简

- user_task.h 集中对 CAN、底盘、云台、摩擦轮、拨弹和 INS 的固定频率使用 _Static_assert，删除四个电机任务及 INS 的运行时整除校验、1 tick 或零频率兜底；实际调度间隔与模块周期边界仍保留
- 删除各任务对 Task_Init 已创建并验证的固定邮箱句柄判空；保留每次 Get、Reset、Put 返回值处理。DR16 ISR 数据指针及接收状态检查保持原样
- 删除只校验节拍并转调初始化的 MotorGimbal_Init；DR16 等待 tick 的固定范围改为编译期约束
- Cmd_ParseChassis 返回输入有效结果，Cmd_ParseChassisMotion 直接消费底盘或云台解析返回的有效结果，不再重复在线状态和 100 ms 帧龄检查；命令上下文、输出及固定遥控缓存指针采用调用方保证非空契约
- INS 删除内部数组及实例/输出判空、PID_Calc 返回值直接 isfinite；保留真实传感器错误、姿态与温控计算出口、PWM 失败处理

新增检查自查：本步骤没有新增运行时判空、isfinite、参数范围检查或保护性 return。Cmd_ParseChassis 的 false 返回替代原无返回值退出，是已有外部输入边界的结果传递；Cmd_ParseChassisMotion 的 input_valid 分支只消费校验结果，不重做检查。固定频率和等待 tick 的范围属于编译期配置约束，无上游编译期断言；Task_Init 对象创建检查保留，未移到循环

删除清单：固定邮箱 NULL、命令 ctx/out/rc NULL、INS 实例和内部数组 NULL、CAN 云台输出重复清零初始化、四电机与 INS 固定节拍运行时兜底、DR16 固定等待周期运行时上下限兜底、旋转意图解析重复帧龄/在线校验、INS 温控 PID 输出直接 isfinite。实际消息缺失、读写失败、外部数据时效和实际周期检查不属于冗余

验证：

```text
python build/validation_boundary_fix/commands.py
30000 组：底盘/云台命令值、状态、旋转意图与 ac3a7c8 逐字节一致 PASS
覆盖 99999/100000/100001 us、未来帧、离线、低 32 位时间回绕、非法周期与拨杆
python build/validation_boundary_fix/tasks.py
真实 CAN/摩擦轮/拨弹任务循环：九路电流路由、空邮箱零输出、缺命令失能、Reset/Put 失败复位 PASS
内核频率 500/999/1001 的编译期拒绝 PASS（本工程 INS 需要 1000 Hz）
python build/validation_boundary_fix/ins.py
10000 次实际 INS 更新：姿态字段与 PWM 占空比与基线逐字节一致 PASS
非法周期、BMI088 读取失败、PWM 失败关闭加热并撤销有效姿态 PASS
python build/validation_boundary_fix/run.py strict    九个修改 C 文件 PASS
python build/validation_boundary_fix/audit.py         全部修改 User 源码排版 PASS
python .auto-embedded/scripts/check.py                ARCH/HW/SPEC PASS
python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug
Debug PASS
```

临时测试曾出现 CMSIS 桩头定义冲突与计数变量和 puts 重名，修正测试替身后通过；严格 ARM 检查发现批量文本替换误把 imu_queue 前缀残留为函数名，已立即修正并全量重新检查。未修改 RTOS 创建流程、优先级、外设配置和 CAN 路由；上述任务证据为主机 RTOS 替身，不证明实际调度或 ISR 并发时序

## 最终复核补充：非法角度命令后的恢复

主机新增“绝对目标 NaN -> FAULT -> RELAX -> 合法增量起控”用例，先复现失败，再修复 Gimbal_ResolveSetpoint：每次先解析本次增量，存在有效运行角度目标时才叠加历史目标，起控和模式切换继续捕获当前姿态。不再沿用非法绝对命令写入的 setpoint。未新增判空、isfinite、范围检查或保护性 return；已有 target_angle_deg 有限性分支用于区别速度模式下合法的无角度目标，保留在原位置

该恢复用例修复后通过，5000 正常周期输出、目标和状态仍与基线逐字节一致。首次修正曾遗漏速度转角度模式时 target_angle_deg 为 NaN 的合法情况，由同一回归在第 200 周期检出，恢复原有适用性条件后全量通过
