# M2006 右拨杆命令链验证

日期：2026-10-31，工作区：F:/RM/Infantry_Robot

## 实测故障依据

- 旧固件 EmberProbe 采样：trigger.initialized、config_valid、feedback_valid 均为 1
- 15 点反馈采样的 last_update_time 增加 2913975 us，电机速度接近零
- trigger_command.enabled、target_speed_rpm、trigger.current_a 均为零，源码没有拨弹命令生产者
- RAM 的 trigger.config.speed_pid.p/i/d 和 applied_speed_pid.p 为零，与 config.c 接入初期配置一致
- 既有全局 Trigger_t trigger、Trigger_Command_t trigger_command 可直接观测，已加入 EmberProbe 侧栏

## 改动与数据所有权

- 仅修改 User/task/motor_chassis.c 和 user_task.h 的邮箱说明
- 同一底盘周期先消费 DR16 邮箱并缓存，再由 Cmd_ParseChassis 校验在线和 100 ms 帧龄
- 复用解析返回结果，右拨杆下位映射 enabled=true、target_speed_rpm=-120，其余情况发布失能零目标
- 每周期发布而非仅在遥控新帧时发布，避免消费者在遥控帧间隙缺命令失能
- 拨弹任务独占 trigger 和 trigger_command，生产者只写 RTOS 邮箱，不增加 ISR 共享变量或临界区
- Reset/Put 返回状态由发布函数短路汇总；发布失败不阻断底盘，拨弹消费者继续按未取得新命令失能
- Reset 本身失败时无法承诺邮箱旧值已清空，既有待消费旧值最多仍可消费一次，不声称所有 RTOS 故障都立即撤驱
- 保留 Trigger_Control 的 RAM 配置快照与集中校验、状态机和 DumpOutput 唯一输出闸门
- module/config.c、设备 ID、方向、CAN 驱动、CubeMX 配置均未修改

## 软件验证

1. python -B .auto-embedded/scripts/check.py：实现前、实现后 ARCH/HW/SPEC 全部 PASS
2. python -B .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug：构建成功
3. 复用 compile_commands.json 的 ARM GCC 命令，追加 -Wall -Wextra -Werror -fsyntax-only
   - motor_chassis.c、motor_trigger.c、init.c 均退出 0，无诊断
4. 主机 GCC 编译真实 motor_chassis.c 和 motor_trigger.c 任务循环，链接生产 cmd、chassis、trigger、config、fault_detect、mixer、PID、filter、user_math
   - 以模拟 RTOS 单槽邮箱、内核节拍和 BSP 时间替代硬件边界，184 项断言通过
   - 覆盖右拨杆上中下及非法值与左拨杆三档组合，底盘模式保持原映射
   - 覆盖 100000 us 有效、100001 us 超时、未来时间、离线及从未收到快照
   - 单帧缓存跨 6 个周期持续发布拨弹命令，不重复消费遥控队列
   - Reset 失败不执行 Put，Put 失败后空邮箱，拨弹发布失败不阻断底盘电流及云台状态邮箱
   - 覆盖实际拨弹任务的使能、失能、缺命令、反馈超时、反馈缓存和状态恢复
   - 默认零 PID 运行态输出仍为零；仅主机测试临时使用 P=0.01 验证非零电流链，未写入源码配置或目标 RAM
   - 模块周期异常进入 FAULT，有效使能命令不能解除，失能命令后解除
   - 最初测试将控制间隔设为 103 ms，实际进入 FAULT；修正夹具为合法 100 ms 间隔且反馈帧龄超过 100 ms 后 OFFLINE 用例通过
5. git diff --check 与源码行宽、花括号、行尾空白扫描通过，人工复核阶段中文注释及 Doxygen

主机结果见 trigger_remote_20261031.json，临时测试源、桩头文件和可执行程序已通过 TemporaryDirectory 清理

## 校验自查

| 新增判断或赋值 | 信任边界 | 上游是否检查同一条件 | 处理 |
| --- | --- | --- | --- |
| 复用 remote_valid | DR16 外部输入在线、时间边界 | Cmd_ParseChassis 已检查 | 下游不再重复检查 |
| sw_r 等于下位 | 遥控命令映射 | 设备层已验证协议合法性 | 仅映射请求，不重复做范围校验 |
| 失能时目标为零 | 命令边界撤销请求 | 消费者仍独立执行缺命令安全语义 | 明确发布本周期停机请求 |
| Reset/Put 状态比较 | RTOS 通信边界 | 操作结果只在发生处判断一次 | 失败沿用消费者缺命令处理 |

未新增判空、isfinite、数值范围校验或保护性提前 return；未引入下游重复校验，无需删除项
任务仅路由已校验请求，控制仍在模块，CAN 电流仍由 CAN 任务统一输出；无新增 RTOS 对象和 HAL 调用

## 参考参数核对

- Infantry_Robot_smc/User/module/feeder.c 的默认位置环 k=12、p=1、i=0、d=0.0450000018
- 参考 F:/RM/未命名文件夹/User/module/config.c 的步兵配置 165~186 行相同，10 齿、减速比 36
- 参考 shoot.c 使用输出轴弧度位置目标，正常每次减一齿，连续模式固定 20 Hz
- 归一化 PID 输出在参考 CAN 层乘 10000 LSB，即 M2006 的 10 A 满量程
- 当前 trigger 使用输出轴 rpm 的速度环并直接输出 A，参数维度与闭环结构不同，不直接复制上述位置增益
- 当前命令 -120 rpm 只对应参考平均进给速度，不等同于逐齿位置轨迹
- 尚未取得 User/module 修改与迁移结构体确认，因此本次只完成已授权的任务层命令链

## 实测边界与后续

- 本次未烧录，未中断当前采样，未写任何目标 RAM 或外设寄存器
- 新 ELF 与板上旧固件不同，未经重新烧录不得按新 ELF 地址解释旧固件变量
- 默认 PID 为零，不能把构建和主机回归当作真实电机已经转动的证据
- 后续须确认采用位置环迁移还是速度环整定，再按 EmberProbe 具体计划确认烧录与 RAM 写入
- 真机仍待验证右拨杆切换、实际拨弹方向、反馈丢失、遥控失联时延及机构载荷响应
