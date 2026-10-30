# yaw SMC 复审修复记录

日期：2026-10-30；分支：feature/gimbal-smc；修复基线：d5e3aa6

## 问题与修复

1. governor 的 RESTORING 持有固定捕获目标，但原 IsTracking 把此阶段也当作持续跟随，导致 SMC 每周期重置至当前姿态。现仅在 BRAKING、HOLD、SUSPECT 和独立观测等待/不可用时持续跟随；RESTORING 保留捕获目标并恢复纠偏，保护状态机及 PID 逻辑不变
2. 在线降低 max_speed_rad_s 后，速度变化斜坡曾保留超出新上限的历史速度。现对斜坡结果施加本周期速度上限；硬速度上限优先于正常加速度约束，target_ddot 记录真实参考变化，电流仍受既有电流上限和保护约束。60 降到 1 度每秒、周期 2 ms 时，参考为 1 度每秒，加速度为 -29500 度每平方秒；反向情况对称
3. 原命令入口只检查原始 float 有限，增量转为度时可溢出。现把绝对角检查迁移到命令解析出口，同时覆盖增量运算结果，在 governor 捕获和饱和运算前拒绝异常。未运行或模式切换时不使用旧参考，而是先验证增量换算，再捕获当前有效姿态

## 本轮验证

- 启动及最终机械门禁：PYTHONDONTWRITEBYTECODE=1 下运行 `python .auto-embedded/scripts/check.py`，ARCH、HW、SPEC 全部通过
- 主机命令：`python F:/RM/Infantry_Robot/build/test_smc_review_fix.py`，编译真实生产 gimbal、governor、SMC、PID、滤波、数学和连续角组件，仅用设备头替身隔离 HAL；默认参数从 config.c 读取
- 611 个模块周期通过：正负速度上限收紧、正负姿态偏差恢复纠偏、两轴正负有限溢出命令、RELAX/OFFLINE/模式切换捕获、非法绝对目标，以及 BRAKING -> HOLD -> RESTORING -> GUARDED 完整恢复
- 正负 2 度偏差、捕获目标 0 度，恢复阶段输出分别为 -0.004 A 与 +0.004 A，参考保留 0 度
- 有限 FLT_MAX 增量命令在 RUNNING、RELAX、OFFLINE 均被拒绝，该轴进入 FAULT 且 DumpOutput 导出零；另一轴继续运行
- 20000 个 2/4 ms 变周期参考轨迹周期通过，速度不超过 60 度每秒，正常推进加速度不超过 180 度每平方秒
- 相对 d5e3aa6 编译旧模块并重命名导出符号，12000 周期 PID 对照通过：SPEED 两轴和 ANGLE pitch 的状态及电流精确一致，覆盖绝对与增量来源
- `python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug -j 2` 通过，产物为本工作树 build/Debug/Infantry_Robot.elf
- 从 compile_commands.json 提取三个修改 C 文件的 ARM 实际编译命令，去掉 -o、-c、-fstack-usage，加入 -Wextra -Werror -fsyntax-only，全部通过
- UTF-8、Allman、120 列、行尾空白及 git diff --check 通过；gimbal.c 为 795 行，governor 为 770 行
- SMC_t 的 19 个 float 字段及顺序不变；config.c 与 CubeMX 配置无差异；未新增 RTOS/HAL 对象、ISR 共享状态或临界区，分层依赖不变

原 22242 周期验收未覆盖上述三种情况，不能用其通过结论代替本次复审结果；初版记录保留为历史证据，以本修复记录补充

## 校验与防御性编程自查

| 本轮新增或迁移项 | 信任边界与实际触发场景 | 上游是否已检查同一条件 |
| --- | --- | --- |
| ResolveSetpoint 的 isfinite(target_angle_deg) 与 false 返回 | 命令解析计算链出口：非法绝对角或有限增量换算溢出，必须在捕获及限幅前拒绝 | 原始速度检查不能保证换算结果有限；原绝对角检查已移除此处统一执行 |
| 斜坡后参考速度限幅 | 在线配置上限变小后，历史速度超出新上限；属于轨迹约束 | speed_goal 的限幅不约束历史 previous_speed，经斜坡后的值需要服从新上限 |
| 非运行或模式切换时选择增量基值 0 | 旧目标由 StopAxis 或 SPEED 设为 NAN，不作为新姿态目标，换算有效后统一捕获反馈 | 按显式运行状态处理参考生命周期，无新增初始化推测或静默异常兜底 |

本轮未新增判空、配置校验或下游 isfinite。删除旧绝对命令字段的提前检查及对应保护返回，改由解析出口统一检查；未增加结构体类型或字段

## 调参与验证边界

EmberProbe 调参仍通过 gimbal.config 和 gimbal_yaw_smc_config 的既有 RAM 副本，Control 入口单次快照、集中校验与同步约定保持不变，DumpOutput 只按状态导出输出

本轮未烧录、未运行真实电机，尚未验证真实负载稳定性、方向、温升、控制周期耗时和任务栈高水位。临时测试源码、可执行文件、旧模块副本和编辑脚本在留存本记录后清理，保留正常固件构建产物
