# 摩擦轮一阶目标差分前馈

用户要求仅增加摩擦轮前馈，系数沿用参考代码，后续自行上板测试。本次修改 `User/module/shoot.c/.h`，不改 PID 参数、`config.c`、结构体字段、底盘、设备驱动和 CubeMX 配置。

## 参考与单位换算

参考根目录：`F:/RM/robowalker_train-main/01_electrical/03_motor_control/09_test_feedforward_pid/homework/test_feedforward_pid`。

- `User/alg_pid.cpp:297`：`f_out = (Target - Pre_Target) * K_F`，先叠加 PID 后限幅，第 310 行保存目标历史
- `Core/Src/main.c:175`：作业答案速度环前馈系数 `K_F=30000`
- `User/dvc_motor.cpp:1021`：C620 速度输入由转子 rpm 换成减速后 rad/s
- `User/dvc_motor.h:321`：默认减速比 `3591/187`
- 本工程 `User/device/motor_rm.c`：M3508 电流指令按 16384 对应 20 A 换算
- 课件 `F:/RM/documents-export-2026-4-4 (1)/01_electrical/03_motor_control/3.9 动态目标与前馈PID.pptx` 第 7～9 页介绍目标差分前馈

本工程摩擦轮速度输入为转子 rpm，输出为 A，因此采用以下等效系数：

```text
Kff = 30000 × (20 / 16384) × (2π / 60) / (3591 / 187)
    ≈ 0.19970370881 A/rpm
Iff[k] = Kff × (target_rpm[k] - target_rpm[k-1])
Icmd[k] = clip(PID[k] + Iff[k], ±max_current_a)
```

保留参考离散差分形式，不除以 period_s。本工程任务为 500 Hz，参考任务按 1 ms 调用；相同连续斜坡在不同频率下的差分量不同，本次不额外按周期缩放，实际响应留待用户调试。

## 运行与调参

`shoot_speed_feedforward_gain_a_per_rpm` 是双轮共用的 `volatile float` RAM 调参变量，默认值用上述换算表达式初始化，可通过 EmberProbe 写入，写入 0 关闭前馈。每次 Control 入口只读取一次，形成两轮共用的本周期系数快照；不改变现有 config 副本、快照或 PID 参数持久引用。

每轮使用已有限幅目标保存独立差分历史。恒定目标前馈为零，失能、离线和故障在非运行周期清零历史，恢复时从零目标计算差分。PID 在线改参不清除连续运行中的目标历史。叠加输出遵守原有 `max_current_a`，当前默认上限为每轮 3 A。

系数非法或叠加计算溢出进入既有 SHOOT_FAULT，DumpOutput 统一使两轮输出零；修正参数后仍须先失能解除故障。未新增状态或旁路输出。

## 验证证据

2026-10-30 执行：

| 命令 / 方法 | 实际结果与验收对照 |
| --- | --- |
| `python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug` | 构建成功，生成 `build/Debug/Infantry_Robot.elf` |
| 提取 compile_commands.json 的 shoot.c 实际 ARM 编译命令，去除 `-o` 与 `-c`，增加 `-Wextra -Werror -fsyntax-only` | `ARM strict syntax: PASS` |
| `python build/shoot_feedforward_validation/validate.py` | 编译实际 shoot.c、pid.c、filter.c、user_math.c，反馈类型从正式头文件提取；正负独立差分、恒定目标、周期不归一化、PID 叠加及在线变参、3 A 限幅、目标限幅后差分通过 |
| 同一主机验证程序 | 失能重启、反馈超时、单轮离线两轮撤驱、恢复历史、NaN/±Inf/负系数、故障锁存与解除、有限极大系数导致计算溢出均通过 |
| 同一主机验证程序，前馈系数置 0 | 相对修改前 HEAD 的实际 shoot.c，5000 个周期的状态、目标、输出电流逐项精确一致，包含目标变化、在线改参、启停、单轮离线 |
| `python .auto-embedded/scripts/check.py` | ARCH、HW、SPEC 全部 PASS |
| `git diff --check -- User/module/shoot.c User/module/shoot.h` 及源码扫描 | 无行尾空白、同行左花括号、同行 else、超过 120 列；人工复核未新增缺失花括号或可合并的机械换行 |

人工复核：算法位于 shoot 模块，仍调用 component PID，未引入反向依赖；无新 RTOS/HAL 对象、ISR 共享状态或临界区；仅新增可写标量在入口读取，PID 参数仍引用实例内持久参数。新增分支为计算出口校验，复用已有故障状态和唯一输出闸门。

两份根规则在每个生产源码文件完成后均重新读取。临时主机测试源码、桩头、基线副本、对象和可执行文件在记录证据后清理；未增加长期测试资产。

## 新增检查自查

| 新增检查 | 实际触发场景 / 信任边界 | 上游是否已检查 |
| --- | --- | --- |
| `isfinite(feedforward_gain)` | EmberProbe 写入 NaN/Inf，在线调参入口 | 否，新参数在 Control 入口检查一次 |
| `feedforward_gain >= 0` | EmberProbe 写入负系数，在线调参入口 | 否，与有限性统一保存为 config_valid |
| `isfinite(current_a)` | 有限大系数乘以目标差分溢出，新增浮点计算链出口 | 否，PID 只保证其自身结果，不能覆盖后续前馈乘法和求和 |

无新增判空、保护性 return 或兜底赋值；未删除检查，未发现需删除的新增重复检查。DumpOutput 未增加配置或计算重复校验。

软件验证不代替实机验收。本次未烧录、未运行真实摩擦轮、未测电流或响应；尚需用户核对实际极性、启动峰值及连续变速响应。
