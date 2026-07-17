# M3508 速度环在线调参与监视接口反思

**日期**: 2026-07-17
**提交类型**: feature
**修改文件数**: 7 个文件

## 1. 概述

将单个 M3508 速度环的 Ozone 调参入口和运行监视量统一到
`g_motor_chassis_monitor`，并把上板整定后的 PI 参数固化为
`Kp=0.5 A/rpm`、`Ki=0.14 A/(rpm*s)`、`Kd=0`。监视结构体同时提供目标
转速、减速箱输出轴实际转速、原始转子转速、电流指令、积分状态、在线状态和
CAN 结果，便于在一个 Ozone 变量中完成调参与故障定位。

## 2. 用户需求与提示词

- 将 Ozone 调参变量集中到一个具名全局结构体。
- 在同一结构体中观察除以减速比后的真实输出轴转速。
- 区分目标转速、原始转子转速和实际输出轴转速。
- 完成速度环上板调试后提交代码。

## 3. 工作流记录

1. 核对 M3508 的 `3591/187` 减速比换算路径。
2. 将目标转速和 PID 参数从任务层按周期复制为局部快照，再传入速度算法。
3. 增加统一 Ozone 监视结构体并持续刷新运行状态。
4. 通过 Ozone 完成速度环上板调参，将最终 PI 参数写回源码宏。
5. 执行 Debug 构建、机械门禁、符号检查和人工代码审查。

## 4. 修改内容

- `motor_chassis.c/.h`：新增 `g_motor_chassis_monitor`，集中暴露调参量和监视量。
- `motor_speed_control.c/.h`：由调用方传入本周期 PID 参数，移除算法模块中的独立
  Ozone 全局变量。
- `.auto-embedded/spec/conventions/index.md`：沉淀 Ozone 统一监视结构体约定。
- `docs/workflows/2026-07/17_reflection_feature_motor_speed_tuning.md`：记录本次调参与
  接口调整过程。

## 5. 遇到的问题

1. 独立 PID 调参变量与运行反馈分散，Ozone 中需要同时展开多个对象。
2. 全局变量初始化器只显式列出非零调参初值，真实转速字段不易从初始化器中确认。
3. `actual_speed_rpm` 与 `raw_rotor_speed_rpm` 的含义容易混淆。
4. Ozone 修改的是 RAM，最终参数若不写回源码，复位后会恢复旧默认值。

## 6. 根本原因分析

- 调试接口最初按算法内部状态组织，没有围绕上板观察流程组织。
- 没有在变量命名和集中说明中同时呈现减速比前后的速度含义。
- 在线调参与持久化配置属于两个阶段，前期只实现了在线修改入口。

## 7. 调试过程

- 确认设备层在 `gear=true` 时执行 `raw_speed / (3591/187)`，任务层读取的
  `motor_3508->feedback.rotor_speed` 已是减速箱输出轴 rpm。
- 确认 `actual_speed_rpm` 每个控制周期写入 `g_motor_chassis_monitor`。
- 使用 `raw_rotor_speed_rpm`、`actual_speed_rpm`、`current_command_a`、
  `pid_integral_error_rpm_s` 和 `feedback_age_us` 区分反馈跳变、PI 输出和 CAN 延迟。
- 用户完成上板调试并将最终参数固化为 `Kp=0.5`、`Ki=0.14`、`Kd=0`。

## 8. 经验总结

- 在线调参入口和关键运行反馈应集中在同一个具名 `volatile` RAM 全局量中。
- 目标、反馈和输出必须标明单位及其位于减速比的哪一侧。
- Ozone 调试完成后必须将最终参数写回编译期默认值，并重新构建验证。
- 监视结构体用于调试观察，不保证跨多个字段的原子一致快照；控制算法只读取任务
  每周期复制出的局部参数快照。

## 9. 知识提炼

### 可复用模式

- task 拥有 Ozone 调参和监视结构体，module 只接收已复制的参数值。
- 同时提供处理后的工程量和协议原始值，便于判断异常发生在设备反馈还是控制计算。
- 实时反馈字段显式初始化为零，随后由唯一任务按周期刷新。

### 检查清单

- [x] 最终 PID 参数已写回源码。
- [x] 目标速度和实际速度单位均为减速箱输出轴 rpm。
- [x] Ozone 全局符号保留在 Debug ELF 中。
- [x] 调参字段使用 `volatile`，任务每周期读取一次局部快照。
- [x] 未修改 CubeMX 配置和受限底层源码。

## 10. 测试与验证

- 用户确认速度环上板调试完成。
- `cmake --build --preset Debug`：编译和链接通过。
- `python .auto-embedded/scripts/check.py`：ARCH、HW、SPEC 全部通过。
- `ctest --test-dir build/Debug --output-on-failure`：工程未注册自动化测试。
- `g_motor_chassis_monitor` 保留在 Debug ELF 的 RAM 符号表中。
- 本次未取得量化的超调量、上升时间和稳态误差记录，报告不虚构测试数值。

---
**生成工具**: Codex
**技能**: commit-with-reflection v3.0
