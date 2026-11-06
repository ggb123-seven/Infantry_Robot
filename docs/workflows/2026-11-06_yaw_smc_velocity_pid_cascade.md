# yaw 滑模角度外环与 PID 速度内环

本文更新 2026-10-30 的 yaw SMC 直接电流控制实现。当前 yaw 的 ANGLE 路径由 SMC 生成速度目标，再交给原速度 PID 生成电流；pitch、yaw 的 SPEED 模式、反馈链、输出限幅和模块输出闸门保持原有职责。

## 实现调整

- `SMC_t` 继续使用扁平结构，参数和观测沿用该类型
- 在线参数字段为 `C`、`K`、`error_eps`、`q`、`p`、`speed_max_deg_s`、`surface_eps_deg_s`
- `SMC_Tick` 生成 `speed_target_deg_s`，不再生成电流
- yaw ANGLE 的滑模速度目标先受 SMC 速度上限、云台速度上限和机械行程允许速度共同约束，再进入原速度 PID
- yaw 速度 PID 的电流输出继续服从轴电流限幅、机械边界电流阻断和 `Gimbal_DumpOutput`
- pitch 继续执行原角度 PID 与速度 PID；yaw SPEED 仍直接把命令速度交给原速度 PID
- SMC 参数快照仍在初始化和 `Gimbal_Control` 入口集中校验；SMC 计算非有限输出时沿原轴故障路径撤驱
- 默认 SMC 速度上限 60 deg/s、边界层宽度 0.5 deg/s，属于未上板试验起点

## 软件验证

| 检查 | 命令或方法 | 结果 |
| --- | --- | --- |
| Debug 固件 | `py .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug -j 2` | 通过，生成 `build/Debug/Infantry_Robot.elf` |
| ARM 严格语法 | 从 `build/Debug/compile_commands.json` 取 `gimbal.c` 和 `slidingmodec.c` 的 ARM 编译命令，增加 `-Wextra -Werror -fsyntax-only` | 两个文件通过 |
| SMC 数值回归 | 临时主机程序链接实际 `slidingmodec.c` | 正负角误差方向、零角差制动、速度限幅、边界层连续性、非有限计算出口和参考轨迹限速断言通过 |
| 云台模块集成 | 临时主机程序链接实际 `gimbal.c`、SMC、PID、滤波和数学组件 | yaw ANGLE 正向速度目标与电流、pitch 串级 PID、yaw SPEED 直通速度 PID、机械边界速度裁剪断言通过 |
| 严格主机编译 | MinGW GCC 编译实际云台模块和依赖，宿主检查额外关闭 64 位主机解析 32 位 CMSIS 地址产生的指针转换警告 | 通过；ARM 严格语法检查另行通过 |
| 架构门禁 | `powershell.exe -NoProfile -ExecutionPolicy Bypass -File .auto-embedded/scripts/arch-check.ps1` | 0 项违规 |
| 硬件与规范门禁 | `py .auto-embedded/scripts/check.py` | HW、SPEC 通过；ARCH 由直接脚本单独复核通过 |
| 源码格式 | `git diff --check` 及 Allman、120 列、行尾空白扫描 | 通过 |

主机集成测试直接设置模块实例中的已就绪反馈状态，验证控制计算与状态路径，不覆盖外部 CAN/IMU 反馈解析和帧龄检查。测试源码与可执行文件均在验证后清理。

## 信任边界自查

- `SMC_IsConfigValid` 检查在线参数的有限性、符号、幂指数关系和速度边界；属于初始化及每周期配置快照边界，本周期算法不重复校验
- `SMC_Tick` 在速度目标限幅前检查计算链出口是否有限；输入参数已校验，但运算仍可能溢出，因此这是计算出口检查
- 本轮未新增判空、反馈范围检查、周期范围检查或其他保护性提前返回
- SMC 失败由现有单轴状态机进入故障，零输出由 `Gimbal_DumpOutput` 统一保证

## 验证边界

未烧录固件，未连接或驱动真实电机；正反方向、负载下稳定性、机械行程和线束安全、温升、控制周期耗时、任务栈高水位及相对原 PID 的实机效果仍需上板测量。
