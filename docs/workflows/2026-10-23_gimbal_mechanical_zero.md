# yaw 固定机械零点与单圈相对角控制

日期：2026-10-23

用户确认以云台机械正前方为固定 0°，并授权修改 `User/module/gimbal.c` 和 `gimbal.h`
现有 `GimbalYawAngle_t` 与 `GimbalYawSpeed_t` 的名称、字段和布局保持不变

## 行为与单位

- `Gimbal_YawRunAt()` 输入仍是原生编码器单圈角度，单位度，不得由调用方预先减去零点
- 实际控制角度为 `wrap360(编码器角度 - 机械零点角度)`，范围 `[0, 360)`
- `gimbal_yaw_angle.target_deg` 为机械相对角，有限目标按 360° 环绕；360°、720° 均与 0° 等价，不表达整圈旋转指令
- `gimbal_yaw_angle.actual_deg` 为机械相对角，反馈失效或未标定时显示非数值
- 单圈外环保留 2π 最短路径误差，359°→1° 为 +2°，1°→359° 为 -2°；内部 PID 角度单位和增益单位不变
- 独立 `Gimbal_YawAngleRun()` 不执行机械零点换算，`range=0` 的连续角度接口保留原有多圈语义
- 串级默认禁用，零点未标定时禁止两环输出；初始化、零点改变或失效后均要求手动重新使能
- 未增加电机主动寻零动作，未写入闪存，本次未采集实际机械正前方的编码器值

## 调试器标定步骤

1. 写入 `gimbal_yaw_angle.enabled=false`，保持任务正常调度，等待电流输出归零
2. 手动将云台对准机械正前方，观察 `gimbal_yaw_encoder_deg` 确认为有效编码器角度
3. 写入 `gimbal_yaw_zero_capture=true`，模块在下个控制周期单次处理并清除此请求
4. 检查 `gimbal_yaw_zero_deg` 等于此时编码器角度，`gimbal_yaw_angle.actual_deg` 接近 0°
5. 记录 `gimbal_yaw_zero_deg`，填写所需目标角度及已整定的角度环增益，再写入 `enabled=true`

运行中、尚未完成停用、未初始化或反馈无效时拒绝采样，请求仍当次清除，不会在恢复后延迟执行
采样请求清除只代表已处理，应通过零点数值和实际相对角确认标定结果
已记录的固定零点可在停用状态直接写入 `gimbal_yaw_zero_deg`，待模块采用后再使能
零点有效范围为 `[0, 360)`，越界、无穷或非数值均表示未标定；编码器数值 0° 本身是合法机械零点
零点保存于 RAM，复位后恢复源码初值非数值，需要重新填入已记录数值或人工标定，不会将上电位置自动设为零点
角度环增益初值仍为零，需要真实电机调参后才会产生角度跟随输出

## 验证证据

- 修改前后 `python .auto-embedded/scripts/check.py` 的 ARCH、HW、SPEC 全部通过
- 使用 `aemb-build-cmake` 构建技能，`python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug` 配置和构建成功
- 固件为 `build/Debug/Infantry_Robot.elf`，SHA256 为 `b9ac2b382f49de4a2d1a8d34ad9b0d4468399bba7e39635fe8c43965c303b7ff`
- 主机链接实际 `gimbal.c`、`pid.c`、`filter.c`、`user_math.c`，使用仓库现有电机反馈类型及 CMSIS-RTOS2 声明，仅替代硬件时间和邮箱服务
- 34,195 条断言通过；整圈扫描采用三角函数的独立参考误差，比对机械相对角、外环输出方向和电流限幅，整半圈只比较误差绝对值
- 覆盖人工标定、相对 90° 目标、运行中拒绝标定、停用当周期拒绝标定、零点改变清除非零积分、零点失效和非法状态停机
- 覆盖正反跨零、多圈及负目标环绕、360° 与 0° 等价、极小负角舍入、最大有限角度和独立连续角度模式兼容
- 覆盖反馈超时精确边界、未来时间、无反馈、非法编码器角度、非数值目标和增益、异常周期、初始化失败
- 覆盖真实模块的 rad→度转换、反馈邮箱缓存及超时、邮箱读取/重置/写入失败时停用两环
- `gimbal.c` 主机独立使用 `-Wall -Wextra -Werror` 编译通过
- 主机共用组件仍有既有告警：`pid.c` 非法枚举分支可能留下未初始化微分量，`user_math.c` 快速平方根存在严格别名告警；本次未修改组件
- 两个修改文件的 Allman 花括号、控制块缺失花括号、120 列、行尾空白、机械断行及差异空白检查通过
- 人工复核中文注释、Doxygen 参数与返回值、集中参数说明、分层依赖、返回值检查和默认零电流路径
- 模块状态与采样事件由云台任务串行处理，未增加 ISR 共享对象或临界区，`volatile` 仅用于调试器可见性，多个调试字段不提供原子快照
- 不修改任务、BSP、设备驱动、HAL、CubeMX 配置或硬件资源，沿用原有邮箱与统一 CAN 输出路径

主机回归最终输出：

```text
PASS manual mechanical calibration and relative 90 degree target
PASS running and stopping calibration rejection
PASS zero changes and invalid zero state stop both loops
PASS full-circle relative coordinates and shortest paths with wrapped targets
PASS independent continuous-angle compatibility and invalid input reset
PASS stale feedback, timestamp and nonfinite input stop paths
PASS module mailbox conversion, cache, timeout and failures
PASS initialization preserves zero and requires manual enable
PASS total assertions=34195
```

临时测试位于 `User/task/.gimbal_zero_verify`，证据记录后删除测试源码、替代头文件、对象文件和可执行文件
本轮验证为软件构建与主机回归，未烧录、未操作电机、未验证实物正方向及真实机械零点，不代替上板整定
