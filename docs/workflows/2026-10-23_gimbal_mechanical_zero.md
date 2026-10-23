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

## 线性 0°～360°目标修正（2026-10-23）

用户选择按数值连续位置控制，覆盖本记录上方“360° 与 0° 等价”和“按最短路径环绕”的旧行为

- `gimbal_yaw_angle.target_deg` 有效范围改为 `[0, 360]`，0° 与 360° 是不同目标；90°→360° 产生正向 +270° 误差，350°→10° 产生反向 −340° 误差
- 串级角度 PID 使用 `range=0`，目标超出范围或反馈非法时速度目标归零，不再将 360° 折叠为 0°
- `actual_deg` 从启动时的单圈相对位置开始连续累计，编码器从 359° 跨到 0° 时反馈位置继续递增
- GM6020 当前 `gear=false`，设备驱动的 `rotor_total_angle` 仍是单圈角度；云台模块以相邻有效编码器样本展开位置，不增加公开结构体字段
- 连续位置已经建立后，若角度样本间隔或反馈年龄超过 90 ms、角度连续性标志失效或半圈方向不确定，模块清除零点与使能并输出零电流，需重新标定后再使能
- `Gimbal_YawRunAt()` 增加 `angle_valid` 输入，当前工程调用点均已更新；独立角度环仍可按参数选择单圈环绕或连续线性模式

## 零点标定参考

`F:\RM\未命名文件夹\校准文档.md` 的 `set-mech-zero` 命令记录当前编码器位置，并将结果写入配置 Flash；这是由程序执行的人工定位采样，不是上电自动寻找机械正前方
当前工程仍使用 `gimbal_yaw_zero_capture` 停机后采样，零点保存在 RAM，未接入命令行或 Flash 配置；没有机械原点传感器时，程序无法仅凭单圈编码器自动判断云台正前方

## 本次验证

- `python .auto-embedded/scripts/check.py`：ARCH、HW、SPEC 全部通过
- `python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug`：Debug 配置与构建通过
- ELF：`build/Debug/Infantry_Robot.elf`，SHA256 `0CE8FB38DD24518E33D4D77419003C17B222DE66B3142C5161CF31A0A2704985`
- 主机回归链接实际 `gimbal.c`、`pid.c`、`filter.c` 与 `user_math.c`，覆盖 90°→360°、0° 与 360° 区别、350°→10° 和 10°→350° 方向、跨编码器零点连续位置、非法目标、角度跟踪失效及 90 ms 样本间隔失效
- `gimbal.c` 使用 `-Wall -Wextra -Werror` 独立编译通过；全量主机回归仅报告既有 `pid.c` 微分变量和 `user_math.c` 严格别名告警
- 本次没有烧录、没有触发电机动作；真实机械正方向、机械零点精度和云台闭环效果仍待上板验证
- 临时验证源码、替代头文件、对象文件与可执行文件位于 `User/task/.gimbal_linear_verify`，记录证据后清理

## 预留外部偏航角速度接口（2026-10-23）

- `Gimbal_Update(period_s, yaw_rate_feedback_rad_s, yaw_rate_feedback_valid)` 接收独立的外部角速度，并传给 `Gimbal_YawRunAt()` 的角度外环
- 外部角速度单位为 `rad/s`，有效且未过期时传 `true`；未接入或不可用时传 `false`，模块继续使用编码器 `rpm` 换算值，忽略外部占位数值
- 当前 `User/task/motor_gimbal.c` 调用为 `Gimbal_Update(period_s, 0.0f, false)`，外部 INS 尚未接入
- 传入 `true` 却提供 NaN 或 Inf 时，本周期两环停止并清空控制历史，目标速度和目标电流归零
- 速度内环仍使用编码器转速，`gimbal_yaw_speed.actual_rad_s` 仍表示编码器换算的实际角速度；外部角速度目前只用于角度外环的反馈微分
- 后续在任务层取得一致且有时效保证的 INS 快照，选择并转换偏航轴，再替换上述两个输入；模块不读取任务全局 INS 对象，不增加结构体字段
- 外部角速度必须与角度反馈采用一致方向和参考系；当前角度为相对底盘的机械位置，陀螺仪为惯性角速度，底盘旋转时需评估参考系转换或相对角速度计算，不能直接视为相同物理量

### 接口验证证据

- Debug 配置与构建通过，使用 `.auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug`
- `python .auto-embedded/scripts/check.py`：ARCH、HW、SPEC 全部通过
- `gimbal.c` 使用主机 GCC `-std=gnu11 -Wall -Wextra -Werror` 独立编译通过
- 主机回归链接实际 `gimbal.c`、`pid.c`、`filter.c`、`user_math.c`，`PASS assertions=27`
- ELF：`build/Debug/Infantry_Robot.elf`，SHA256 `B347E4350342E9F8010A199E164F02E55D81BEF749D7B65A5845D16FE2AB06FB`
- 覆盖外部正负角速度选择、与速度内环反馈隔离、无效标志回退、有效标志下 NaN/Inf 停机及恢复、90°→360°正向误差、0°与 360°区分、跨零连续位置、角度失效、公开更新入口传递和电流发布失败停机
- 临时测试源码、替代头文件、对象文件和可执行文件位于 `User/task/.gimbal_rate_verify`，记录证据后清理
- 本次没有烧录或触发电机动作，尚未验证真实陀螺仪数据链路、安装方向及闭环精度
