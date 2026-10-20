# 修改记录

## gm6020-rad-reverse-20260823-03

- 目标：在 CAN 设备快照中增加 GM6020 的 `rad/s` 反馈量，为后续速度环单位切换提供明确输入
- 源码改动：`User/device/can_devices.h` 新增 `gm6020_speed_rad_s`；`User/device/can_devices.c` 按 `2π/60` 将协议反馈 rpm 换算为 rad/s；保留 `gm6020_speed_rpm` 作为原始协议诊断量
- 方向边界：保持 GM6020 ID `0x209` 的 `.reverse = false`，本轮未改变输出和反馈符号处理
- 验证：`git diff --check` 通过；`cmake --build build/Debug --parallel 4` 成功生成 `build/Debug/Infantry_Robot.elf`
- 状态：本改动点已完成，待继续切换 GM6020 速度环和 Ozone 字段单位

## CMAKE-PATH-20260721-01

- 目标：修正 VS Code CMake Tools 对不存在的 `cube-cmake` 的引用。
- 配置改动：将 `.vscode/settings.json` 中的 `cmake.cmakePath` 指向系统 CMake 3.31.4，并移除错误的 `CMAKE_COMMAND=cube-cmake` 配置参数。
- 验证：JSON 解析通过且配置中已无 `cube-cmake`；系统 CMake 3.31.4 完成 `Debug` 预设配置，增量构建返回 `ninja: no work to do.`。
- 状态：已完成。

## MOTOR-FEEDBACK-DIAG-02

- 目标：修复 `Task_Init` 初始化任务栈不足。
- 源码改动：将 `User/task/user_task.c` 中 `attr_init.stack_size` 从 1024 字节增至 4096 字节，并同步修正任务属性集中说明。
- 依据：Debug 构建的 `.su` 报告显示最深静态初始化链约 2080 字节，原配置和 2048 字节中间配置均不足以覆盖该链路并保留中断余量。
- 验证：源码规范扫描通过；主机侧 `init_task_test` 输出 `Init task tests passed`；`build/Debug` ARM Debug 构建成功；新 ELF 中 `attr_init.stack_size` 为 `0x00001000`（4096 字节）。
- 资源影响：只在 `Task_Init` 存活期间从 15360 字节 FreeRTOS heap 额外占用 3072 字节；任务退出后由 Idle 任务回收，不改变其他任务的优先级、周期或栈配置。
- 残余风险：仍需在目标板确认初始化峰值 heap 和实际栈水位，并验证 CAN 反馈链是否恢复。
- 限制：当前未连接 GDB Server/目标板，无法读取运行态栈水位，也无法确认刷写后 `feedback_update_status` 是否仍为 `-4`。
- 状态：软件验证完成，待烧录新 ELF 进行上板验证。

## GM6020-R01-CAN-MAILBOX-COMMENT

- 目标：明确底盘 CAN 反馈邮箱和命令邮箱的 3508 电机业务归属
- 源码改动：仅修改 `User/task/init.c` 中 `can_feedback` 和 `can_command` 创建前的两处注释，未改动运行逻辑
- 依据：`can_feedback` 由 `Task_can` 发布并由 `Task_motor_chassis` 消费；`can_command` 承载四路 M3508 电流命令
- 验证：注释格式与 120 列检查通过，`git diff --check` 通过，CMake Debug 构建成功
- 状态：待用户确认后创建本地 Git 快照

## GM6020-R02-STATE-MACHINE-RULE

- 目标：为全局协作规则和项目局部规则补充复杂业务分支优先状态机约束
- 规则改动：在 `AGENTS.md` 与 `agent.md` 中分别新增“分支与状态机”章节
- 依据：`User` 源码当前检出约 512 个 `if` 和 15 个 `switch`，复杂流程需要与简单保护判断区别治理
- 边界：跨周期状态、互斥业务模式、超时重试和故障恢复使用状态机；空指针、参数及返回值检查保留 `if`
- 验证：两份规则均包含适用场景、状态结构、安全默认值和允许保留 `if` 的边界，`git diff --check` 通过
- 状态：待用户确认后创建本地 Git 快照

## GM6020-R07-AUTO-REVERSAL

- 目标：让 GM6020 上电后以 0.05 A 电流进行每 3 秒换向的自动往复测试
- 源码改动：在 `User/task/motor_gimbal.c` 中新增正向和反向两状态测试状态机，依次输出 `+0.05 A` 和 `-0.05 A`
- 安全边界：仅在本周期取得在线反馈时允许非零输出，反馈缺失、设备离线、命令发布失败或非法状态均输出零电流
- 监视方式：通过 `g_motor_gm6020_monitor` 观察目标电流、限幅电流、实际下发电流和反馈电流
- 验证：UTF-8、Allman 花括号、120 列和 `git diff --check` 检查通过，CMake Debug 构建成功并生成 `build/Debug/Infantry_Robot.elf`
- 状态：待用户确认后创建本地 Git 快照

## GM6020-TIME-01

- 目标：将 GM6020 正反转测试的每方向电流持续时间由 3 秒调整为 10 秒
- 源码改动：`User/task/motor_gimbal.c` 中 `MOTOR_GIMBAL_TEST_DIRECTION_TIME_MS` 由 `3000U` 改为 `10000U`，并同步更新对应说明
- 验证：方向状态机使用 `10000U` 参与毫秒到 RTOS 节拍的换算；源码检查无超过 120 列、同行 `else` 或行尾空白；`git diff --check` 通过；`cmake --build build/Debug` 成功生成 `build/Debug/Infantry_Robot.elf`
- 状态：已验证，待用户确认创建本地 Git 快照

## GM6020-CURRENT-02

- 目标：将 GM6020 正反转测试电流由 0.05 A 调整为 0.1 A，方向持续时间保持 10 秒
- 源码改动：`User/task/motor_gimbal.c` 中 `MOTOR_GIMBAL_TEST_CURRENT_A` 由 `0.05F` 改为 `0.1F`，并同步更新集中说明
- 验证：正向和反向状态均引用 `MOTOR_GIMBAL_TEST_CURRENT_A` 的 `0.1F`；源码检查无超过 120 列、同行 `else` 或行尾空白；`git diff --check` 通过；`cmake --build build/Debug --verbose` 成功生成 `build/Debug/Infantry_Robot.elf`
- 状态：已验证，待用户确认创建本地 Git 快照

## GM6020-MONITOR-06

- 目标：精简 GM6020 Ozone 监视结构，保留当前速度环规划所需的命令、注册状态和物理反馈
- 源码改动：从 `MotorGM6020Monitor_t` 移除 `feedback_update_status`、`limited_current_a`、`applied_current_a`、`command_status`、`can_tx_status` 和 `command_sequence`，同步精简初始化、发布接口及集中注释
- 边界：设备层 CAN 快照中的同名诊断字段继续保留，供通信安全判断使用，不属于本次 Ozone 监视精简范围
- 验证：工程内无被移除的 GM6020 Ozone 成员引用；发布接口声明、定义和调用一致；源码规范与 `git diff --check` 检查通过；`cmake --build build/Debug --verbose` 成功生成 `build/Debug/Infantry_Robot.elf`
- 状态：已验证，待用户确认创建本地 Git 快照

## GM6020-SPEED-MODULE-07

- 目标：新增 GM6020 单路速度控制模块，与 3508 速度环复用同一套 PID 组件和计算顺序，仅使用独立参数与状态
- 源码改动：新增 `User/module/gimbal.c` 和 `User/module/gimbal.h`，实现目标斜坡、反馈滤波、`PID_Calc`、电流滤波、目标与电流限幅以及禁用和异常复位，并加入 CMake 源文件列表
- 初始参数：目标转速限幅 100 rpm、斜坡 100 rpm/s、Kp=0.01、Ki=0.005、Kd=0、积分限幅 50 rpm*s、电流限幅 1 A，均为待上板整定的保守初值
- 边界：本轮只建立并编译模块，尚未接入 `motor_gimbal` 任务，因此不会改变当前电机运行行为
- 验证：源码规范与 `git diff --check` 检查通过；ARM Debug 构建重新生成并编译 `gimbal.c`；主机侧边界测试覆盖禁用归零、正负速度方向、目标和电流限幅、反馈无效、非法目标及非法 PID 参数并输出 `Gimbal speed tests passed`
- 清理：临时测试源码 `build/gimbal_speed_test.c` 和可执行文件 `build/gimbal_speed_test.exe` 已删除
- 状态：已验证，待用户确认创建本地 Git 快照

## GM6020-FILTER-OFF-001

- 目标：关闭 GM6020 速度反馈低通滤波器和 PID D 项低通滤波器
- 源码改动：修改 `User/module/gimbal.h` 中的 `GIMBAL_PID_D_CUTOFF_HZ` 与 `GIMBAL_FEEDBACK_LPF_CUTOFF_HZ` 为 `-1.0F`，电流指令滤波保持 `-1.0F`
- 验证：三个截止频率均为非正值，滤波器进入直通分支；`git diff --check` 和 `cmake --build build/Debug --parallel 4` 均通过
- 风险：反馈量化噪声将直接进入控制链，需要上板观察转速和电流指令
- 状态：已按用户确认修改并完成构建验证，待本地 Git 快照

## GM6020-SPEED-INTEGRATION-08

- 目标：移除固定正反转电流测试，将 `motor_gimbal` 改为由 Ozone 目标转速驱动的 GM6020 速度闭环任务
- 任务改动：删除正反转状态枚举、10 秒定时器和固定电流限幅函数；任务在 500 Hz 读取速度目标与独立 PID 参数，使用 CAN 反馈调用 `Gimbal_Run`，再通过原有独立邮箱发布电流命令
- 调试改动：新增 `g_motor_gm6020_tune` 保存速度使能、目标转速和 PID 参数；`g_motor_gm6020_monitor` 改为发布速度环状态、目标、滤波反馈、误差、电流指令及设备物理反馈，并同步更新集中注释
- 安全默认：速度控制默认关闭、目标转速默认 0 rpm；禁用、反馈缺失、设备离线、输入非法或模块异常时持续发布零电流
- 验证：源码格式和旧正反转引用扫描通过；`git diff --check` 通过；ARM Debug 构建成功，RAM 占用 41672 B / 128 KB，FLASH 占用 87364 B / 1 MB；主机集成测试覆盖默认禁用、目标与 PID 参数映射、正负速度方向和监视区发布，并输出 `Gimbal integration tests passed`
- 清理：临时测试源码 `build/gimbal_integration_test.c` 和可执行文件 `build/gimbal_integration_test.exe` 已删除
- 状态：已验证，用户已确认创建本地 Git 快照

## GM6020-SPEED-NO-RAMP-001

- 目标：移除 GM6020 速度环目标转速斜坡，使 PID 每周期直接使用限幅后的目标转速
- 源码改动：删除 `Gimbal_ApplyRamp()`、斜坡参数及其配置校验；保留 `ramped_target_speed_rpm` 兼容字段，但其值直接等于限幅目标；同步更新 Ozone 字段说明
- 边界：不修改 IMU 反馈、GM6020 电流模式控制帧 `0x2FE`、滤波参数、电流限幅和 Ozone 数据布局
- 验证：`Gimbal_Run()` 中目标转速直接赋值；工程内 `gimbal.c` 和 `gimbal.h` 不再引用 GM6020 斜坡参数或函数；`git diff --check` 通过；`cmake --build build/Debug --parallel 4` 成功生成 `build/Debug/Infantry_Robot.elf`
- 状态：已验证，待本地 Git 快照

## CHASSIS-PROGRESS-DOC-01

- 日期：2026-09-15
- 目标：保存当前底盘进度，并要求重新开发底盘前先阅读进度
- 文档改动：在根目录 `agent.md` 顶部新增必读规则、当前阶段、已实现链路、参数与轮序、验证证据边界、未完成项及后续开发顺序
- 核心结论：四轮运动学与速度闭环已接入，当前输入来自 Ozone；DR16 状态邮箱尚无运动命令消费者，整车使能与故障恢复策略仍需补齐
- 验证：重新阅读 `AGENTS.md` 与 `agent.md`，人工核对源码和历史记录；`git diff --check -- agent.md` 通过，原有项目规则保留
- 边界：本轮仅修改文档，未修改源码，未运行构建或上板测试，未创建临时测试程序
- 状态：文档已写入并完成检查

## OZONE-DISPLAY-TRIM-01

- 日期：2026-09-15
- 目标：精简底盘与 GM6020 的重复展示，将低频诊断收进子结构体，保留日常控制量
- 源码范围：`User/task/ozone_debug.h`、`User/task/ozone_debug.c`
- GM6020：监视区目标合并为 `target_speed_rad_s`，取模块实际控制目标，禁用与控制异常时为零；移除请求、限幅、兼容斜坡和直通滤波反馈的重复展示
- GM6020 诊断：初始化状态、注册结果、单圈角度、原始电流和温度迁入 `diagnostics`；保留顶层实际角速度、误差、指令电流、反馈电流、使能、在线和控制状态
- 底盘：移除重复的限幅目标与外露零电流计数，保留解算目标、斜坡目标及内部五周期停机确认；初始化、通信、温度迁入 `diagnostics`
- 数据有效性：底盘新增 `diagnostics.feedback_available`，缺少 CAN 快照时标明设备量为保留值；GM6020 监视初始化改为尚未实际使能，调参使能与 10 rpm 目标维持原样，并修正对应注释
- 边界：DR16 展示、故障模块、PID、任务通信和控制输入函数未调整；修改过的分支仅用于快照判空、数值校验和停机确认计数边界，不新增业务模式
- 验证：逐文件重读规则，中文集中注释、Allman、120 列和旧字段引用检查通过；`git diff --check` 通过；`cmake --build --preset Debug --parallel 4` 成功；`python .auto-embedded/scripts/check.py` 的 ARCH/HW/SPEC 全部通过
- 构建占用：RAM 41656 B / 128 KB，FLASH 87172 B / 1 MB
- 文档：同步 `agent.md` 的 Ozone 查看路径和验证边界；旧监视表达式与曲线需按新 ELF 的字段路径调整
- 清理：本轮未创建临时测试源码或测试可执行文件；未烧录和上板验证
- 状态：实现和软件验证完成，用户已确认纳入本地 Git 快照

## gm6020-rad-loop-20260823-04

- 目标：将 GM6020 速度环及 Ozone 监视字段统一为 `rad/s`，并采用参考工程的速度 PID 初值
- 源码改动：CAN 反馈通过 `2π/60` 从协议 `rpm` 转换为 `rad/s`；GM6020 模块、任务和 Ozone 链路改用 `target_speed_rad_s`、`actual_speed_rad_s` 等字段；速度 PID 初值设为 `Kp=0.03`、`Ki=0.3`、`Kd=0`；10 rpm 目标换算为 `1.0471976 rad/s`
- 方向核对：GM6020 使用 CAN ID `0x209`，对应参考工程 Yaw，`.reverse = false` 与参考工程一致，发送电流和反馈符号保持同向
- 边界：GM6020 控制帧继续使用电流模式 `0x2FE`；速度斜坡保持关闭，兼容字段直接等于限幅目标；滤波截止频率保持非正值以关闭滤波；底盘速度字段继续使用 `rpm`
- 验证：旧 GM6020 速度环字段仅保留设备层原始诊断量；`git diff --check` 通过；`cmake --build build/Debug --parallel 4` 成功生成 `build/Debug/Infantry_Robot.elf`
- 状态：已验证，用户已确认纳入本地 Git 快照

## LOCAL-GIT-SNAPSHOT-01

- 目标：按用户要求将当前已验证改动保存到本地 Git
- 范围：GM6020 角速度单位与参数调整、Ozone 展示精简、`agent.md` 底盘进度和本日志，共 7 个已跟踪文件
- 验证依据：本轮核对差异，`git diff --check` 通过；沿用上一轮同版源码的 Debug 构建和 ARCH/HW/SPEC 通过结果
- 提交方式：逐个指定文件暂存，仅创建本地提交；未跟踪的 `.obsidian/` 不纳入，不推送远端

## chassis-direction-ozone-001

- 目标：增加 Ozone 单变量底盘方向测试，并将底盘常规监视数据收敛为在线状态、温度、速度环状态和目标/反馈转速
- 源码改动：`User/task/ozone_debug.h` 增加 `MotorChassisDirection_t`、`MotorChassisSpeed_t` 和方向转换接口；`User/task/ozone_debug.c` 实现前进、后退、左移、右移方向向量生成并接入底盘输入，同时移除底盘详细诊断字段和停机确认监视字段
- 数据路径：`direction=MANUAL` 时保留 `vx/vy/wz`，其他方向由单变量覆盖运动向量；目标转速和反馈转速统一发布到 `g_motor_chassis_monitor.speed`
- 边界：未修改 `User/module`、`User/device`、CAN 协议、速度 PID 或底盘逆运动学；详细故障信息继续由独立故障检测模块和 CAN 快照提供
- 验证：全仓旧字段引用检查通过；`git diff --check` 通过；`cmake --build --preset Debug` 成功，RAM 41592 B / 128 KB，FLASH 86828 B / 1 MB；`python .auto-embedded/scripts/check.py` 的 ARCH/HW/SPEC 全部通过
- 上板边界：方向变量、轮位符号和电机安装方向尚未上板实测，Ozone 需重新载入新 ELF 并更新监视表达式
- 状态：软件实现和构建验证完成，待用户确认创建本地 Git 快照

## CHASSIS-SINGLE-MOTOR-PID-001

- 目标：为底盘电机 ID4 单独保留速度环 PID 调试接口，调试完成后可持续使用独立参数
- 源码改动：`User/task/ozone_debug.h` 增加 `MotorChassisPidOverride_t` 结构体，集中保存 `pid_override_enable`、`pid_override_motor_index`、`pid_override_kp`、`pid_override_ki`、`pid_override_kd`；`User/task/ozone_debug.c` 默认选择 ID4 并将合法覆盖参数映射到对应输入路；`User/module/chassis.h/.c` 增加逐路 PID 覆盖快照和选择逻辑
- 使用方式：在 Ozone 中修改 `g_motor_chassis_tune.pid_override`，保持 `pid_override_motor_index=4`，调试时将 `pid_override_enable=true`；调好后保持该使能和参数即可继续使用
- 安全边界：覆盖默认关闭；电机 ID 仅接受 1~4，非法编号继续使用公共 PID；未覆盖的三路始终使用公共 PID
- 验证：`cmake --build --preset Debug --parallel 4` 成功，RAM 41608 B / 128 KB，FLASH 87140 B / 1 MB；`python .auto-embedded/scripts/check.py` 的 ARCH/HW/SPEC 全部通过；`git diff --check` 通过；ID4 映射为内部数组下标 3 的源码检查通过
- 上板边界：需重新加载新 ELF，并在低速、小幅目标下观察 ID4 的目标转速、实际转速和电流指令后再固化参数
- 状态：软件实现和验证完成，待创建本地 Git 快照

## CHASSIS-SINGLE-MOTOR-PID-FINAL-002

- 目标：固化底盘电机 ID4 的速度环参数，并删除调试完成后的独立 PID 临时接口
- 源码改动：`User/module/chassis.h/.c` 固化 ID4 `Kp=0.20`、`Ki=0.10`、`Kd=0`，每个控制周期按电机下标选择参数；`User/task/ozone_debug.h/.c` 删除 `MotorChassisPidOverride_t`、PID 在线调参字段和逐路覆盖映射，保留底盘使能、方向/运动量和轮速尺度接口
- 参数边界：ID1~3 继续使用公共 `Kp=0.23`、`Ki=0.14`、`Kd=0`；ID4 对应内部数组下标 3，始终使用固化参数
- 验证：`pid_override`、`pid_tune_override` 和底盘 Ozone 在线 PID 字段无残留引用；`cmake --build --preset Debug --parallel 4` 成功，RAM 41576 B / 128 KB，FLASH 86984 B / 1 MB；`python .auto-embedded/scripts/check.py` 的 ARCH/HW/SPEC 全部通过；`git diff --check` 通过
- 上板边界：需重新加载新 ELF，并在低速、小幅目标下确认 ID4 的速度响应和电流指令
- 状态：软件实现和验证完成，待创建本地 Git 快照

## CHASSIS-DR16-LINK-003

- 目标：将 DT7/DR16 遥控状态接入底盘控制任务，按左拨杆三态选择底盘运动模式
- 源码改动：`User/task/motor_chassis.c` 增加 DR16 最新状态邮箱消费和本地快照缓存；邮箱读到新状态时更新缓存，未读到新状态时继续使用最近快照；按 `last_online_time` 和 `100 ms` 超时保护生成安全使能
- 控制映射：左拨杆中位输出 `vx=0、vy=0、wz=0`；上位由左摇杆 `ch_l_y -> vx`、`ch_l_x -> vy`；下位由左摇杆 `ch_l_y -> vx`、`ch_l_x -> wz` 且显式强制 `vy=0`；下位横纵摇杆同时越过死区时丢弃 `vx` 和 `vy`，只保留 `wz` 自旋；右摇杆和右拨杆暂不参与控制并保留原始接口
- 输入处理：DR16 通道按已减中心值除以 660 归一化，执行 5% 中心死区和限幅；非法拨杆、未收到合法帧、状态离线或状态超时均保持底盘禁用和零运动量
- 验证：`cmake --build --preset Debug --parallel 4` 成功，RAM 41632 B / 128 KB，FLASH 87612 B / 1 MB；`python .auto-embedded/scripts/check.py` 的 ARCH/HW/SPEC 全部通过；`git diff --check` 通过；源码行宽和 Allman 花括号检查通过
- 上板边界：需确认 DT7 左摇杆前后和左右的实际正方向，若方向相反只调整任务层通道符号；需低速验证拨杆中位停机、上位平动、下位旋转平动和失联停机
- 状态：软件实现和验证完成，待创建本地 Git 快照

## CHASSIS-DR16-RETURN-CENTER-005

- 目标：左摇杆回中后关闭底盘速度环，避免零速目标下部分电机持续抖动
- 源码改动：`User/task/motor_chassis.c` 在 DR16 模式映射完成后，根据经过死区处理的 `vx、vy、wz` 是否全为零决定 `input->enabled`；回中时进入底盘已有禁用路径，清除目标斜坡、PID、反馈滤波和电流输出
- 行为边界：左拨杆上位或下位仍只负责选择运动模式；有任一有效运动分量时速度环使能；摇杆回中或拨杆中位时速度环关闭
- 验证：`cmake --build --preset Debug --parallel 4` 成功，RAM 41632 B / 128 KB，FLASH 87664 B / 1 MB；`python .auto-embedded/scripts/check.py` 的 ARCH/HW/SPEC 全部通过；`git diff --check` 和源码行宽检查通过
- 上板边界：需观察摇杆回中后的电流指令是否立即归零，以及重新推动摇杆时速度环是否平滑恢复
- 状态：软件实现和验证完成，待创建本地 Git 快照

## CHASSIS-DR16-SPIN-TRANSLATE-006

- 日期：2026-09-22
- 目标：左拨杆下位时立即开始固定自旋，并允许底盘同时执行前后与左右平动
- 源码改动：`User/task/motor_chassis.c` 增加固定自旋参数 `MOTOR_CHASSIS_DR16_SPIN_WZ=+0.30F`；下位映射恢复 `ch_l_y -> vx`、`ch_l_x -> vy`，并固定输出 `wz`；删除斜向输入时丢弃平动的限制
- 行为边界：下位且摇杆回中时仍因固定 `wz` 使能速度环；拨杆中位、DR16 未初始化、离线或超时继续输出零运动量并关闭速度环；自旋正方向为俯视逆时针
- 验证：`cmake --build --preset Debug --parallel 4` 成功，RAM 41632 B / 128 KB，FLASH 87636 B / 1 MB；`python .auto-embedded/scripts/check.py` 的 ARCH/HW/SPEC 全部通过；`git diff --check` 通过；源码行宽和 Allman 花括号检查通过
- 上板边界：需低速确认固定自旋方向与速度是否符合机械预期，再根据实际方向调整 `MOTOR_CHASSIS_DR16_SPIN_WZ` 的正负号或幅值
- 状态：软件实现和构建验证完成，待创建本地 Git 快照

## CHASSIS-DR16-ENABLE-ONLY-007

- 日期：2026-09-22
- 目标：删除底盘旧 Ozone 运动使能链路，保留遥控器作为唯一运动使能及运动向量来源
- 源码：仅修改 `User/task/motor_chassis.c`、`ozone_debug.c` 和 `ozone_debug.h`；删除 `motor_debug_enable`、方向枚举、方向转换函数、手动运动量和旧输入生成函数；沿用现有调参结构体，仅保留 `scale_rpm`
- 链路：任务初始化默认禁用的控制输入、固定 PID 参数及周期，再由 DR16 映射唯一生成使能和运动向量；Ozone 仅提供轮速尺度并接收监视数据
- 行为：保留中位停机、上位回中停机、下位回中持续自旋，以及未收到状态、离线、超时和非法拨杆禁用；GM6020 独立调试控制保持原样
- 验证：旧入口源码引用扫描无残留；新增代码人工检查与行宽、花括号、行尾空白检查通过；构建技能脚本使用 Debug 预设成功配置和构建；ARCH/HW/SPEC 全部通过；`git diff --check` 通过
- 构建产物：`F:/RM/Infantry_Robot/build/Debug/Infantry_Robot.elf`，构建目录 `build/Debug`，生成器 Ninja
- 文档：同步 `agent.md` 的当前数据流、已实现能力、未完成项与后续验证顺序，保留原有协作规则修改
- 边界：未烧录或上板测试；Ozone 需重新加载 ELF，移除旧底盘使能和运动量监视表达式；未创建临时测试程序
- 状态：软件实现和验证完成，按本地提交约定保存快照

## CHASSIS-DR16-MAP-REVERSE-009

- trace_id：`rc-chassis-map-20261006`
- 目标：修正 DR16 左摇杆上下和左右方向均反向的问题
- 源码改动：在 `User/task/motor_chassis.c` 中将 `ch_l_y -> vx` 与 `ch_l_x -> vy` 的两个输入分量分别取反，上位和下位模式保持一致，自旋角速度不变
- 验证标准：静态检查确认两个模式均使用负号映射，`git diff --check` 通过，Debug 构建通过
- 停止条件：不修改 DR16 解码、底盘混合器、电机驱动和 CubeMX 配置
- 验证：`git diff --check` 通过；`cmake --build --preset Debug --parallel 4` 通过并生成 `build/Debug/Infantry_Robot.elf`；`python .auto-embedded/scripts/check.py` 的 ARCH/HW/SPEC 全部通过
- 上板边界：尚未烧录和实车验证；需重新加载 ELF 后分别确认前进、后退、左移、右移方向
- 状态：软件修改与构建验证完成，等待本地 Git 快照

## OZONE-CHASSIS-PID-20261007

- trace_id：`ozone-pid-002`
- 目标：重新暴露底盘四个 M3508 速度环的 Ozone 在线调参接口，保留 ID1~3 公共参数与指定单电机独立参数
- 源码改动：`User/task/ozone_debug.h/.c` 增加 `MotorChassisPidOverride_t` 和公共 PID 参数；`User/task/motor_chassis.c` 将 Ozone 参数映射到每路 `Chassis_Input_t`；`User/module/chassis.h/.c` 增加每路 PID 覆盖输入并在控制计算时选择覆盖参数
- 默认行为：独立覆盖关闭，默认选择 ID4，ID4 继续使用固化参数；开启覆盖后仅指定电机使用 Ozone 独立参数，其他电机使用公共参数
- 验证：`git diff --check` 通过；`cmake --build --preset Debug --parallel 4` 通过并生成 `build/Debug/Infantry_Robot.elf`；`python .auto-embedded/scripts/check.py` 的 ARCH/HW/SPEC 全部通过
- 上板边界：需要重新加载 ELF，并在低速小目标下分别核对 ID1~3 公共参数和 ID4 独立参数的速度响应与电流指令
- 状态：软件实现与静态验证完成，待 Ozone 重新加载及上板调参

## CHASSIS-DR16-MONITOR-008

- 日期：2026-09-22
- 目标：删除已不再需要的 `DR16_Monitor_t` 接收诊断监视结构及其 Ozone 全局变量
- 源码改动：`User/task/ozone_debug.h/.c` 删除 DR16 诊断结构、全局监视变量和对应头文件依赖；`User/task/dr16_task.c` 删除接收统计、UART 错误、线程等待、邮箱和重启计数的维护逻辑，并保留 DR16 状态邮箱发布
- 安全边界：`DR16_State_t` 接收状态、100 ms 离线判断和底层 `FaultDetect_UpdateDR16` 故障快照继续保留，避免影响底盘失联停机链路；GM6020 和底盘监视数据不变
- 验证：全仓 `DR16_Monitor_t`、`g_dr16_monitor` 和旧监视更新函数引用无残留；Debug 构建成功生成 `build/Debug/Infantry_Robot.elf`；`python .auto-embedded/scripts/check.py` 的 ARCH/HW/SPEC 全部通过；`git diff --check` 通过
- 边界：未烧录和上板；Ozone 重新加载 ELF 后不再使用旧 DR16 诊断监视字段；未创建临时测试程序
- 状态：软件实现和验证完成，待创建本地 Git 快照

## CHASSIS-DR16-LATERAL-SIGN-20261007

- trace_id：`RC-STEER-20261007-03`
- 目标：修正 DR16 左摇杆左右输入映射到底盘横向运动方向相反的问题
- 源码改动：`User/task/motor_chassis.c` 将上位和下位模式的 `ch_l_x -> vy` 从负号映射改为正号，保持前后通道、自旋角速度和失联保护不变
- 验证标准：静态检查确认两种模式均使用正号横向映射，`git diff --check` 通过，Debug 构建通过
- 停止条件：不修改 DR16 解码、底盘混合器、电机驱动和 CubeMX 配置
- 验证：`git diff --check` 通过；`cmake --build --preset Debug --parallel 4` 通过并生成 `build/Debug/Infantry_Robot.elf`，RAM 41904 B、FLASH 88184 B；`python .auto-embedded/scripts/check.py` 的 ARCH/HW/SPEC 全部通过
- 上板边界：重新加载 ELF 后低速分别确认左移、右移及下位自旋叠加平动方向
- 状态：软件修改与构建验证完成，等待本地 Git 快照

## CHASSIS-RAMP-200RPM-20261008

- trace_id：`chassis-ramp-200rpm-20261008`
- 目标：将底盘目标转速斜坡由 150 rpm/s 调整为 200 rpm/s
- 源码改动：`User/module/chassis.h` 将 `CHASSIS_RAMP_RATE_RPM_S` 修改为 `200.0F`
- 行为边界：不修改换向零速门控、速度 PID、反馈滤波、电流限幅、遥控映射和 CubeMX 配置
- 验证：参数引用保持统一；`git diff --check` 通过；Debug 构建通过并生成
  `build/Debug/Infantry_Robot.elf`；`python .auto-embedded/scripts/check.py` 的 ARCH/HW/SPEC 全部通过
- 上板边界：需架空复测快速换向，再触地确认响应和电流冲击
- 状态：软件验证完成，等待用户确认创建本地 Git 快照

## CHASSIS-PID-P1-20261008

- trace_id：`chassis-pid-p1-20261008`
- 目标：将四个 M3508 的速度环参数统一调整为 `Kp=1`、`Ki=0`、`Kd=0`
- 源码改动：`User/module/chassis.h` 同步修改 ID1~3 公共参数和 ID4 固化参数
- Ozone 行为：公共 PID 和默认 ID4 独立覆盖初值均引用参数宏，重新加载新 ELF 后显示新参数
- 行为边界：不修改 PID 算法、目标斜坡、反馈滤波、电流限幅、遥控映射和 CubeMX 配置
- 验证：四路默认参数均为 `Kp=1`、`Ki=0`、`Kd=0`，Ozone 公共参数和 ID4 默认覆盖初值均引用对应宏；
  `git diff --check` 通过；Debug 构建通过并生成 `build/Debug/Infantry_Robot.elf`；
  `python .auto-embedded/scripts/check.py` 的 ARCH/HW/SPEC 全部通过
- 上板边界：`Kp=1 A/rpm` 在约 4 rpm 误差时触及 4 A 电流限幅，需架空并以低目标转速首次测试
- 状态：软件验证完成，等待用户确认创建本地 Git 快照

## CHASSIS-PID-P08-20261008

- trace_id：`chassis-pid-p08-20261008`
- 目标：将四个 M3508 速度环的 `Kp` 从 `1.0` 调整为 `0.8`，保持 `Ki=0`、`Kd=0`
- 源码改动：`User/module/chassis.h` 同步修改 ID1~3 公共 `Kp` 和 ID4 固化 `Kp`
- Ozone 行为：公共 PID 和默认 ID4 独立覆盖初值均引用参数宏，重新加载新 ELF 后显示 `Kp=0.8`
- 行为边界：不修改 PID 算法、目标斜坡、反馈滤波、电流限幅、遥控映射和 CubeMX 配置
- 验证：四路默认参数均为 `Kp=0.8`、`Ki=0`、`Kd=0`，Ozone 公共参数和 ID4 默认覆盖初值均引用对应宏；
  `git diff --check` 通过；Debug 构建通过并生成 `build/Debug/Infantry_Robot.elf`；
  `python .auto-embedded/scripts/check.py` 的 ARCH/HW/SPEC 全部通过
- 上板边界：`Kp=0.8 A/rpm` 在约 5 rpm 误差时触及 4 A 电流限幅，需架空并以低目标转速测试
- 状态：软件验证完成，等待用户确认创建本地 Git 快照

## CHASSIS-PID-P05-20261008

- trace_id：`chassis-pid-p05-20261008`
- 目标：将四个 M3508 速度环的 `Kp` 从 `0.8` 调整为 `0.5`，保持 `Ki=0`、`Kd=0`
- 源码改动：`User/module/chassis.h` 同步修改 ID1~3 公共 `Kp` 和 ID4 固化 `Kp`
- Ozone 行为：公共 PID 和默认 ID4 独立覆盖初值均引用参数宏，重新加载新 ELF 后显示 `Kp=0.5`
- 行为边界：不修改 PID 算法、目标斜坡、反馈滤波、电流限幅、遥控映射和 CubeMX 配置
- 验证：四路默认参数均为 `Kp=0.5`、`Ki=0`、`Kd=0`，Ozone 公共参数和 ID4 默认覆盖初值均引用对应宏；
  `git diff --check` 通过；Debug 构建通过并生成 `build/Debug/Infantry_Robot.elf`；
  `python .auto-embedded/scripts/check.py` 的 ARCH/HW/SPEC 全部通过
- 上板边界：`Kp=0.5 A/rpm` 在约 8 rpm 误差时触及 4 A 电流限幅，需架空并以低目标转速测试
- 状态：软件验证完成，等待用户确认创建本地 Git 快照

## CHASSIS-PID-P03-20261008

- trace_id：`chassis-pid-p03-20261008`
- 目标：将四个 M3508 速度环的 `Kp` 从 `0.5` 调整为 `0.3`，保持 `Ki=0`、`Kd=0`
- 源码改动：`User/module/chassis.h` 同步修改 ID1~3 公共 `Kp` 和 ID4 固化 `Kp`
- Ozone 行为：公共 PID 和默认 ID4 独立覆盖初值均引用参数宏，重新加载新 ELF 后显示 `Kp=0.3`
- 行为边界：不修改 PID 算法、目标斜坡、反馈滤波、电流限幅、遥控映射和 CubeMX 配置
- 验证：四路默认参数均为 `Kp=0.3`、`Ki=0`、`Kd=0`，Ozone 公共参数和 ID4 默认覆盖初值均引用对应宏；
  `git diff --check` 通过；Debug 构建通过并生成 `build/Debug/Infantry_Robot.elf`；
  `python .auto-embedded/scripts/check.py` 的 ARCH/HW/SPEC 全部通过
- 上板边界：`Kp=0.3 A/rpm` 在约 13.3 rpm 误差时触及 4 A 电流限幅，需架空并以低目标转速测试
- 状态：软件验证完成，等待用户确认创建本地 Git 快照

## CHASSIS-PID-P025-20261008

- trace_id：`chassis-pid-p025-20261008`
- 目标：将四个 M3508 速度环的 `Kp` 从 `0.3` 调整为 `0.25`，保持 `Ki=0`、`Kd=0`
- 源码改动：`User/module/chassis.h` 同步修改 ID1~3 公共 `Kp` 和 ID4 固化 `Kp`
- Ozone 行为：公共 PID 和默认 ID4 独立覆盖初值均引用参数宏，重新加载新 ELF 后显示 `Kp=0.25`
- 行为边界：不修改 PID 算法、目标斜坡、反馈滤波、电流限幅、遥控映射和 CubeMX 配置
- 验证：四路默认参数均为 `Kp=0.25`、`Ki=0`、`Kd=0`，Ozone 公共参数和 ID4 默认覆盖初值均引用对应宏；
  `git diff --check` 通过；Debug 构建通过并生成 `build/Debug/Infantry_Robot.elf`；
  `python .auto-embedded/scripts/check.py` 的 ARCH/HW/SPEC 全部通过
- 上板边界：`Kp=0.25 A/rpm` 在约 16 rpm 误差时触及 4 A 电流限幅，需架空并以低目标转速测试
- 状态：软件验证完成，等待用户确认创建本地 Git 快照

## CHASSIS-PID-P020-20261008

- trace_id：`chassis-pid-p020-20261008`
- 目标：将四个 M3508 速度环的 `Kp` 从 `0.25` 调整为 `0.20`，保持 `Ki=0`、`Kd=0`
- 源码改动：`User/module/chassis.h` 同步修改 ID1~3 公共 `Kp` 和 ID4 固化 `Kp`
- Ozone 行为：公共 PID 和默认 ID4 独立覆盖初值均引用参数宏，重新加载新 ELF 后显示 `Kp=0.20`
- 行为边界：不修改 PID 算法、目标斜坡、反馈滤波、电流限幅、遥控映射和 CubeMX 配置
- 验证：四路默认参数均为 `Kp=0.20`、`Ki=0`、`Kd=0`，Ozone 公共参数和 ID4 默认覆盖初值均引用对应宏；
  `git diff --check` 通过；Debug 构建通过并生成 `build/Debug/Infantry_Robot.elf`；
  `python .auto-embedded/scripts/check.py` 的 ARCH/HW/SPEC 全部通过
- 上板边界：`Kp=0.20 A/rpm` 在约 20 rpm 误差时触及 4 A 电流限幅，需架空并以低目标转速测试
- 状态：软件验证完成，等待用户确认创建本地 Git 快照

## INS-HARDWARE-CONFIG-20261009

- trace_id：`ins-hardware-config-20261009`
- 目标：为 C 板 BMI088 和板载加热电阻准备完整 INS 移植所需的 SPI1 与 PWM 硬件配置
- CubeMX 改动：SPI1 从 328.125 Kbit/s 调整为 10.5 Mbit/s；新增 PF6/TIM10_CH1，按 84 MHz 定时器时钟配置 500 Hz PWM，初始比较值为 0
- 生成代码：新增 `Inc/tim.h` 和 `Src/tim.c`，在 `main()` 中初始化 TIM10，并将 `Src/tim.c` 加入 CMake 源文件列表
- 资源登记：硬件锁补充 BMI088 片选、SPI1、EXTI、DMA、PF6 和 TIM10_CH1 资源
- 生成文件：保留 CubeMX 更新的 `.mxproject` 和随 TIM10 复制的官方 `stm32f4xx_ll_tim.h`
- 验证：`git diff --check` 通过；Debug 构建成功，RAM 41976 B / 128 KB，FLASH 88 KB / 1 MB；ARCH/HW/SPEC 全部通过
- 边界：尚未新增 BMI088 设备驱动、INS 解算和 `INS_Task`，也未启动 PWM 或接入底盘、云台反馈
- 状态：本轮实现和软件验证完成，等待用户确认后创建本地提交

## INS-BSP-IMU-P2-20261009

- trace_id：`ins-bsp-imu-p2-20261009`
- 目标：新增 BMI088 所需的 L2 BSP 接口，隔离 SPI1、片选、延时和加热 PWM 硬件细节
- 源码改动：新增 `User/bsp/imu.h`、`User/bsp/imu.c`；根 `CMakeLists.txt` 登记 `User/bsp/imu.c`
- 接口行为：SPI HAL 状态映射为 `BSP_OK`、`BSP_ERR_TIMEOUT` 或 `BSP_ERR`；空接收指针返回 `BSP_ERR_NULL`；加速度计与陀螺仪片选均按低有效控制；PWM 占空比使用 0~1000 千分比，越界时先关闭 PWM
- 边界：本轮不实现 BMI088 寄存器协议、数据换算、EKF、INS 模块或 `INS_Task`，不接入底盘和云台闭环
- 验证：Allman/120 列/行尾扫描通过；`git diff --check` 通过；Debug 构建成功，RAM 41976 B / 128 KB，FLASH 88 KB / 1 MB；`python .auto-embedded/scripts/check.py` 的 ARCH/HW/SPEC 全部通过
- 状态：软件验证完成，等待用户确认后创建本地 Git 快照

## INS-BMI088-DEVICE-P3-20261009

- trace_id：`ins-bmi088-device-p3-20261009`
- 目标：完成 BMI088 L3 设备驱动，提供芯片识别、复位、寄存器配置校验、三轴数据读取和状态诊断
- 源码改动：新增 `User/device/bmi088.h`、`User/device/bmi088.c`；根 `CMakeLists.txt` 登记 `User/device/bmi088.c`
- 协议行为：加速度计 SPI 读取使用读命令、dummy 和数据阶段；陀螺仪使用读命令和数据阶段；加速度量程为 6G、陀螺仪量程为 2000dps、加速度计配置为 800Hz
- 来源边界：寄存器定义和初始化配置序列取自 `F:\RM\RoboMaster-C-Board-INS-Example-master\Components\Devices\BMI088reg.h`、`BMI088driver.h` 与 `BMI088driver.c`；本次未收到或查阅 BMI088 数据手册，重力常数 `9.80665F` 为本次实现选用
- 数据行为：输出加速度单位 m/s²、角速度单位 rad/s、温度单位摄氏度；任一读取事务失败时清零输出并将 `valid` 置为 false
- 错误处理：芯片 ID、寄存器写回和 SPI/延时失败均保留 `BMI088_Status_t`；避免参考工程陀螺仪错误表索引和 `uint8_t` 索引下溢
- 边界：本轮不实现 Kalman 矩阵、Quaternion EKF、INS 模块或 `INS_Task`，不接入底盘和云台闭环
- 验证：参考寄存器序列对照完成；Allman/120 列/行尾扫描通过；`git diff --check` 通过；Debug 构建成功，RAM 41976 B / 128 KB，FLASH 88 KB / 1 MB；`python .auto-embedded/scripts/check.py` 的 ARCH/HW/SPEC 全部通过
- 状态：本轮软件验证完成，直接创建本地 Git 快照，不推送远端

## INS-BMI088-HEADER-MERGE-P4-20261009

- trace_id：`ins-bmi088-header-merge-p4-20261009`
- 目标：移除仅供 BMI088 驱动使用的独立寄存器头文件，统一由设备头文件承载 BMI088 API 与寄存器定义
- 源码改动：将 `User/device/bmi088_reg.h` 中的寄存器、量程和换算宏合并至 `User/device/bmi088.h`；移除 `User/device/bmi088.c` 对独立寄存器头文件的包含并删除该文件
- 来源边界：合并的寄存器定义和换算值来自上述本机参考工程；不代表已依据 BMI088 数据手册核验
- 验证：全仓 `User` 和 CMake 范围未发现 `bmi088_reg.h` 残余引用；Allman/120 列/行尾扫描通过；`git diff --check` 通过；Debug 构建成功，RAM 41976 B / 128 KB，FLASH 88 KB / 1 MB；`python .auto-embedded/scripts/check.py` 的 ARCH/HW/SPEC 全部通过
- 状态：本轮验证完成，创建本地 Git 快照，不推送远端

## GITHUB-RELEASE-AND-README-20261009

- 目标：建立首个可识别的预发布版本并补充项目入口文档
- Release：已发布 `v0.1.0-alpha.1 - 底盘初步完善`，类型为 Pre-release，目标为 GitHub 远端 `main` 提交 `3b1dd3a3e3d8105a6b7a9b095886762cd54f8f83`
- 版本边界：Release 正文标注底盘控制功能初步完善，尚未接入 IMU 姿态反馈调节；当前本地 INS 改动未进入该版本
- 文档改动：新增根目录 `README.md`，说明硬件平台、当前功能、分层结构、CMake 构建方式和开发状态
- 规则改动：`AGENTS.md` 增加 Release 只能基于 GitHub 远端已存在提交、发布前后必须核验目标 SHA 和发布状态的约束
- 验证：GitHub API 已复核 Tag 目标、`prerelease=true` 和 `draft=false`；README 构建命令与 `CMakePresets.json` 一致；`git diff --check` 通过
- 状态：Release 已完成；README 与规则保留在本地工作树，尚未提交或推送

## CHASSIS-REVERSAL-SMOOTH-20261009

- trace_id：`chassis-reversal-smooth-20261009`
- 现象：底盘悬空快速前后换向时卡顿数次，落地基本无此现象
- 原因分析：摇杆过中心死区时速度环被关闭并清零斜坡；重新使能时轮子仍在转动，斜坡从 0 起步使纯 P 速度环直接输出 4 A 满幅制动，死区边缘抖动造成零电流滑行与满幅制动反复切换
- 源码改动：`User/task/motor_chassis.c` 中左拨杆上位和下位始终使能速度环，摇杆回中以零目标经斜坡主动制动，中位、异常拨杆和遥控离线仍关闭输出；`User/module/chassis.c` 的 `Chassis_PrimeFeedback()` 将斜坡目标预置为限幅后的当前滤波转速
- 授权：用户确认同时修改 `User/task` 与 `User/module`
- 验证：Allman/120 列/行尾扫描通过；`git diff --check` 通过；Debug 构建成功，RAM 41976 B / 128 KB，FLASH 90088 B / 1 MB；`python .auto-embedded/scripts/check.py` 的 ARCH/HW/SPEC 全部通过
- 待验证：上板悬空和落地快速换向、摇杆回中制动距离、拨中位停机零电流输出
- 状态：本轮软件验证完成，单独创建本地 Git 快照，不推送远端

## INS-KALMAN-MATRIX-P5-20261009

- trace_id：`ins-kalman-matrix-p5-20261009`
- 目标：新增 L4 静态 Kalman 矩阵组件，为后续 Quaternion EKF 提供预测、量测更新和阶段回调
- 源码改动：新增 `User/component/kalman_filter.h`、`User/component/kalman_filter.c`，矩阵与向量数据固定保存在对象内，默认维数上限为状态/控制/量测各 6，可通过宏覆盖；未使用堆分配
- 安全校验：拒绝非法初始化维数、非有限输入、矩阵维度不兼容和超出静态元素容量的矩阵视图；CMSIS 奇异逆矩阵映射为可查询组件状态
- 构建改动：根 `CMakeLists.txt` 登记 CMSIS-DSP 所需 MatrixFunctions、包含目录及 Cortex-M4F/FPU 编译定义
- 验证：临时主机测试覆盖一维更新 `x=1、P=0.5`、无有效量测预测、奇异逆矩阵、越界矩阵视图和 NaN 输入；ARM Debug 构建成功，RAM 41976 B / 128 KB，FLASH 90088 B / 1 MB；ARCH/HW/SPEC 全部通过；临时测试资产已清理
- 边界：未新增或适配 Quaternion EKF，未进行目标板数值实测；后续按新的静态状态数组接口适配
- 状态：软件验证完成，创建本地 Git 快照，不推送远端

## CHASSIS-MOTOR4-KP-20261009

- trace_id：`chassis-motor4-kp-20261009`
- 目标：按用户要求将 ID4 底盘电机速度环 Kp 由 0.20 减小 0.04
- 源码改动：`User/module/chassis.h` 中 `CHASSIS_MOTOR4_PID_KP` 由 `0.20F` 改为 `0.16F`；ID1～ID3 公共 Kp 保持 `0.20F`，Ozone 独立覆盖初值随宏同步为 0.16，开启覆盖时以覆盖值为准
- 授权：用户明确要求修改 ID4 速度环参数
- 验证：`git diff --check` 通过；Debug 构建成功，RAM 41976 B / 128 KB，FLASH 90104 B / 1 MB；`python .auto-embedded/scripts/check.py` 的 ARCH/HW/SPEC 全部通过；尚未上板验证
- 状态：单独创建本地 Git 快照，不推送远端
# IMU-XR-003 / IMU-XR-004：替换为 XRobot Madgwick（2026-10-19）

- 用户授权 module/component 与 CMake/Keil 修改，并选择 A：不新增结构体，复用公开姿态字段
- 固定来源：`xrobot-org/MadgwickAHRS`，`e98a223b84e44ca8718fe8e68b4b63577ec70499`，Apache-2.0
- 新增 `User/component/madgwick_ahrs.c/.h`，移植六轴梯度计算；补零梯度、数值校验及错误时原值保留
- `INS_t` 删除 EKF 实例，使用 `ahrs_status` 与 `ahrs_beta`；INS 内展开累计 Yaw，保持原 Pitch/Roll 命名映射
- 删除旧 Quaternion EKF 与其专属 Kalman 组件，移除矩阵源文件登记，CMake/Keil 接入新核心
- 保留 BMI088 BSP/Device、CubeMX、任务频率、温控及模块故障停热行为；未烧录或复位
- ARM Debug 全量构建 85 步通过，RAM 42,176 B，Flash 102,288 B；ARCH/HW/SPEC 与格式检查通过
- 主机回归包括上游 10,000 组单步与 20,000 步连续对照、水平静止、三轴旋转、变 dt、倾斜、32 例边界输入、累计 Yaw 与 10 例模块故障
- 说明与证据：`docs/imu_xrobot_port.md`、`docs/validation/imu_xrobot_20261019.json`
- 临时测试资产保存结果后清理；未执行 Git add/commit/push；实际漂移、轴向和运行耗时待上板确认

## INS-ROLL-PITCH-20261019

- 用户明确授权修改 `User/module/ins.c`、`ins.h`，修正 Roll/Pitch 互换
- 发布映射改为 `Roll=绕 X 轴横滚`、`Pitch=绕 Y 轴俯仰`，同步集中字段说明与移植记录
- 主机使用实际 INS/Madgwick/PID/Filter/user_math 代码验证 13 组单轴正反转与组合姿态，最大误差 0.000003815 度
- GCC C11 严格编译、ARM Debug 构建、ARCH/HW/SPEC、格式与行尾检查通过，RAM 42176 B、Flash 102288 B
- 验证证据与源码及 ELF 哈希见 `docs/imu_xrobot_port.md` 的映射修正验证节；临时测试资产已清理
- 未烧录、复位或上板实测；未执行 Git add/commit/push

## INS-LIVE-MEASURE-20261019

- 按用户要求复用已有 EmberProbe 连接实测 C 板 INS，未修改生产源码、硬件配置或固件参数，未烧录或主动复位
- beta 全程 0.033，静止基线 66.428 秒的 Roll/Pitch 标准差为 0.02468°/0.02548°，温度 39.875℃～40.125℃
- 分别采到 Roll 与 Pitch 正反转；动作后另采 43.258 秒，Roll/Pitch 标准差为 0.02303°/0.02136°
- 用户确认明显降温期间电源突然断开，相应数据独立标记；曾出现非用户主动的更新计数重新起算，未核实复位原因
- 在线 beta 写入预检查被工具拒绝，未执行写入或绕过；没有不同 beta 对照，不宣称获得最优参数或绝对角度精度
- 原始数据、统计、曲线与调参建议见 `docs/validation/imu_live_20261019.md`，移植文档已追加后续实测链接
- ARCH/HW/SPEC、Debug 构建与差异空白检查通过；复核本轮无 ISR、临界区或 RTOS 对象改动，临时统计程序已清理
- 保持用户原有后台采样运行，未执行 Git add/commit/push
