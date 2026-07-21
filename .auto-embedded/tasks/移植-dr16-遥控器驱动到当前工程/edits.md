# 编辑清单

| 文件 | 改动 | 验证标准 | 结果 | commit |
|---|---|---|---|---|
| `User/bsp/uart.c/.h` | 融合 receive-to-idle Circular DMA 字节流、普通 UART API 和 HAL 回调 | HT/TC/IDLE 连续接收，错误可恢复，CAN BSP 差异为 0 | DR16-UART-FUSED-02 主机与 ARM GCC 验证通过，用户已确认 | 本地快照 |
| `tests/uart_stream_test.c`、`tests/mocks/usart.h` | 使用 HAL mock 验证连续、重复、回绕、错误和重启事件 | 新字节不重不漏，错误后停止交付 | DR16-UART-FUSED-02 全部主机测试通过，用户已确认 | 本地快照 |
| `User/task/init.c`、`user_task.c/.h` | 检查 RTOS 创建结果并删除无用途运行时占位 | 创建失败不启动业务任务，运行时对象均有所有者 | DR16-TASK-03 主机失败路径与 ARM 构建通过，用户已确认 | 本地快照 |
| `User/task/motor_chassis.c/.h` | 修复安全默认值、格式、while 注释和初始化错误处理 | 上电零输出，task 专用规则扫描为 0 | E1 样式、clang-tidy 与 Debug clean build 通过，用户已确认 | 本地快照 |
| `User/module/motor_speed_control.c/.h` | 从 task 迁移速度闭环业务模块 | task 不再承载 PID/滤波/斜坡算法 | E1 分层扫描与 Debug clean build 通过，用户已确认 | 本地快照 |
| `User/task/motor_speed_control.c/.h` | 完成 module 迁移后删除旧路径文件 | 全仓无旧 include，构建通过 | E1 旧路径零引用，用户已确认 | 本地快照 |
| `User/module/chassis.c/.h` | 建立 Chassis 模块并组合四个单电机速度控制器 | 四路独立控制，单路异常隔离，全局禁用零输出 | CHASSIS-MODULE-06 主机测试、clang-tidy 与 Debug clean build 通过，用户已确认 | 本地快照 |
| `User/task/motor_chassis.c/.h` | 使用单个 Chassis 上下文和一致输入/输出快照 | task 不再直接持有或调用 MotorSpeedControl | CHASSIS-MODULE-06 直接引用为 0；CAN 设备 I/O 边界留待下一改动点，用户已确认 | 本地快照 |
| `tests/chassis_test.c` | 覆盖四控制器独立性、离线隔离、全局禁用和非法反馈 | Chassis 真实实现主机测试全部通过 | CHASSIS-MODULE-06 通过，用户已确认 | 本地快照 |
| `CMakeLists.txt` | 接入 Chassis 模块并集中 module 源文件分组 | Debug clean build 包含 chassis.c | CHASSIS-MODULE-06 通过，用户已确认 | 本地快照 |
| `User/module/chassis_can.c/.h` | 建立四个 M3508 的唯一 CAN 设备边界，统一注册、反馈、电流写槽和单次组发送 | 单路失败隔离，非法电流零覆盖，四路后只 Flush 一次 | CHASSIS-CAN-BOUNDARY-07 主机测试、clang-tidy 与 Debug clean build 通过，用户已确认 | 本地快照 |
| `User/task/motor_chassis.c` | 移除 CAN BSP、RM device、设备参数和实例，改为编排 ChassisCAN 与 Chassis 快照 | task 直接 BSP_CAN/MOTOR_RM 引用为 0，失败输出归零 | CHASSIS-CAN-BOUNDARY-07 分层扫描与回归测试通过，用户已确认 | 本地快照 |
| `tests/chassis_can_test.c`、`tests/mocks/bsp/can.h`、`tests/mocks/device/motor_rm.h` | 覆盖固定注册、注册/反馈隔离、四槽单发、失败零覆盖、非法值和初始化失败 | 主机测试使用真实 chassis_can.c 且全部断言通过 | CHASSIS-CAN-BOUNDARY-07 通过，用户已确认 | 本地快照 |
| `CMakeLists.txt` | 仅接入 `User/module/chassis_can.c`，不修改 Keil 配置 | Debug clean build 编译 chassis_can.c | CHASSIS-CAN-BOUNDARY-07 通过，用户已确认 | 本地快照 |
| `User/device/dr16.c/.h` | 改为显式 18 字节纯解码与校验 | 主机协议测试全部通过 | DR16-DECODE-01 编译与分层扫描通过，用户已确认 | 本地快照 |
| `tests/dr16_decode_test.c` | 覆盖合法帧、边界值、非法长度、非法通道、非法拨杆和状态复位 | 失败路径不污染旧输出 | DR16-DECODE-01 全部主机测试通过，用户已确认 | 本地快照 |
| `User/task/dr16_task.c/.h` | 新增 RingBuffer、重同步、离线与发布任务 | 错位恢复、100 ms 离线、邮箱快照通过 | DR16-TASK-03 主机测试、clang-tidy 与 ARM 构建通过，用户已确认 | 本地快照 |
| `User/task/user_task.c/.h`、`User/task/init.c` | 增加任务句柄、属性和最新状态邮箱 | 所有 RTOS 对象创建并检查成功 | DR16-TASK-03 创建与失败回滚测试通过，用户已确认 | 本地快照 |
| `User/device/device.h` | 移除旧 DR16 线程标志 | 设备层无任务通知定义 | DR16-SIGNAL-CLEANUP-04 零引用、主机测试与 Debug clean build 通过，用户已确认 | 本地快照 |
| `CMakeLists.txt` | 加入 DR16 链路所需源文件 | CMake clean build 通过，Keil 工程保持不变 | DR16-TASK-03 Debug clean build 通过，未修改 Keil，用户已确认 | 本地快照 |
| `tests/dr16_task_test.c`、`tests/mocks/cmsis_os2.h` | 覆盖逐字节重同步、100 ms 离线和两类错误恢复 | 实际任务代码主机测试全部通过 | DR16-TASK-03 通过，用户已确认 | 本地快照 |
| `tests/init_task_test.c` | 覆盖邮箱和任务创建成功、各创建失败与清理路径 | 所有 RTOS 对象创建结果均被检查 | DR16-TASK-03 通过，用户已确认 | 本地快照 |
| `build/Debug/Infantry_Robot.elf`、`.map`、`.su` | 软件终检、内存与静态栈帧分析 | 主机回归、clang-tidy、Debug clean build、check.py 全通过 | DR16-SOFTWARE-VERIFY-05：Flash 76312 B；普通 SRAM 含链接器预留 40668 B；DR16/底盘可见项目函数链约 260/416 B，板上 high-water mark 待测 | 本地快照 |
| `User/task/ozone_debug.c/.h`、`dr16_task.c/.h`、`motor_chassis.c/.h`、`tests/dr16_task_test.c`、`CMakeLists.txt` | 集中保存 DR16 与底盘的 Ozone 调试结构、全局实例和初值 | 调试结构与符号归属唯一，任务行为和构建结果不变 | OZONE-DEBUG-09 主机测试、Debug clean build、check.py 与 ELF 符号归属检查通过，用户已确认 | 本地快照 |
| `User/task/ozone_debug.c` | 上电默认开启四路 M3508 速度控制并将目标转速统一设为 100 rpm | 控制输入快照为 enabled=true，四路 requested_speed_rpm 均为 100.0F；主机回归、Debug clean build、ELF 初值核对和 check.py 通过 | MOTOR-SPEED-100-01 已验证 | 本地快照 |
| `User/task/init.c`、`tests/init_task_test.c` | 将底盘模块初始化移到调度器锁定前，避免 CAN 注册在锁定期间执行带超时的互斥锁等待 | 初始化顺序测试覆盖模块初始化早于 `osKernelLock()`，失败路径不创建业务对象；独立 ARM Debug 构建通过 | DR16-MOTOR-FIX-01 主机测试通过；FLASH 78728 B，RAM 40808 B | 本地快照 |
| `tests/` | 按用户确认删除暂不使用的主机测试目录及其 mock 文件 | `tests/` 目录不存在，正式固件构建配置不引用该目录 | TESTS-REMOVE-01 删除完成并提交 | `8eabc28` |
| `User/module/chassis.c/.h`、`motor_speed_control.h`、`User/task/motor_chassis.c/.h`、`ozone_debug.c/.h`、`user_task.h`、`CMakeLists.txt`、`AGENTS.md` | 将 `chassis_can`、`chassis_control` 合并到唯一底盘模块，保留单电机速度控制器，更新反馈滤波参数并规定 Git 提交使用中文 | 旧底盘接口零引用；任务仅调用 `Chassis_Init`、`Chassis_Run`；逐路故障隔离和统一发送语义保持；Debug clean build 通过 | CHASSIS-MERGE-02 通过；FLASH 77940 B，RAM 40784 B | 本地快照 |
| `User/device/can_devices.c/.h`、`User/module/chassis.c`、`CMakeLists.txt` | 新增整车 CAN 设备集合，将四个 M3508 的固定参数、注册、反馈更新、电流写入和统一发送迁出 Chassis | Chassis 不直接引用 BSP_CAN/MOTOR_RM；单路异常归零；四路仅统一发送一次 0x200 控制帧；Debug clean build 与门禁通过 | CAN-DEVICE-COLLECTION-01 主机测试、clang-tidy、Debug clean build 与 check.py 通过，用户已确认 | 本地快照 |
