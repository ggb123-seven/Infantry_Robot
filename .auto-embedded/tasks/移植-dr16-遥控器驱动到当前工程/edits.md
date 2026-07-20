# 编辑清单

| 文件 | 改动 | 验证标准 | 结果 | commit |
|---|---|---|---|---|
| `User/bsp/uart.c/.h` | 融合 receive-to-idle Circular DMA 字节流、普通 UART API 和 HAL 回调 | HT/TC/IDLE 连续接收，错误可恢复，CAN BSP 差异为 0 | DR16-UART-FUSED-02 主机与 ARM GCC 验证通过，用户已确认 | 本地快照 |
| `tests/uart_stream_test.c`、`tests/mocks/usart.h` | 使用 HAL mock 验证连续、重复、回绕、错误和重启事件 | 新字节不重不漏，错误后停止交付 | DR16-UART-FUSED-02 全部主机测试通过，用户已确认 | 本地快照 |
| `User/task/init.c`、`user_task.c/.h` | 检查 RTOS 创建结果并删除无用途运行时占位 | 创建失败不启动业务任务，运行时对象均有所有者 | DR16-TASK-03 主机失败路径与 ARM 构建通过，用户已确认 | 本地快照 |
| `User/task/motor_chassis.c/.h` | 修复安全默认值、格式、while 注释和初始化错误处理 | 上电零输出，task 专用规则扫描为 0 | E1 已实现、验证并经用户确认，本次提交建立本地快照 | - |
| `User/module/motor_speed_control.c/.h` | 从 task 迁移速度闭环业务模块 | task 不再承载 PID/滤波/斜坡算法 | E1 已实现、验证并经用户确认，本次提交建立本地快照 | - |
| `User/task/motor_speed_control.c/.h` | 完成 module 迁移后删除旧路径文件 | 全仓无旧 include，构建通过 | E1 已删除且旧路径零引用，经用户确认，本次提交建立本地快照 | - |
| `User/device/dr16.c/.h` | 改为显式 18 字节纯解码与校验 | 主机协议测试全部通过 | DR16-DECODE-01 编译与分层扫描通过，用户已确认 | 本地快照 |
| `tests/dr16_decode_test.c` | 覆盖合法帧、边界值、非法长度、非法通道、非法拨杆和状态复位 | 失败路径不污染旧输出 | DR16-DECODE-01 全部主机测试通过，用户已确认 | 本地快照 |
| `User/task/dr16_task.c/.h` | 新增 RingBuffer、重同步、离线与发布任务 | 错位恢复、100 ms 离线、邮箱快照通过 | DR16-TASK-03 主机测试、clang-tidy 与 ARM 构建通过，用户已确认 | 本地快照 |
| `User/task/user_task.c/.h`、`User/task/init.c` | 增加任务句柄、属性和最新状态邮箱 | 所有 RTOS 对象创建并检查成功 | DR16-TASK-03 创建与失败回滚测试通过，用户已确认 | 本地快照 |
| `User/device/device.h` | 移除旧 DR16 线程标志 | 设备层无任务通知定义 | 待授权执行 | - |
| `CMakeLists.txt` | 加入 DR16 链路所需源文件 | CMake clean build 通过，Keil 工程保持不变 | DR16-TASK-03 Debug clean build 通过，未修改 Keil，用户已确认 | 本地快照 |
| `tests/dr16_task_test.c`、`tests/mocks/cmsis_os2.h` | 覆盖逐字节重同步、100 ms 离线和两类错误恢复 | 实际任务代码主机测试全部通过 | DR16-TASK-03 通过，用户已确认 | 本地快照 |
| `tests/init_task_test.c` | 覆盖邮箱和任务创建成功、各创建失败与清理路径 | 所有 RTOS 对象创建结果均被检查 | DR16-TASK-03 通过，用户已确认 | 本地快照 |
