# 研究发现

| 关键词 | 来源 | 摘要 | 可信度 | 状态 |
|---|---|---|---|---|
| 硬件基线 | `Infantry_Robot.ioc:15-26,213-218`、`spec/hardware/hw-lock.yaml` | USART3 已配置为 100000 baud、9-bit word length、Even parity、RX-only；USART3_RX 使用 DMA1 Stream1 Channel 4、Circular、High priority；DMA 与 USART3 IRQ 均为 5/0。 | 高 | 已验证 |
| 生成代码链路 | `Src/usart.c`、`Src/dma.c`、`Src/stm32f4xx_it.c:171-205` | DMA、USART3、DMA IRQ 和 USART3 IRQ 已生成；IRQ 分别进入 `HAL_DMA_IRQHandler()` 与 `HAL_UART_IRQHandler()`，本任务不需要修改生成 ISR。 | 高 | 已验证 |
| HAL 能力 | `stm32f4xx_hal_uart.h:758-780`、`stm32f4xx_hal_uart.c:1781-1861,3060-3170` | 当前 HAL 提供 `HAL_UARTEx_ReceiveToIdle_DMA()`、`HAL_UARTEx_RxEventCallback()` 和事件类型查询；Circular DMA 下 HT、TC、IDLE 均不会停止接收。 | 高 | 已验证 |
| 待迁移文件状态 | `git status --short`、`CMakeLists.txt`、`MDK-ARM/Infantry_Robot.uvprojx` | `User/bsp/uart.*` 与 `User/device/dr16.*` 是未跟踪文件，当前 CMake/Keil 均未纳入这些业务源文件；CMake 已包含生成的 USART/DMA/HAL UART 源。 | 高 | 已验证 |
| 旧接收模型冲突 | `User/device/dr16.c:75-106`、`User/bsp/uart.c` | 旧实现每次启动固定长度 DMA 并等待完成标志，设备层直接调用 HAL；这与当前 Circular DMA + RingBuffer 的冻结方案及分层约束不一致。 | 高 | 待重构 |
| 原始帧布局 | ARM GCC 静态断言探针、`User/device/dr16.h` | 当前 ARM GCC 下 `sizeof(DR16_RawData_t) == 18`，但协议解析依赖 packed bitfield 的编译器布局；迁移后应改为对 18 字节数组显式移位解码。 | 高 | 待重构 |
| 协议与安全要求 | `全向轮底盘学习章节.md:1142-1410` | DR16 帧长 18 字节，四通道典型原始范围 364~1684、中心 1024；非法帧不得污染输出，100 ms 可作为首版离线阈值，恢复后不得自动沿用旧使能。 | 高 | 已确认 |
| RingBuffer 可复用 | `Middlewares/Third_Party/RingBuffer`、`spec/conventions/index.md` | 工程已有 LwRB v3.2.0 适配；UART 字节流计划由任务层持有 RingBuffer，避免 BSP 反向依赖中间件。 | 高 | 已确认 |
| 任务运行时缺口 | `User/task/user_task.h`、`User/task/init.c` | 当前运行时只有 `motor_chassis` 任务，没有 DR16 任务句柄、线程标志、最新状态邮箱或接收统计。 | 高 | 待新增 |
| 板上验证缺口 | 当前工作区无 DR16 串口抓包或实测日志 | 代码与配置证据不能替代 C 板反相接口、18 字节帧边界、通道方向和断线恢复实测。 | 高 | 待实测 |
| BSP 所有权 | 用户确认、`AGENTS.md` 分层约束 | DR16 字节流能力融合进现有 `uart.c/.h`，由单一 UART BSP 统一拥有普通收发、HAL 回调、DMA 状态和错误恢复；不修改 CAN BSP。 | 高 | 已确认 |
| 第三方格式边界 | `Middlewares/Third_Party/RingBuffer`、项目注释规则 | RingBuffer 是上游第三方实现，保留其原始排版和英文许可证注释；项目自写 `PORTING.md` 保持中文，不对第三方源码做风格重排。 | 高 | 已确认 |
| task 安全默认值 | `User/task/motor_chassis.c:62-68`、`motor_speed_control.h:36-38` | 当前上电默认 `motor_debug_enable=true` 且目标为 100 rpm；电机反馈在线后会进入非零速度环，不符合先零输出、明确使能后再运行的安全原则。 | 高 | 阻塞修复 |
| task 分层违规 | `User/task/motor_speed_control.c/.h`、`agent.md` | 速度 PID、滤波、斜坡和控制状态属于完整业务模块，却位于 task 层；task 应只保留调度、消息和模块编排。 | 高 | 阻塞修复 |
| RTOS 创建未检查 | `User/task/init.c:36-40`、`agent.md` | `osThreadNew()` 与 `osMessageQueueNew()` 返回值未检查，创建失败后仍解锁调度并终止初始化任务。 | 高 | 阻塞修复 |
| task 占位运行时 | `User/task/user_task.h:39-70`、全仓引用扫描 | `user_msg`、battery/vbat/cpu_temp、stack/freq/last_up_time 等字段没有所有者或消费者；`user_msg` 仍被创建，违反零占位符和运行时对象必须有明确用途的原则。 | 高 | 待清理 |
| task 排版违规 | 项目规则扫描、`User/task` | `motor_chassis.c` 有 16 处 Allman 违规，`motor_speed_control.h` 有 4 处；`motor_speed_control.c` 有 14 个无花括号条件块，且存在可在 120 列内表达却被机械拆行的声明与调用。 | 高 | 待修复 |
| while 注释缺口 | `User/task/motor_chassis.c:205,218`、`agent.md` | while 内 `MotorSpeedControl_DumpOutput()` 与 `MotorChassis_UpdateOzoneData()` 调用前缺少本次调用目的说明。 | 高 | 待修复 |
| task 初始化错误处理 | `User/task/motor_chassis.c:155` | `BSP_CAN_Init()` 返回值被忽略，初始化失败后任务仍继续注册电机并进入控制循环。 | 高 | 阻塞修复 |
| 通用工具局限 | code-review `check_style.py`、clang-tidy | 通用脚本针对 Python 且 clang-tidy 为 0 条发现，均无法覆盖本项目 Allman、结构体集中说明、while 注释和分层规则；最终必须执行项目专用扫描与人工门禁。 | 高 | 已确认 |
