# 移植 DR16 遥控器驱动到当前工程

## 需求 / 验收标准
- 在现有 `User/bsp/uart.*` 中融合 Circular DMA 字节流接口并保持普通收发 API，保留现有 `User/bsp/can.*`；重构 `User/device/dr16.*` 并纳入构建。
- UART 接收启动后保持循环 DMA 运行；HT、TC、IDLE 事件只推进新字节并通知任务，不在 ISR 中解码或执行控制。
- DR16 解码接口必须接收显式长度的 18 字节数组，使用移位与掩码解析，禁止依赖 packed bitfield 布局。
- 完整帧通过通道范围和拨杆枚举校验后一次性提交；非法帧不得修改上一份合法状态或更新时间戳。
- 字节错位时逐字节重同步；RingBuffer 溢出或 UART ORE/FE/NE/PE 后丢弃未验证数据并重新开始同步。
- 首版离线阈值为 100 ms；离线时发布 `online=false` 的零化状态，恢复后只接受新的完整合法帧。
- DR16 任务通过长度为 1 的最新状态邮箱发布一致快照，不直接修改底盘目标、电流或使能状态。
- CMake Debug clean build、机械门禁和主机解码/重同步测试全部通过；最终还需完成 C 板 + DR16 板上实测。

## 约束
- 本轮不修改 `Infantry_Robot.ioc`、`Src/Inc` 生成代码或 IRQ 入口；现有 UART/DMA 配置与 HAL IRQ 链路已满足方案。
- 依赖方向保持 `task -> device/bsp -> HAL`；RingBuffer 由任务层持有，UART BSP 通过注册的函数指针通知任务，不包含任务头文件或 DR16 协议。
- ISR 只复制 DMA 新增字节、记录错误和设置线程标志；禁止阻塞、日志、动态内存、解码和控制计算。
- `lwrb_reset()` 仅在暂停 UART 接收或最小临界区内执行，避免 ISR 写入与任务复位并发。
- 新增或修改注释使用简体中文和 UTF-8，C/C++ 代码使用 Allman 风格，单行不超过 120 列。
- 项目自研源码必须统一格式；第三方 RingBuffer 保持上游源码排版和许可证注释，不进行项目风格重排。
- 实施阶段修改 `User/bsp/uart.c/.h` 以统一拥有 HAL UART 回调和字节流状态，不修改现有 CAN BSP；此外将修改 `User/device`、`User/task`、`User/module` 和 CMake 配置，明确不修改 Keil 工程。

## 数据流与所有权

```text
USART3 + Circular DMA
  -> HAL HT/TC/IDLE callback
  -> UART BSP 计算 DMA 新增区段并调用已注册回调
  -> DR16 任务私有回调写入任务私有 RingBuffer + 设置线程标志
  -> DR16 任务逐帧查找/重同步
  -> DR16 device 纯函数解码与校验
  -> 长度 1 最新状态邮箱
  -> 后续命令映射任务消费（本轮不接底盘电流）
```

## 遥控状态模型

### L3 Driver：协议解码结果

`User/device/dr16.h` 规划公开以下稳定数据，不公开 packed bitfield 原始结构：

```c
typedef struct
{
    int16_t ch_l_x;
    int16_t ch_l_y;
    int16_t ch_r_x;
    int16_t ch_r_y;
    int16_t wheel;
    DR16_SwitchPos_t sw_l;
    DR16_SwitchPos_t sw_r;
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    bool mouse_left;
    bool mouse_right;
    uint16_t key_mask;
} DR16_Data_t;

typedef struct
{
    DEVICE_Header_t header;
    uint32_t valid_frame_sequence;
    DR16_Data_t data;
} DR16_State_t;
```

- 五个通道统一保存减去中心值 1024 后的有符号量，典型范围 `-660..660`，单位为 DR16 原始计数。
- 死区、`-1..1` 归一化、最大速度映射和斜坡属于后续命令模块，不进入 DR16 device。
- `header.last_online_time` 只在完整合法帧提交时更新，单位沿用 BSP time 的微秒。
- `valid_frame_sequence` 只在合法帧提交时递增；离线发布保留最后序号，避免把离线事件误判为新遥控帧。

规划接口：

```c
int8_t DR16_Decode(const uint8_t *frame, size_t length, DR16_Data_t *output);
void DR16_ResetState(DR16_State_t *state);
```

`DR16_Decode()` 先写局部临时对象，全部字段校验通过后才覆盖 `output`；失败时 `output` 保持不变。

### L6 App：最新状态与诊断状态

- `Task_dr16()` 独占 `DR16_State_t` 的写权限。
- `task_runtime.msgq.dr16_state` 使用 `osMessageQueueNew(1U, sizeof(DR16_State_t), NULL)` 创建。
- 每次合法帧使用“重置后写入”发布最新快照；该邮箱只规划一个业务消费者，后续若出现多消费者再改为广播快照机制。
- 初始状态为 `online=false`、时间戳 0、序号 0、数据全零。
- 非法帧只增加诊断计数，不发布、不更新时间戳；连续无合法帧达到 100 ms 时只发布一次离线零状态。
- UART ORE/FE/NE/PE 或 RingBuffer 溢出时立即发布离线零状态，并在接收链路恢复后等待新的合法帧。

Ozone 诊断结构 `volatile DR16_Monitor_t g_dr16_monitor` 与业务邮箱分离，规划包含：接收字节数、UART 事件数、
合法帧数、非法帧数、重同步丢弃字节数、RingBuffer 溢出数、UART 错误数、最后错误码、当前帧年龄、
在线状态和最近一份解码值。控制任务不得读取该调试结构作为控制输入。

规划任务入口：

```c
void Task_dr16(void *argument);
```

### L2 BSP：融合字节流接口

在现有 `User/bsp/uart.c/.h` 中融合 UART 字节流、普通收发、HAL 回调和错误状态，统一使用 `BSP_UART_t` 标识端口；该 BSP 不包含 DR16 类型，也不包含或直接调用 task 符号：

```c
typedef void (*BSP_UART_StreamRxCallback_t)(const uint8_t *data, uint16_t length);
typedef void (*BSP_UART_StreamErrorCallback_t)(uint32_t error_code);

int8_t BSP_UART_StreamRegisterCallbacks(BSP_UART_t uart, BSP_UART_StreamRxCallback_t rx_callback,
                                         BSP_UART_StreamErrorCallback_t error_callback);
int8_t BSP_UART_StreamStart(BSP_UART_t uart);
int8_t BSP_UART_StreamStop(BSP_UART_t uart);
int8_t BSP_UART_StreamRestart(BSP_UART_t uart);
uint32_t BSP_UART_StreamGetError(BSP_UART_t uart);
```

字节指针只在 ISR 回调期间有效；回调必须立即复制到任务私有 RingBuffer，禁止保存 DMA 区指针。

`User/task/dr16_task.c` 内部注册以下私有回调，负责把 BSP 提供的字节和错误转换为任务线程标志；它们不是 HAL 回调，不会让 task 直接依赖 HAL：

```c
static void DR16_TaskRxDataCallback(const uint8_t *data, uint16_t length);
static void DR16_TaskUartErrorCallback(uint32_t error_code);
```

## 寄存器与 HAL 配置

本任务不直接写寄存器，也不修改 `.ioc`。实施以现有 CubeMX/HAL 配置为冻结输入：

- USART3：100000 baud，9-bit word length，Even parity，1 stop bit，RX-only，无硬件流控。
- USART3 CR1：HAL 初始化得到 8 个有效数据位 + 偶校验；`HAL_UARTEx_ReceiveToIdle_DMA()` 启用 IDLEIE。
- USART3 CR3：HAL DMA 接收启用 DMAR；UART 错误中断由 HAL 管理。
- DMA1 Stream1 Channel 4：Peripheral-to-memory、byte alignment、memory increment、Circular、High priority。
- DMA1 Stream1 IRQ 与 USART3 IRQ：抢占优先级 5、子优先级 0，可调用 CMSIS-RTOS2 ISR-safe API。

## 文件分层

| 文件 | 层级 | 角色 | review |
|---|---|---|---|
| `User/bsp/uart.c/.h` | L2 BSP | 融合普通 UART API、USART3 Circular DMA 字节流、HAL 回调和错误恢复 | `true` |
| `User/device/dr16.c/.h` | L3 Driver | 18 字节协议解码、合法性检查和状态复位 | `true` |
| `User/task/dr16_task.c/.h` | L6 App | RingBuffer、重同步、离线判断和邮箱发布 | `true` |
| `User/module/motor_speed_control.c/.h` | L5 Service / Module | 从 task 迁移单电机速度闭环业务模块 | `true` |
| `User/task/motor_chassis.c/.h` | L6 App | 只保留周期调度、设备 I/O 和 module 编排，修复安全默认值 | `true` |
| `User/task/user_task.c/.h`、`User/task/init.c` | L6 App | 任务属性、运行时句柄和 RTOS 对象创建 | `false` |
| `tests/dr16_decode_test.c` | Host Test | 纯解码、非法帧和错位恢复测试 | `false` |
| `CMakeLists.txt` | Build | 接入 DR16 链路所需源文件 | `false` |

## 待确认决策

| 编号 | 规划决定 | review |
|---|---|---|
| DR16-STATE-1 | 业务状态保存中心化后的 `int16_t -660..660` 原始计数，不在 device 层提前转成 float。 | `true`，用户已确认 |
| DR16-STATE-2 | 非法帧不发布；100 ms 无合法帧或 UART/RingBuffer 错误立即发布一次离线零状态。 | `true`，用户已确认 |
| DR16-STATE-3 | 当前使用容量 1 的最新状态邮箱，并限定单一业务消费者；必须保序的按键动作后续使用独立事件队列。 | `true`，用户已确认 |
| DR16-STATE-4 | 本轮状态只供观察和后续命令模块消费，不连接电机电流、底盘使能或模式切换。 | `true`，用户已确认 |
| TASK-COMPLIANCE-1 | 先修复现有 `User/task`：默认禁止非零输出、检查 RTOS/CAN 初始化、移走速度算法、删除无用途占位、统一注释与排版。 | `true`，待确认 |
| BSP-OWNER-1 | 修改现有 `User/bsp/uart.c/.h`，由单一 UART BSP 统一拥有 HAL 回调和字节流状态；不修改 CAN BSP，DR16 数据处理回调放在 `dr16_task.c`。 | `true`，用户已确认 |

## 实施清单

1. 修复现有 task 基线：默认关闭电机调试使能并将启动目标清零；检查 `osKernelLock()`、任务/队列创建和 `BSP_CAN_Init()`；删除无用途运行时占位字段与队列。
2. 新建 `User/module/motor_speed_control.c/.h`，把 PID、滤波、斜坡和速度控制状态从 `User/task` 迁出；`motor_chassis` 只保留周期调度、设备 I/O 与 module 调用。
3. 统一现有 `User/task` 的 Allman、必要花括号、120 列、中文 Doxygen、结构体字段集中说明、处理阶段目的注释和 while 内调用目的注释。
4. 扩展 `User/bsp/uart.h/.c`：融合 receive-to-idle 循环 DMA 启停、字节区段回调、错误查询和恢复接口；不修改 CAN BSP。
5. 重构 `User/device/dr16.h/.c`：定义 18 字节协议常量、稳定输出结构和纯解码接口；显式解析摇杆、拨杆、鼠标、键盘及第五通道，并采用临时对象完成全量校验后提交。
6. 新增 `User/task/dr16_task.h/.c`：拥有 RingBuffer、DMA 事件通知、逐字节重同步、100 ms 离线检测、错误恢复和调试统计。
7. 更新 `User/task/user_task.h/.c` 与 `init.c`：集中声明和创建 DR16 任务、线程标志及长度 1 状态邮箱，检查所有 RTOS 对象创建结果后再运行任务。
8. 清理 `User/device/device.h` 中旧的 `SIGNAL_DR16_RAW_REDY` 及拼写问题，线程标志归属任务层。
9. 更新根 `CMakeLists.txt`，加入 module、USART/DMA/HAL UART、UART BSP、DR16 device 和 DR16 task；不修改 Keil 工程。
10. 增加主机测试：中心/端点通道、拨杆、键鼠、第五通道、非法长度、非法通道、非法拨杆、输出不污染、前置噪声与错位重同步。
11. 执行项目规则扫描、clang-tidy、clean build、`check.py`、ELF 产物与内存占用检查；板上观察有效帧率、非法帧、重同步、溢出、UART 错误、最后更新时间和在线状态。

BSP 保持性验证标准：

- EXECUTE 首次源码编辑前记录 `can.c/.h` 的 SHA-256，完成后逐一比对；融合修改 `uart.c/.h` 时保留已有普通收发与回调能力，不回退其他未提交改动。
- `uart.c/.h` 不包含 `task/`、`device/dr16.h` 或 RingBuffer，只依赖 HAL、生成的 USART/DMA 句柄和 BSP 公共状态码。
- `dr16_task.c` 不直接包含 HAL 或生成代码头文件，HAL 回调只存在于 L2 BSP。
- 新增或修改的函数注释包含中文 `@brief`、完整 `@param` 和 `@return`。

## CAN 设备聚合延续改造

- MVP 只纳管当前实际存在的四个底盘 M3508，不提前加入云台、射击或超级电容字段。
- `User/device/can_devices.c/.h` 负责固定设备参数、注册、反馈更新、电流写入和统一组发送；
  `motor_rm.c/.h` 继续负责 RM 电机协议、反馈解码和分组发送。
- `User/task/can_task.c/.h` 是 `CANDevices_Init()`、`CANDevices_UpdateFeedback()` 和
  `CANDevices_ApplyCurrent()` 的唯一 task 层调用者。
- CAN task 与底盘 task 使用两个容量为 1 的最新状态邮箱：CAN task 发布 `CANDevices_Snapshot_t`，
  底盘 task 发布 `CANDevices_Command_t`。
- CAN task 每周期只消费最新电流命令；没有新命令、邮箱异常或命令非法时必须提交四路零电流，禁止保持旧非零命令。
- 底盘 task 每周期只消费最新 CAN 快照；没有新反馈时四路 `Chassis_Feedback_t.valid` 均为 false，
  由 Chassis 产生安全零输出。
- CAN task 负责总线周期，底盘 task 只负责控制周期和消息路由；两者均使用 `osDelayUntil()` 保持稳定周期。
- 两个邮箱必须先于 CAN 和底盘任务创建，任一对象创建失败时按依赖逆序终止任务并删除邮箱。
- 本延续改造为 `review:true`：独立 CAN task、双邮箱和底盘 task 改造完成并通过验证后，等待用户确认再提交。

## 每次编码后的强制复核门

每完成一个实现项，必须在进入下一项前执行：

1. 重新读取仓库根 `agent.md` 与 `AGENTS.md`，确认当前文件的分层、授权、注释和排版要求。
2. 扫描同行左花括号、`} else {`、无花括号控制块、超过 120 列、行尾空白和 120 列内可单行却被机械拆分的语句。
3. 人工检查结构体字段是否集中说明、每个处理阶段和 while 内函数调用是否有中文目的注释、是否存在逐行翻译式无效注释、Doxygen 参数/返回值是否完整。
4. 检查 task 是否只保留调度、消息、设备 I/O 和 module 编排，禁止把协议、PID、滤波或业务状态机堆回 task。
5. 检查所有 RTOS/HAL/BSP 创建与初始化返回值、ISR 共享状态和临界区内容，确认无占位对象和无默认非零输出。
6. 运行 clang-tidy、对应主机测试、Debug clean build 和 `python .auto-embedded/scripts/check.py`。
7. Verifier 按本清单独立复核；未给出命令输出和逐条对照时，不得宣称完成。

## 不在本轮范围

- 不把 DR16 数据直接映射到底盘电流或电机使能。
- 不实现底盘模式状态机、速度命令映射、斜坡或断线恢复后的重新解锁逻辑。
- 不改 CubeMX 引脚、DMA stream、UART 参数、中断优先级或 `.ioc`。
- 不修改 `MDK-ARM/Infantry_Robot.uvprojx` 或其他 Keil 配置。
