# 驱动电机控制链路与 BSP_CAN_Init 分析记录

**日期**: 2026-07-13
**报告类型**: analysis
**提交类型**: docs

## 1. 概述

记录当前工程中 M3508 驱动电机上电后持续旋转的实际控制链路，并拆解 `BSP_CAN_Init()` 的软件资源、CAN 过滤器、收发回调和中断通知初始化过程。

本记录依据当前参与 CMake Debug 构建的源码整理，不包含开发板烧录和实车运行结论。

## 2. 当前电机配置

电机参数定义在 `User/task/motor_chassis.c`：

```c
static MOTOR_RM_Param_t motor_3508_param =
{
    .can = BSP_CAN_1,
    .id = 0x201U,
    .module = MOTOR_M3508,
    .reverse = false,
    .gear = true,
};
```

参数含义：

- 使用 CAN1。
- C620 电调 ID 为 1，其反馈帧 ID 为 `0x201`。
- 电机控制方向不反转。
- 反馈角度和转速按照 M3508 减速箱输出轴换算。

## 3. 任务启动链路

当前任务启动顺序为：

```text
main()
  -> MX_CAN1_Init()
  -> osKernelInitialize()
  -> MX_FREERTOS_Init()
  -> 创建 Task_Init
  -> osKernelStart()
  -> Task_Init 创建 Task_motor_chassis
  -> Task_motor_chassis 初始化 BSP CAN 和 RM 电机
```

涉及文件：

- `Src/main.c`：初始化 CAN1、FreeRTOS 内核并启动调度器。
- `Src/freertos.c`：创建 `Task_Init`。
- `User/task/init.c`：创建 `Task_motor_chassis`。
- `User/task/user_task.h`：将电机任务频率配置为 `500 Hz`，启动延时配置为 0。

## 4. 电机持续旋转的控制循环

`Task_motor_chassis()` 完成初始化后持续执行：

```c
MOTOR_RM_Update(&motor_3508_param);
MOTOR_RM_SetTorqueCurrent(&motor_3508_param, 0.8F);
MOTOR_RM_Ctrl(&motor_3508_param);
```

控制链路为：

```text
读取 0x201 电机反馈
  -> 设置固定目标转矩电流 0.8 A
  -> 写入 CAN 管理器的 ID 1 输出槽
  -> 生成标准数据帧 0x200
  -> BSP_CAN_TransmitStdDataFrame()
  -> HAL_CAN_AddTxMessage()
  -> C620 接收电流指令并驱动 M3508
```

该循环不是速度闭环。反馈中的角度、转速、电流和温度只被更新到电机实例，当前任务没有根据反馈计算新的速度或电流目标。电机持续旋转的直接原因是任务以固定周期重复下发正向 `0.8 A` 转矩电流。

## 5. 电流指令与 CAN 帧

M3508/C620 的转矩电流指令范围为：

```text
-16384 ~ 0 ~ 16384 raw
对应
-20 A ~ 0 A ~ 20 A
```

`0.8 A` 换算结果为：

```text
0.8 A * 16384 / 20 A = 655.36 raw
转换为 int16_t 后为 655，即 0x028F
```

当前只有 ID 1 的 M3508 注册到低 ID 控制组，因此发送帧为：

```text
标准帧 ID：0x200
DLC：8
Data：02 8F 00 00 00 00 00 00
```

前两个字节控制 ID 1，后续三个电机槽位保持为零。

## 6. BSP_CAN_Init 的职责边界

`MX_CAN1_Init()` 与 `BSP_CAN_Init()` 分工不同：

```text
MX_CAN1_Init()
  -> CAN 位时序和正常工作模式
  -> PD0/PD1 复用功能
  -> CAN1 外设时钟
  -> CAN1_RX0_IRQn 优先级和 NVIC 使能

BSP_CAN_Init()
  -> 软件发送队列和回调表
  -> CAN 接收过滤器
  -> 启动 CAN1
  -> BSP 收发回调注册
  -> HAL CAN 中断通知激活
```

`BSP_CAN_Init()` 不负责设置 CAN 波特率、GPIO 或 NVIC，也不直接创建 `0x201` 接收队列。`0x201` 最新帧队列由后续的 `MOTOR_RM_Register()` 通过 `BSP_CAN_RegisterLatestId()` 创建。

## 7. BSP_CAN_Init 执行顺序

### 7.1 防止重复初始化

函数首先检查静态标志 `inited`。已初始化时返回 `BSP_ERR_INITED`。

### 7.2 初始化软件资源

依次完成：

1. 清零 `CAN_Callback` 回调函数表。
2. 将每条 CAN 软件发送队列的 `head` 和 `tail` 归零。
3. 将 ID 解析器设置为 `BSP_CAN_DefaultIdParser()`，当前默认解析器原样返回 CAN ID。
4. 创建接收队列管理使用的 CMSIS-RTOS2 互斥锁 `queue_mutex`。

### 7.3 配置 CAN1 过滤器

Filter Bank 0 使用 32 位掩码模式：

```text
FilterIdHigh       = 0
FilterIdLow        = 0
FilterMaskIdHigh   = 0
FilterMaskIdLow    = 0
FilterFIFOAssignment = FIFO0
```

ID 和掩码均为零，因此硬件层接收全部 CAN ID，并送入 FIFO0。软件层只把已注册 ID 的消息放入相应队列；未注册 ID 在 FIFO0 回调中找不到队列节点后被忽略。

### 7.4 配置过滤器并启动 CAN1

调用顺序为：

```c
HAL_CAN_ConfigFilter(&hcan1, &can1_filter);
HAL_CAN_Start(&hcan1);
```

任一调用失败时清除 `inited` 并返回 `BSP_ERR`。

### 7.5 注册接收和发送完成回调

CAN1 注册以下 BSP 回调：

- FIFO0 消息待处理：`BSP_CAN_RxFifo0Callback()`。
- 邮箱 0 发送完成：`BSP_CAN_TxCompleteCallback()`。
- 邮箱 1 发送完成：`BSP_CAN_TxCompleteCallback()`。
- 邮箱 2 发送完成：`BSP_CAN_TxCompleteCallback()`。

### 7.6 激活 HAL 通知

激活以下通知：

```c
CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY
```

当前没有激活 FIFO full、bus-off 或 CAN error 等错误类通知，也没有为这些错误路径注册实际处理函数。

## 8. CAN 接收链路

收到 C620 的 `0x201` 反馈后，执行路径为：

```text
CAN1_RX0_IRQHandler
  -> HAL CAN IRQ Handler
  -> HAL_CAN_RxFifo0MsgPendingCallback()
  -> CAN_Callback[CAN1][RX_FIFO0_PENDING]
  -> BSP_CAN_RxFifo0Callback()
  -> HAL_CAN_GetRxMessage()
  -> BSP_CAN_ParseId()
  -> 查找 0x201 软件队列
  -> BSP_CAN_PushRxMessage()
  -> MOTOR_RM_Update() 非阻塞读取最新一帧
  -> Motor_RM_Decode() 解码反馈
```

解码内容包括编码器角度、转速、转矩电流、温度、在线时间和多圈角度状态。

## 9. CAN 发送链路

任务发送电流控制帧时：

```text
MOTOR_RM_Ctrl()
  -> MOTOR_RM_SendGroup()
  -> MOTOR_RM_FillTxFrame()
  -> BSP_CAN_TransmitStdDataFrame()
  -> BSP_CAN_Transmit()
```

`BSP_CAN_Transmit()` 在 FreeRTOS 临界区内检查发送资源：

- 软件队列为空且硬件邮箱有空间时，直接调用 `HAL_CAN_AddTxMessage()`。
- 硬件邮箱不可用时，将帧压入软件发送环形队列。
- 任一硬件邮箱发送完成后，`BSP_CAN_TxCompleteCallback()` 继续从软件队列取帧并装入空闲邮箱。
- 软件发送队列已满时，本次发送返回错误。

## 10. 当前实现的边界与风险

1. `BSP_CAN_Init()`、`MOTOR_RM_Register()` 和 `MOTOR_RM_Update()` 的返回值当前未保存，也未据此切断电机输出。
2. `MOTOR_RM_SetTorqueCurrent()` 和 `MOTOR_RM_Ctrl()` 的返回值仍未保存。
3. 电机反馈超过 100 ms 未更新时，驱动会标记电机离线，但任务仍继续写入并发送 `0.8 A` 指令。
4. 当前没有遥控器使能、急停状态、上电解锁流程或故障零输出路径。
5. 当前没有速度 PID，不能将 `0.8 A` 理解为固定转速命令。
6. `BSP_CAN_Init()` 失败后会清除 `inited`，但已创建的互斥锁、已启动的 CAN 状态和已注册的部分回调没有统一回滚。
7. 发送完成回调从队列弹出帧后，如果 `HAL_CAN_AddTxMessage()` 失败，该帧会被直接丢弃。

## 11. 调试器反馈观察接口

`User/task/motor_chassis.h` 定义了全局易失快照：

```c
g_motor_chassis_feedback
```

在调试器 Watch 窗口中添加该符号，可以观察：

- 电机在线状态。
- 输出轴累计角度和转速。
- 现有 RM 设备层按减速比换算后的输出侧等效转矩电流。
- 电机温度。
- 最近一次成功解析反馈的微秒时间戳。

任务内发布快照和应用层读取快照均使用短临界区，避免其他任务通过 `MotorChassis_GetFeedbackSnapshot()` 读取到字段更新一半的结构体。

## 12. 验证与已知限制

- `CMake Debug` 构建成功，产物为 `build/Debug/Infantry_Robot.elf`。
- 本次未修改 `.ioc` 或底层 CAN 配置；电机任务保持固定 `0.8 A` 开环转矩电流控制。
- 未执行开发板烧录、示波器/CAN 分析仪抓帧或实车电机测试。
- auto-embedded `SPEC` 检查通过。
- `ARCH` 检查脚本存在 PowerShell 解析错误，未产生有效架构检查结果。
- `HW` 检查报告 PendSV、SysTick 和 TIM6_DAC 使用相同优先级，机械门禁未全通过，因此未执行 promote。
