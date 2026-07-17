# 分层架构 spec（architecture）

> index.md 是本层入口：开发前清单（Pre-Development Checklist）+ 质量门（Quality Check）。
> RESEARCH/PLAN 必读；派 Builder/Verifier 时按 implement/verify.jsonl 自动注入。

## 六层模型与依赖方向

| 层 | 前缀 | 职责 | 只能依赖 |
|---|---|---|---|
| L1 HAL Port | `halport_` | 厂商 HAL/寄存器的薄封装，隔离硬件差异 | 厂商头 |
| L2 BSP | `bsp_` | 板级：时钟/引脚/外设实例 | L1 |
| L3 Driver | `drv_` | 设备驱动（传感器/屏/存储） | L1/L2 |
| L4 Middleware | `mw_` | 协议/算法/文件系统 | L1~L3 |
| L5 Service | `svc_` | 业务服务（采集/控制/CLI） | L1~L4 |
| L6 App | `app_` | 编排与主循环 | L2~L5（**禁** L1 厂商头） |

依赖只能自上而下；跨硬件一律走 L1 HAL Port。

## 开发前清单（Pre-Development Checklist）

- [ ] 每个新文件标明层级 L1~L6 与命名前缀
- [ ] 应用层（app/）**不** `#include` 厂商 HAL 头（stm32*/gd32*/esp_*/ti_msp_dl_* 等）
- [ ] 应用层**不**直接裸写寄存器（`*(volatile..*)0x..`）——封装到 HAL/BSP
- [ ] 不引入 catch-all mega-header（`*_headfile.h`/`all.h`）间接拉厂商头
- [ ] `main.c` 只做启动编排与主循环调度；CubeMX/HAL/RTOS 生成的初始化与调度调用数量不限，其余自定义顶层调用 ≤ 6

## 质量门（Quality Check，REVIEW 必跑）

机械门禁（与 embedded-dev 同源思路，可外接 arch-check）：

| 编号 | 规则 |
|---|---|
| ARCH-1/1B/1C | 应用层禁厂商头 / 禁 catch-all mega-header / 禁裸 MMIO |
| ARCH-2 | main.c 自定义顶层调用 ≤ 6；`HAL_Init`、时钟配置、`MX_*_Init` 和 RTOS 内核/调度器调用不计数 |
| ARCH-3 | ISR/回调函数体 ≤ 20 行 |
| ARCH-4 | 应用层 extern 变量 = 0 |
| ARCH-5/6 | 单 .c ≤ 800 行；单 .h 公共 API ≤ 20 |
| ARCH-7/7B | mega-header 检测 / app 层引用 mega-header |
| ARCH-8 | hw-lock.yaml pin/dma/irq/timer 冲突检测 |

> 若工程已装 embedded-dev 的 `scripts/arch-check.sh`/`.ps1`，REVIEW 阶段直接跑它做门禁。

## 沉淀（promote 回流）
> 只沉淀**可复用知识**（决策/约定/坑/模式），任务过程性事实留在 tasks/ 不要 promote。下次会自动注入。
- [设计决策] ARCH-2 只统计 main 类入口的自定义顶层调用；HAL_Init、时钟配置、MX_*_Init 和 RTOS 内核/调度器调用属于生成的启动编排，不计入数量上限。
- [设计决策] 单个 M3508 电机控制按主运行与速度算法拆分：motor_chassis 拥有任务循环、电机驱动编排、在线判断和电流指令下发；motor_speed_control 只负责这一台电机的速度限幅、缓启动、PID 计算、调参和速度反馈结构体，不代表整个底盘速度闭环。
- [坑/gotcha] motor_speed_control 是单个 M3508 电机的速度算法模块，不应拥有 Task_motor_chassis 或直接调用 RM 设备驱动；现有 motor_chassis 任务入口、在线判断、反馈采集和电流指令下发必须由 motor_chassis 编排。
- [约定] motor_chassis 作为当前单电机主运行文件时直接调用现有 RM 驱动 API；仅被该任务使用的一对一 getter/setter 不再包装成公共函数，只有多调用者共享或需要统一策略时才增加封装。
- [可复用模式] RTOS 业务任务的 while 循环只保留周期推进、module/driver API 调用和等待；算法细节放入 module，实现中不为一对一设备调用增加 task 内 static 步骤函数。
- [可复用模式] 参考工程采用 task/module/component 三层：task 只负责消息或设备 I/O、调用顺序和周期调度；module 用一个主结构体集中保存 parameter/setpoint/feedback/PID/output，并提供 Init/UpdateFeedback/Control/DumpOutput 业务 API；component 只提供通用 PID、滤波等算法。禁止用 task 文件内的 static 步骤函数替代 module 边界。
- [约定] osKernelLock() 只锁定 RTOS 任务调度，不会屏蔽硬件中断；ISR 与任务共享的数据仍须按访问模型使用 volatile、原子操作、RTOS 通知或最小临界区保护，禁止把调度锁描述或使用为中断锁。
