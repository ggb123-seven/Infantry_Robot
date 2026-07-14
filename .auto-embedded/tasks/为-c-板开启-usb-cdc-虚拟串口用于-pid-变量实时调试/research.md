# 研究发现

| 关键词 | 来源 | 摘要 | 可信度 | 状态 |
|---|---|---|---|---|
| MCU 与框架 | `Infantry_Robot.ioc` | MCU 为 STM32F407IGH6，CubeMX 6.17.0，STM32Cube FW_F4 V1.28.3，使用 CMSIS-RTOS2/FreeRTOS。 | 高 | 已确认 |
| USB 时钟 | `Infantry_Robot.ioc` | HSE 12 MHz，PLLM=6、PLLN=168、PLLQ=7，USB 48 MHz 时钟值为 48000000 Hz。 | 高 | 已确认 |
| 当前 USB 状态 | `Infantry_Robot.ioc`、仓库文件扫描 | 当前 `.ioc` 未启用 USB_OTG_FS 和 USB_DEVICE；仓库不存在 USB Device/CDC 中间件与生成接口，仅有 HAL LL USB 底层文件。 | 高 | 已确认 |
| C 板 USB 引脚 | `F:/RM/Development-Board-C-Examples-master/20.standard_robot/standard_robot.ioc` | 官方 C 板例程使用 PA11=USB_OTG_FS_DM、PA12=USB_OTG_FS_DP，USB_OTG_FS 为 Device_Only。 | 高 | 已确认 |
| C 板 USB 参数 | `F:/RM/Development-Board-C-Examples-master/20.standard_robot/Src/usbd_conf.c` | 官方例程采用 Full Speed、内置 PHY、禁用 DMA/SOF/低功耗/LPM/VBUS sensing，OTG_FS 中断优先级为 5。 | 高 | 已确认 |
| CDC 发送语义 | `F:/RM/Development-Board-C-Examples-master/17.chassis_task/Src/usbd_cdc_if.c` | `CDC_Transmit_FS` 在前一帧未完成时返回 `USBD_BUSY`；发送缓冲区由 USB 栈异步使用。 | 高 | 已确认 |
| 现有控制数据 | `User/task/motor_chassis.c`、`User/task/motor_chassis.h` | 已有带临界区读取的电机反馈快照；当前控制输出是固定转矩电流，尚无 PID 目标、误差和 PID 输出字段。 | 高 | 已确认 |
| 调试协议 | 任务设计决策 | 默认使用 VOFA+ JustFloat：4 个 float 后接 `00 00 80 7F` 帧尾；100 Hz、20 B/帧时约 2 kB/s。 | 中 | 待实现验证 |
| 非阻塞发送 | 任务设计决策 | 低优先级独立任务读取快照并使用双缓冲发送；`USBD_BUSY` 时不等待、不重试阻塞，只累计丢帧。 | 中 | 待实现验证 |

## CubeMX 待授权配置差异

- `Connectivity > USB_OTG_FS`：选择 `Device_Only`，占用 PA11/PA12。
- `Middleware and Software Packs > USB_DEVICE`：选择 `Communication Device Class (Virtual Port Com Port)`。
- USB 参数与官方 C 板例程保持一致：Full Speed、Embedded PHY、VBUS sensing disabled、SOF disabled、Low Power disabled、LPM disabled、DMA disabled。
- `OTG_FS_IRQn`：启用，抢占优先级 5、子优先级 0。
- 保持现有 48 MHz USB 时钟配置不变。
- 重新生成后检查 CMake 是否自动纳入 HAL PCD/LL USB、USB Device Core、CDC 类和生成的 USB 接口源码。

## 已知门禁问题

- `python .auto-embedded/scripts/check.py` 当前因 `.auto-embedded/scripts/arch-check.ps1` 解析错误导致 ARCH 门失败。
- 同一检查还把 PendSV、SysTick、TIM6_DAC 共用优先级 15/0 报为硬件冲突；中断优先级数值相同并不等于 IRQ 资源冲突，需要修正检查规则后再作为有效门禁。
