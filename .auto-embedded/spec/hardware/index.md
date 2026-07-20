# 硬件资源 spec（hardware）

> 全工程共享的"硬件事实基线"。RESEARCH 阶段冻结，后续以此为准；改动留变更记录。
> 机器可读锁定区在 `hw-lock.yaml`，REVIEW 的 ARCH-8 与冲突检测据此运行。

## 芯片与框架
- 芯片型号：STM32F407IGH6，UFBGA176
- 开发框架：STM32Cube FW_F4 V1.28.3、HAL、FreeRTOS、CMSIS-RTOS2
- 主频 / 时钟树：HSE 12 MHz，SYSCLK/HCLK 168 MHz，PLLQ 输出 48 MHz 供 USB FS 使用

## 引脚 / DMA / 中断（人类视图）
> 详细分配维护在 `hw-lock.yaml`（机器可读）；此处写设计理由与变更记录。

## 变更记录
| 日期 | 改了什么 | 原因 |
|---|---|---|
| 2026-07-14 | 登记 USB FS 的 PA11/PA12、OTG_FS_IRQn 和 48 MHz 时钟 | 为 C 板 CDC 虚拟串口冻结硬件资源；引脚与参数参考官方 C 板例程 |
| 2026-07-19 | 登记 DR16 的 USART3、PC10/PC11、DMA1 Stream1 Channel 4、DMA 和 USART3 中断 | 按 100000 baud、8E1、RX-only、循环 DMA 配置 RingBuffer 接收链路；STM32F4 使用 9-bit Word Length 承载 8 位数据与偶校验 |

## 沉淀（promote 回流）
> 本板踩过的硬件坑（如某 strap 脚、某外设时钟门）沉淀于此。
- [约定] 硬件资源锁按 pin、DMA stream、IRQn 和 timer 标识判重；不同 IRQ 使用相同抢占优先级和子优先级是合法配置。
- [约定] STM32F4 UART 开启偶校验时，校验位占用所选字长的最高位；DR16 的 100000 baud、8 data bits、even parity、1 stop bit 必须在 CubeMX 中配置为 9-bit Word Length、Even Parity、1 Stop Bit。
- [约定] UART 使用 DMA+RingBuffer 连续接收时，RX DMA 配置为 Circular 并只启动一次；USART IDLE 与 DMA HT/TC 事件根据 NDTR 推进生产者位置，任务侧消费数据并处理环回与溢出，禁止在事件间反复停止和重启 DMA。
