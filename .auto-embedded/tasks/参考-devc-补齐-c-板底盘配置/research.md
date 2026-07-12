# 研究发现

| 关键词 | 来源 | 摘要 | 可信度 | 状态 |
|---|---|---|---|---|
| 共同硬件基线 | `DevC.ioc`、`Infantry_Robot.ioc` | 两者均为 STM32F407IGHx/UFBGA176，HSE=12 MHz，SYSCLK=168 MHz，APB1=42 MHz，APB2=84 MHz，CMSIS-RTOS V2。 | 高 | 已验证 |
| 当前配置缺口 | `Infantry_Robot.ioc` | 当前仅 RCC/SYS/NVIC/FreeRTOS 和 3 路 LED GPIO，无 CAN、USART、DMA；不能支撑底盘电机或 DR16 学习路线。 | 高 | 已验证 |
| CAN1 板级映射 | `DevC.ioc:429-434`、`Core/Src/can.c` | PD0=CAN1_RX、PD1=CAN1_TX，AF9，GPIO very high speed，无上下拉；CAN1 RX0/RX1 IRQ 为 5/0。 | 高 | 已验证 |
| CAN1 位时序 | `DevC.ioc:15-23`、`Core/Src/can.c:41-52` | APB1=42 MHz；Prescaler=3、SJW=1TQ、BS1=6TQ、BS2=7TQ，计算波特率 1 Mbps；TX FIFO priority enabled。 | 高 | 已验证 |
| DR16 UART 映射 | `DevC.ioc:393-398,676-680`、`Core/Src/usart.c` | PC10=USART3_TX、PC11=USART3_RX，AF7，100000 baud、even parity、RX-only。 | 高 | 已验证 |
| USART3 RX DMA | `DevC.ioc:102-111`、`Core/Src/usart.c:220-239` | DMA1 Stream1 Channel4，peripheral-to-memory、byte alignment、memory increment、normal mode、high priority；IRQ 5/0。 | 高 | 已验证 |
| USART3 字长风险 | `DevC Core/Src/usart.c:78-84`、`全向轮底盘学习章节.md:1144-1160` | DevC 使用 `UART_WORDLENGTH_8B + EVEN`；当前 HAL 中 parity 占据选定字长最高位，DR16 的 8 data bits + parity 需在实施前核对是否应配置 9B。 | 高 | 待实施确认 |
| 有意排除范围 | `DevC.ioc`、用户要求“学习路线不变” | 不整体复制 28 个 IP；本轮仅补底盘第一阶段所需 CAN1、USART3、DMA，保留当前 LED GPIO，不启用 CAN2/IMU/USB 等。 | 高 | 已确认范围 |
| 代码改动撤销 | 用户明确要求“现在不需要改动代码” | 已恢复 main/ISR/HAL 配置/CMake/Keil 文件，并删除新增 can/usart/dma 源文件；本轮仅保留 `.ioc` 和硬件锁配置。 | 高 | 已验证 |
| IOC 配置撤销 | 用户明确要求“ioc也恢复” | `Infantry_Robot.ioc` 已恢复到提交 `f6c571b` 的 CMSIS-RTOS V2 基线；DevC 的 CAN1/USART3/DMA/引脚/NVIC 配置已全部撤销，硬件锁同步清理。 | 高 | 已验证 |
| 门禁结果 | `py .auto-embedded/scripts/check.py` | SPEC 通过；ARCH 为既有 PowerShell 解析故障；HW 检查器把多个不同 IRQ 使用同一优先级误判为资源冲突，IRQ 名称和 DMA stream 本身无重复。 | 高 | 工具问题 |
