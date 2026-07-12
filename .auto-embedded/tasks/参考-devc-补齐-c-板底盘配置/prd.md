# 参考 DevC 补齐 C 板底盘配置

## 需求 / 验收标准
- 参考 `F:/RM/未命名文件夹/DevC.ioc`，为当前 STM32F407 C 板工程补齐底盘学习路线所需的 CAN1、USART3 RX 和 DMA 配置。
- CAN1 使用 PD0/PD1、1 Mbps，并启用 RX FIFO0/RX FIFO1 中断。
- USART3 使用 PC10/PC11、100000 baud、偶校验、RX-only；USART3_RX 使用 DMA1 Stream1 Channel4。
- 保持当前 12 MHz HSE、168 MHz 主频、CMSIS-RTOS V2、LED GPIO 和既有任务不变。
- 本轮只更新 `.ioc` 配置，不生成或修改 CAN/UART/DMA 源码、CMake、Keil 工程。

## 约束
- 不改变 `全向轮底盘学习章节.md` 的学习路线和章节结构。
- 不迁移 DevC 中与当前底盘第一阶段无关的 CAN2、ADC、SPI、I2C、USB、RNG、CRC、IMU 和多路定时器配置。
- USART3 字长不得机械照搬 DevC 的 `8B + EVEN`；执行前需依据当前 HAL 对 parity/word length 的定义确认 DR16 所需有效 8 data bits + even parity。
- 保留用户工作区现有未提交改动，不自动覆盖或回退。
- 代码生成和构建同步留到用户明确要求的后续阶段。
