# 编辑清单

| 文件 | 改动 | 验证标准 | 结果 | commit |
|---|---|---|---|---|
| `Infantry_Robot.ioc` | 原计划增加 CAN1、USART3、DMA 配置。 | 用户要求恢复 IOC | 已恢复至提交 `f6c571b` | - |
| `Inc/can.h`、`Src/can.c` | 原计划新增 CAN 初始化代码。 | 本轮不改代码 | 已撤销并删除 | - |
| `Inc/usart.h`、`Src/usart.c` | 原计划新增 USART3/DMA 初始化代码。 | 本轮不改代码 | 已撤销并删除 | - |
| `Inc/dma.h`、`Src/dma.c` | 原计划新增 DMA 初始化代码。 | 本轮不改代码 | 已撤销并删除 | - |
| `Src/main.c`、`Src/stm32f4xx_it.c`、`Inc/stm32f4xx_it.h` | 原计划接入初始化和 IRQ。 | 本轮不改代码 | 已恢复至 HEAD | - |
| `Inc/stm32f4xx_hal_conf.h`、CMake、Keil | 原计划同步生成与构建配置。 | 本轮不改代码 | 已恢复至 HEAD | - |
| 验证 | 确认工作区中只有 `.ioc`/任务记录/硬件锁变化，不存在新增 CAN/UART/DMA 代码。`review:false` | 代码与构建文件无本轮 diff | 通过 | - |
