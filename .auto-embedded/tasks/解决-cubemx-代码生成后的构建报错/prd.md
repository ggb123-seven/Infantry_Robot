# 解决 CubeMX 代码生成后的构建报错

## 需求 / 验收标准
- `Debug` 配置完成编译和链接，命令返回码为 0。
- 生成新的 `build/Debug/Infantry_Robot.elf` 固件产物。
- 保留 CubeMX 生成的 CAN1、TIM6 HAL 时基和 CMSIS-RTOS V2 配置。
- 保留现有 `User/task` 与 `applications` 任务源码，并确保它们进入同一构建目标。

## 约束
- 不修改 `Infantry_Robot.ioc`，不改变芯片、引脚、时钟、外设、DMA、中断或 FreeRTOS 配置。
- 不回退工作区内已有的 CubeMX、厂商库和 `User/` 改动。
- 仅修改解决当前构建错误所需的普通工程文件。
