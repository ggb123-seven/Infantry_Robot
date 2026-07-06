# 自动探测结果（草案，待人工确认）

> 由 `aemb init` 扫描工程文件名/扩展名推断，**可能误判**。
> 确认无误后，请把相关信息手工并入 `index.md` 与 `hw-lock.yaml`，再删除本文件。

- 疑似芯片: STM32F407XX, STM32F4XX
- 疑似框架: STM32CubeMX
- 构建系统: Keil MDK

## 证据
- STM32F407XX ← stm32f407xx.h
- STM32F4XX ← stm32f4xx.h
- STM32CubeMX ← Infantry_Robot.ioc
- Keil MDK ← Infantry_Robot.uvprojx
