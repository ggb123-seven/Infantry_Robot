# CMSIS-RTOS V2 迁移记录

**日期**: 2026-07-11
**报告类型**: reflection
**提交类型**: feature

## 1. 概述

将 STM32F407 工程从 CMSIS-RTOS V1 迁移至 V2，并同步更新 CubeMX 配置、FreeRTOS wrapper、CMake 和 Keil 工程。

## 2. 用户需求

更新 CubeMX 工程配置，在 `.ioc` 中切换到 CMSIS-RTOS V2，并提交大致更改说明。

## 3. 工作流记录

完成环境核对、V1 API 扫描、迁移计划审查、分轮实现、CMake 构建和 ELF 符号验证。

## 4. 修改内容

- `.ioc` 更新为 CubeMX 6.17、STM32CubeF4 V1.28.3 和 CMSIS-RTOS V2。
- 线程 API 从 `osThreadDef/osThreadCreate` 迁移到 `osThreadNew`，栈保持等效 512 bytes。
- 添加官方 `CMSIS_RTOS_V2` wrapper，并更新 CMake/Keil 路径。
- 增加 `osKernelInitialize()`，修正 SysTick 到 FreeRTOS tick 的转发。
- 消除 `blue_led_handle` 重复定义。

## 5. 遇到的错误

- 当前 shell 配置阶段找不到交叉编译器，改为复用已有 Debug 构建目录。
- V1 专用 `osSystickHandler()` 在 V2 中不存在。
- 旧 `FreeRTOSConfig.h` 缺少 V2 wrapper 要求的配置宏。
- wrapper 默认定义 `SysTick_Handler`，与 HAL handler 冲突。
- 旧任务文件为 GBK 编码，迁移过程中需要保持原编码。

## 6. 根本原因

CMSIS-RTOS V2 不只是 API 名称变化，还改变了内核初始化、优先级模型、配置契约和 SysTick 集成方式。旧工程来自 CubeMX 5.2.1，不能仅替换头文件完成迁移。

## 7. 调试过程

按编译器证据逐步处理：先迁移 API，再补 FreeRTOSConfig 契约，最后使用官方自定义 SysTick 开关解决链接冲突。每轮都重新执行增量构建。

## 8. 经验总结

- V1 的 128 words 栈迁移到 V2 时应写成 512 bytes。
- V2 创建对象前必须调用 `osKernelInitialize()`。
- HAL 与 FreeRTOS 共用 SysTick 时，应启用 `USE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION`。
- 厂商旧工程文件修改前需要先识别文本编码。

## 9. 验证

- CMake Debug 编译和链接成功。
- ELF: `build/Debug/Infantry_Robot.elf`，1,139,008 bytes。
- RAM: 35,208 B / 128 KiB（26.86%）。
- Flash: 17,232 B / 1 MiB（1.64%）。
- ELF 包含 `osThreadNew`，不包含 `osThreadCreate`。
- V2 wrapper SHA-256 与本机 STM32CubeF4 V1.28.3 官方文件一致。

## 10. 已知限制

尚未进行开发板实机验证和 Keil 编译。仓库 `arch-check.ps1` 存在既有解析错误，HW 检查器会把 PendSV/SysTick 共用最低优先级误判为冲突。

