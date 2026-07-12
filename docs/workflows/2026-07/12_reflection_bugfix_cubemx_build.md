# CubeMX 重生成后 CMake 构建修复记录

**日期**: 2026-07-12
**报告类型**: reflection
**提交类型**: bugfix

## 1. 概述

记录 CubeMX 重新生成 CAN1、TIM6 HAL 时基和 CMSIS-RTOS V2 工程后，CMake Debug 构建因用户源文件列表被替换而失败的定位与修复过程。

## 2. 用户需求

解决 VS Code 附件日志中的构建报错，保留现有 CubeMX 配置、`User/task` 和 LED 示例任务，并提交相关改动。

## 3. 工作流记录

先运行 auto-embedded 机械门禁，再读取完整构建日志和 CMake 配置；按首个真实错误修复后执行增量构建、干净构建、ELF 符号检查、硬件资源核对和关键代码人工审查。

## 4. 修改内容

- 根 `CMakeLists.txt` 同时纳入 `User/task` 与 `applications`，恢复 LED 任务头文件路径。
- 移除 VS Code 中与 `Debug` 预设重复的 CMake 参数和生成器覆盖。
- 记录当前 CubeMX 生成的 CAN1、TIM6 HAL 时基、CMSIS-RTOS V2 核心文件和实际使用的固件包文件。
- 将 PD0/PD1、CAN1_RX0_IRQn、TIM6_DAC_IRQn 和 TIM6 写入硬件资源锁。

## 5. 遇到的错误

- `Src/main.c:29:10: fatal error: red_led_task.h: No such file or directory`。
- VS Code 提示使用 `Debug` 预设时仍应用工作区覆盖。
- auto-embedded 的 `arch-check.ps1` 存在既有解析错误；硬件检查器会把允许共享最低优先级的系统异常和 TIM6 误判为冲突。

## 6. 根本原因

新增 `User/task` 时，根 CMake 中原有 `applications` 源文件及包含目录被替换，而 `main.c` 和 `freertos.c` 仍引用 LED 任务。CubeMX 重新生成还将旧版厂商模板更新到当前固件包版本，使工作区出现大量与本次构建错误无关的库文件差异。

## 7. 调试过程

1. 从附件和本地构建中确认首个失败点是缺少 `red_led_task.h`。
2. 对比 Git 基线，发现 `applications` 被 `User/task` 替换。
3. 将两组源文件合并到同一目标，恢复 `applications` 包含路径。
4. 删除与 `CMakePresets.json` 重复的 VS Code 覆盖设置。
5. 执行增量构建和 `--clean-first` 干净构建，并检查最终 ELF 符号。

## 8. 经验总结

- 向 CMake 增加新业务目录时应追加源文件和包含路径，不能覆盖现有列表。
- CubeMX 生成目录与用户代码目录要分开维护；提交时仅纳入目标实际使用的厂商文件，避免未使用库产生噪声。
- `Keep User Code` 只保护匹配的用户代码段，构建系统和模板升级仍需通过 Git 差异复核。

## 9. 测试与验证

- `cmake --build build/Debug --clean-first --`：44/44 编译链接成功，返回码 0。
- FLASH：21176 B（2.02%）；RAM：35328 B（26.95%）。
- ELF 包含 `SysTick_Handler`、`TIM6_DAC_IRQHandler`、`HAL_InitTick` 和三个 LED 任务入口。
- `ctest` 未发现测试项；未进行开发板烧录与实机运行验证。

## 10. 已知限制

机械门禁未全通过，因此未执行 auto-embedded promote/归档。失败项为检查工具自身解析错误和共享中断优先级误报，不是本次固件编译错误。
