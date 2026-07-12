# 研究发现

| 关键词 | 来源 | 摘要 | 可信度 | 状态 |
|---|---|---|---|---|
| 芯片与固件包 | `Infantry_Robot.ioc` | STM32F407IGH6，STM32Cube FW_F4 V1.28.3 | 高 | 已确认 |
| RTOS | `FreeRTOSConfig.h`、生成 CMake | CMSIS-RTOS V2，FreeRTOS 软件定时器启用 | 高 | 已确认 |
| 构建环境 | 用户附件、`build/Debug/CMakeCache.txt` | Ninja，GNU Arm Embedded 14.3.1，Debug | 高 | 已确认 |
| 首个编译错误 | 用户附件及本地复现 | `Src/main.c:29` 找不到 `red_led_task.h` | 高 | 已复现 |
| 根因 | 根 `CMakeLists.txt` 与 Git 差异 | 加入 `User/task` 时替换了原有 `applications` 源文件和头文件目录 | 高 | 已定位 |
| VS Code 驱动提示 | `.vscode/settings.json`、`CMakePresets.json` | 使用 `Debug` 预设时又设置了重复的配置参数与生成器覆盖 | 高 | 已定位 |
| 硬件影响 | `.ioc` 与本次修改范围 | 本次仅修复 CMake，不新增或变更硬件资源 | 高 | 已确认 |

## 修复策略

- 在根 `CMakeLists.txt` 中保留 `User/task` 三个源文件。
- 恢复 `applications` 下三个 LED 任务源文件及其头文件搜索路径。
- 保留 `cube-cmake` 可执行路径，移除与预设重复的 VS Code 配置参数和生成器覆盖。
- 使用现有 `build/Debug` 配置增量构建，按首个后续错误继续收敛。

## 验证结果

- `cmake --build build/Debug --clean-first --`：44/44 编译及链接成功，返回码 0。
- 固件产物：`build/Debug/Infantry_Robot.elf`，SHA-256 为 `71BCACE5A314EB25A5AC21B43D48883B509F4D0C2D84F36EB5698BB1993050F7`。
- 内存占用：FLASH 21176 B（2.02%），RAM 35328 B（26.95%）。
- ELF 符号存在：`SysTick_Handler`、`TIM6_DAC_IRQHandler`、`HAL_InitTick` 和三个 LED 任务入口。
- `ctest --test-dir build/Debug --output-on-failure`：工程未定义主机测试，未发现测试项。
- 硬件人工核对：PD0/PD1、CAN1_RX0_IRQn、TIM6_DAC_IRQn 和 TIM6 与当前 `.ioc` 一致。
- 代码质量核对：本次未修改 C 源码；`main.c` 仍只负责初始化与调度，TIM6/CAN 中断入口仅转发 HAL；未新增 ISR/任务共享状态，因此没有新增 `volatile` 或临界区需求。

## 门禁限制

- `SPEC` 通过。
- `ARCH` 未执行到规则检查：`arch-check.ps1` 第 279 行起存在既有 PowerShell 解析错误。
- `HW` 检查器把 PendSV、SysTick 与 TIM6 共用最低优先级 15/0 判为冲突；三者资源标识不同，共用优先级不构成 NVIC 资源重复，未通过修改真实优先级规避工具误报。
- 未进行开发板烧录与实机运行验证，本次验收范围为构建报错修复。
