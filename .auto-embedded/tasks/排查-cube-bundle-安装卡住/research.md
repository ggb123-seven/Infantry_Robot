# 研究发现

| 关键词 | 来源 | 摘要 | 可信度 | 状态 |
|---|---|---|---|---|
| Cube bundle 清单 | `.settings/bundles.store.json`、`.settings/bundles-lock.store.json` | 已解析 CMake 4.3.1、Ninja 1.13.2、GNU Tools for STM32 14.3.1、ST Arm Clangd 21.1.0、J-Link GDB Server 9.24.0。 | 高 | 已验证 |
| 工具链实际路径 | `build/Debug/CMakeFiles/4.3.1/CMakeCCompiler.cmake` | C 编译器来自 `%LOCALAPPDATA%/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin/arm-none-eabi-gcc.exe`。 | 高 | 已验证 |
| Debug 构建 | `py .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug -j 8` | 配置和构建成功，产物为 `build/Debug/Infantry_Robot.elf`，大小约 1054.5 KB。 | 高 | 已验证 |
| 项目门禁 | `py .auto-embedded/scripts/check.py` | HW 与 SPEC 通过；ARCH 因 `arch-check.ps1` 第 279 行附近的 PowerShell 解析错误未能执行，不是本次源码编译错误。 | 高 | 工具故障 |
| CubeMX 工程版本 | `Infantry_Robot.ioc` | 工程由 STM32CubeMX 5.2.1 / DB 5.0.21 保存，目标 MCU 为 STM32F407IGH6（UFBGA176），固件包字段为 `STM32Cube FW_F4 V1.24.1`。 | 高 | 已验证 |
| `.ioc` 与工具 bundle 的关系 | `Infantry_Robot.ioc`、`.settings/bundles.store.json` | `.ioc` 的 `FirmwarePackage` 是 STM32CubeF4 MCU 固件包；`.settings` 清单记录 CMake/Ninja/GCC/clangd/J-Link 工具 bundle。两类包用途不同，`.ioc` 本身没有工具 bundle 下载状态或卡住原因字段。 | 高 | 已验证 |
| 当前外设范围 | `Infantry_Robot.ioc` | 仅启用 RCC、SYS、NVIC、FreeRTOS；实体引脚为 HSE PH0/PH1、SWD PA13/PA14、LED PH10/PH11/PH12。未配置 UART、CAN、SPI、I2C、ADC、定时器或 DMA 外设。 | 高 | 已验证 |
| 时钟树 | `Infantry_Robot.ioc` | 外部 HSE=12 MHz，PLLM=6、PLLN=168、PLLQ=7；SYSCLK/HCLK=168 MHz，APB1=42 MHz（定时器 84 MHz），APB2=84 MHz（定时器 168 MHz），48 MHz 域为 48 MHz。 | 高 | 已验证 |
| FreeRTOS 配置 | `Infantry_Robot.ioc` | CMSIS-RTOS v1；两个动态任务：`LED_RED` 优先级 0、栈 128、weak 入口，`LED_GREEN` 优先级 2、栈 128、external 入口；PendSV/SysTick 抢占优先级均为 15。 | 高 | 已验证 |
| 兼容性风险线索 | `Infantry_Robot.ioc` | `MxCube.Version=5.2.1`、`ProjectManager.AskForMigrate=true`、`ProjectManager.LastFirmware=true` 同时存在。用较新 CubeMX 打开时预期会触发迁移/固件包解析，但是否造成安装卡住仍需 Cube 日志或界面状态佐证。 | 中 | 待取日志 |
| CMSIS-RTOS V1 由来 | `Infantry_Robot.ioc:26,156-157`、`Src/freertos.c` | `.ioc` 明确选择 `FREERTOS_VS_CMSIS_V1`；生成代码使用 `cmsis_os.h`、`osThreadDef`/`osThreadCreate`，因此 V1 是原工程的显式配置，不是检测误判。 | 高 | 已验证 |
| V1/V2 与内核版本 | `Inc/FreeRTOSConfig.h`、`Middlewares/Third_Party/FreeRTOS/Source/include/task.h` | 当前 FreeRTOS Kernel 为 V10.0.1；CMSIS-RTOS V1/V2 是上层统一 API/wrapper 版本，不等同于 FreeRTOS 内核主版本。 | 高 | 已验证 |
| V2 迁移影响 | `Src/freertos.c`、`Src/main.c`、`Src/stm32f4xx_it.c`、`Middlewares/Third_Party/FreeRTOS/Source/st_readme.txt` | 当前源码依赖 V1 API；切换 V2 会涉及 `cmsis_os2.h`、`osThreadNew` 等接口及生成文件变化。V2 对新开发的 API 能力更完整，但对现有 V1 源码不是直接向后兼容替换。 | 高 | 已验证 |
| 本机 CubeMX 版本 | `C:/Program Files/STMicroelectronics/STM32Cube/STM32CubeMX/STM32CubeMX.exe` 文件版本信息 | 已安装 STM32CubeMX `6.17.0-RC5`，明显新于 `.ioc` 记录的 5.2.1；当前无需额外下载安装 CubeMX。 | 高 | 已验证 |
| 本地 STM32CubeF4 包 | `C:/Users/seven/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3` | 本机已有 STM32CubeF4 V1.28.3；包含 `Drivers/CMSIS/RTOS2/Include/cmsis_os2.h` 与 `Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c/.h`，具备生成/构建 CMSIS-RTOS V2 的文件基础。 | 高 | 已验证 |
| 执行阶段约束 | 当前 workflow-state | 用户已授权升级/切换，但 active task 仍处于 RESEARCH，阶段规则禁止修改 `.ioc`、源码和执行安装；需先进入允许变更的阶段再生成迁移。 | 高 | 阶段阻塞 |
