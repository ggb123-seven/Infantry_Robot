# 研究发现

| 关键词 | 来源 | 摘要 | 可信度 | 状态 |
|---|---|---|---|---|
| 目标平台 | `Infantry_Robot.ioc` | STM32F407IGH6/UFBGA176，CubeMX 工程原版本 5.2.1，当前明确选择 CMSIS-RTOS V1。迁移不改变引脚、DMA、IRQ 或定时器占用。 | 高 | 已验证 |
| 可用生成环境 | 本机 `STM32CubeMX.exe`、`~/STM32Cube/Repository` | 本机 CubeMX 为 6.17.0-RC5，已有 STM32CubeF4 V1.28.3；固件仓库含 `CMSIS_RTOS_V2/cmsis_os2.c/.h`、`freertos_os2.h`，具备 V2 生成基础。 | 高 | 已验证 |
| 应用侧 V1 范围 | `Src/freertos.c`、`Src/main.c`、`Src/stm32f4xx_it.c` | 使用 `cmsis_os.h`、`osThreadId`、`osThreadDef`、`osThreadCreate`、`osDelay`、`osKernelStart`。业务对象仅 3 个 LED 线程，未发现消息队列、信号量、互斥锁、软件定时器等 V1 对象。 | 高 | 已验证 |
| 构建侧 V1 范围 | `cmake/stm32cubemx/CMakeLists.txt`、`MDK-ARM/Infantry_Robot.uvprojx` | CMake 和 Keil 均显式包含 `CMSIS_RTOS` 目录并编译 `cmsis_os.c`；迁移需切到 `CMSIS_RTOS_V2` 和 `cmsis_os2.c`。 | 高 | 已验证 |
| 栈单位差异 | CMSIS-RTOS V1/V2 API | 当前任务栈深度为 128 个 `StackType_t`；V2 `osThreadAttr_t.stack_size` 使用字节，应保持等效为 512 bytes（Cortex-M4 的 `StackType_t` 为 4 bytes），不能原样填 128 bytes。 | 高 | 已验证 |
| 既有重复符号 | `Src/freertos.c:58`、`Src/main.c:52` | `blue_led_handle` 在两处均为定义。迁移到 `osThreadId_t` 时应收敛为单一定义；这是既有代码问题，不是 V2 引入的问题。 | 高 | 已验证 |
| 用户工程改动保护 | `git diff -- MDK-ARM/Infantry_Robot.uvprojx` | Keil 工程文件存在非本轮产生的工作区修改；后续 CubeMX 生成或手工调整前必须保留并核对差异，禁止覆盖。 | 高 | 待执行保护 |
| 最终 CMake 构建 | `%LOCALAPPDATA%/stm32cube/bundles/cmake/4.3.1+st.1/bin/cmake.exe --build build/Debug --parallel 8` | 编译链接成功；`Infantry_Robot.elf` 1,139,008 bytes，RAM 35,208 B/128 KiB（26.86%），Flash 17,232 B/1 MiB（1.64%）。 | 高 | 已验证 |
| ELF API 证据 | `arm-none-eabi-nm build/Debug/Infantry_Robot.elf` | 存在 `osThreadNew`、3 个 `LED_*_attributes`、`SysTick_Handler`、`xPortSysTickHandler`；不存在 `osThreadCreate`。 | 高 | 已验证 |
| 门禁残留 | `py .auto-embedded/scripts/check.py` | ARCH 因 `arch-check.ps1` 第 279 行起解析错误无法运行；HW 将 PendSV/SysTick 同为原始最低优先级 15/0 判作冲突。两项均非本次迁移引入，SPEC 通过。 | 高 | 工具问题/已说明 |
