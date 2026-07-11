# 编辑清单

| 文件 | 改动 | 验证标准 | 结果 | commit |
|---|---|---|---|---|
| `Infantry_Robot.ioc` | 用 CubeMX 6.17 迁移工程，选择 STM32CubeF4 V1.28.3 和 CMSIS-RTOS V2；保留现有硬件配置。`review:true` | `.ioc` 为 `CMSIS_V2`，MCU/时钟/7 个实体引脚和 RTOS IRQ 与迁移前一致 | 通过：V2/F4 V1.28.3；硬件字段未变 | - |
| `Src/freertos.c`、`Src/main.c`、`Src/stm32f4xx_it.c` | 合并 CubeMX V2 生成结果；任务句柄改为 `osThreadId_t`，任务改用 `osThreadNew`，栈为 512 bytes，消除 `blue_led_handle` 重复定义。`review:true`，L6 App/CubeMX generated | 无 V1 include/API；3 个线程入口和优先级保持；单一定义 | 通过：ELF 含 `osThreadNew` 和 3 个属性符号，无 `osThreadCreate` | - |
| `Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/*`、必要 CMSIS RTOS2 头文件 | 使用 STM32CubeF4 V1.28.3 官方 V2 wrapper，移除构建对旧 wrapper 的依赖。`review:true`，L4 Middleware | 文件来源与本地官方包一致；编译无缺失符号 | 通过：4 文件 SHA-256 与本地官方包一致 | - |
| `cmake/stm32cubemx/CMakeLists.txt` | include/source 从 `CMSIS_RTOS` 切换到 `CMSIS_RTOS_V2`。`review:true`，构建配置 | CMake 配置成功且实际编译 `cmsis_os2.c` | 通过：`build.ninja` 引用 V2，Debug 构建成功 | - |
| `MDK-ARM/Infantry_Robot.uvprojx` | 在保留当前用户差异的前提下切换 V2 include/source。`review:true`，IDE 配置 | 用户原差异仍存在；工程不再引用 V1 `cmsis_os.c` | 配置通过；未调用 Keil 编译器验证 | - |
| 全工程验证 | 扫描残留 V1 API，运行 CMake Debug 构建并核对 ELF。`review:false` | V1 应用引用为 0；构建退出码 0；生成 `Infantry_Robot.elf` | 通过：ELF 1,139,008 bytes；RAM 26.86%，Flash 1.64% | - |
