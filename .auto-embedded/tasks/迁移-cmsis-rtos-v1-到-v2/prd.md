# 迁移 CMSIS-RTOS V1 到 V2

## 需求 / 验收标准
- `Infantry_Robot.ioc` 迁移到本机 STM32CubeMX 6.17 可维护格式，并将 FreeRTOS 接口切换为 CMSIS-RTOS V2。
- 3 个 LED 任务继续存在，优先级语义保持不变，原 128 words 栈转换为等效 512 bytes。
- 应用代码不再包含 `cmsis_os.h`，不再调用 `osThreadDef` / `osThreadCreate`，改用 `cmsis_os2.h` / `osThreadNew`。
- CMake 构建使用 `CMSIS_RTOS_V2/cmsis_os2.c`，Debug 配置编译和链接成功。
- Keil 工程引用 V2 wrapper，同时保留用户现有的 `MDK-ARM/Infantry_Robot.uvprojx` 工作区改动。
- `blue_led_handle` 只保留一个定义，消除重复符号风险。
- 不改变 MCU、时钟树、GPIO、SWD、HSE、IRQ 优先级等硬件配置。

## 约束
- 只使用本机已安装的 STM32CubeMX 6.17.0-RC5 和 STM32CubeF4 V1.28.3，不下载未经确认的第三方文件。
- 生成前保存精确 Git 差异；不得覆盖或回退用户已有改动。
- 每轮只处理一个迁移边界，发现 CubeMX 产生计划外硬件变化立即停止并回到 PLAN。
- 不自动 push；本任务不自动提交用户未确认的文件。
