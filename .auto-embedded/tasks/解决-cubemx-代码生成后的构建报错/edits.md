# 编辑清单

- `CMakeLists.txt`：保留 `User/task`，恢复三个 `applications` LED 任务源文件和头文件路径。
- `.vscode/settings.json`：移除与 `Debug` 预设重复的配置参数及生成器覆盖，保留 `cube-cmake`。
- `.auto-embedded/spec/hardware/hw-lock.yaml`：同步当前 `.ioc` 已有的 CAN1 与 TIM6 HAL 时基资源。
- `cmake/gcc-arm-none-eabi.cmake`：自动查找当前环境、显式根目录或 STM32Cube Bundle 中的 GNU Arm 工具链。
- `.auto-embedded/scripts/arch-check.ps1`：修复 UTF-8、PowerShell 5 路径兼容和 `build/` 误扫描问题。
- `.auto-embedded/scripts/aemb_core.py`、`arch-check.sh`：只按硬件资源标识判重，允许不同 IRQ 使用相同优先级。

| 文件 | 改动 | 验证标准 | 结果 | commit |
|---|---|---|---|---|
| `CMakeLists.txt` | 同时纳入 `User/task` 与 `applications` | Debug 干净构建通过 | 通过，44/44 | 本次提交 |
| `.vscode/settings.json` | 删除预设重复覆盖 | JSON 有效且仅保留 `cube-cmake` 路径 | 静态检查通过，需 VS Code 重载确认提示消失 | 本次提交 |
| `hw-lock.yaml` | 同步 CAN1 与 TIM6 资源 | 与 `.ioc` 逐项一致 | 人工核对通过 | 本次提交 |
| `cmake/gcc-arm-none-eabi.cmake` | 增加 GNU Arm 工具链自动发现 | 不设置 `PATH` 时完成 Debug 干净构建 | 通过，自动选用 STM32Cube Bundle 14.3.1 | 待提交 |
| `arch-check.ps1`、`arch-check.sh`、`aemb_core.py` | 修复 ARCH 执行兼容性、生成初始化误计数和 HW 优先级误报 | 生成初始化调用不计数；自定义调用仍受上限约束；重复 IRQn 回归用例仍能检出 | ARCH、HW 门禁通过 | 待提交 |
