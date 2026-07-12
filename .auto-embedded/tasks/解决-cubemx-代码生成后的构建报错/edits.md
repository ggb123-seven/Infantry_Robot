# 编辑清单

- `CMakeLists.txt`：保留 `User/task`，恢复三个 `applications` LED 任务源文件和头文件路径。
- `.vscode/settings.json`：移除与 `Debug` 预设重复的配置参数及生成器覆盖，保留 `cube-cmake`。
- `.auto-embedded/spec/hardware/hw-lock.yaml`：同步当前 `.ioc` 已有的 CAN1 与 TIM6 HAL 时基资源。

| 文件 | 改动 | 验证标准 | 结果 | commit |
|---|---|---|---|---|
| `CMakeLists.txt` | 同时纳入 `User/task` 与 `applications` | Debug 干净构建通过 | 通过，44/44 | 本次提交 |
| `.vscode/settings.json` | 删除预设重复覆盖 | JSON 有效且仅保留 `cube-cmake` 路径 | 静态检查通过，需 VS Code 重载确认提示消失 | 本次提交 |
| `hw-lock.yaml` | 同步 CAN1 与 TIM6 资源 | 与 `.ioc` 逐项一致 | 人工核对通过 | 本次提交 |
