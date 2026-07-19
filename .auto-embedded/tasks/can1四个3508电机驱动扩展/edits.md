# 编辑清单

| 文件 | 改动 | 验证标准 | 结果 | commit |
|---|---|---|---|---|
| `User/task/motor_chassis.h` | 将 Ozone 调参和监视结构扩展为 4 电机数组 | 每台具有独立目标、反馈、状态和电流指令 | 已完成 | - |
| `User/task/motor_chassis.c` | 注册/Update/Control/SetTorqueCurrent 扩展为 4 实例，末尾统一 Flush | `0x201~0x204` 映射正确，每周期一次 `0x200` 发送 | 已完成 | - |

## 验证证据

| 验证项 | 命令 / 证据 | 结果 |
|---|---|---|
| Debug 固件构建 | `py .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug --jobs 8` | 成功，产物 `build/Debug/Infantry_Robot.elf` |
| 机械门禁 | `py .auto-embedded/scripts/check.py --json` | `ok: true`，arch/hw/spec 均为 `rc: 0` |
| 组帧静态核对 | 检查任务源码中 ID 与发送 API 调用数 | ID 为 `0x201~0x204`，`FlushGroup` 1 处，直接 `Ctrl` 0 处 |
| 代码排版 | 检查 120 列行宽、Allman 花括号和 `git diff --check` | 通过 |
| 上板实测 | 本次未执行烧录和实车运转 | 待确认电调 ID、安装方向和四路反馈 |
