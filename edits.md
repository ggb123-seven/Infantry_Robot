# 修改记录

## CMAKE-PATH-20260721-01

- 目标：修正 VS Code CMake Tools 对不存在的 `cube-cmake` 的引用。
- 配置改动：将 `.vscode/settings.json` 中的 `cmake.cmakePath` 指向系统 CMake 3.31.4，并移除错误的 `CMAKE_COMMAND=cube-cmake` 配置参数。
- 验证：JSON 解析通过且配置中已无 `cube-cmake`；系统 CMake 3.31.4 完成 `Debug` 预设配置，增量构建返回 `ninja: no work to do.`。
- 状态：已完成。

## MOTOR-FEEDBACK-DIAG-02

- 目标：修复 `Task_Init` 初始化任务栈不足。
- 源码改动：将 `User/task/user_task.c` 中 `attr_init.stack_size` 从 1024 字节增至 4096 字节，并同步修正任务属性集中说明。
- 依据：Debug 构建的 `.su` 报告显示最深静态初始化链约 2080 字节，原配置和 2048 字节中间配置均不足以覆盖该链路并保留中断余量。
- 验证：源码规范扫描通过；主机侧 `init_task_test` 输出 `Init task tests passed`；`build/Debug` ARM Debug 构建成功；新 ELF 中 `attr_init.stack_size` 为 `0x00001000`（4096 字节）。
- 资源影响：只在 `Task_Init` 存活期间从 15360 字节 FreeRTOS heap 额外占用 3072 字节；任务退出后由 Idle 任务回收，不改变其他任务的优先级、周期或栈配置。
- 残余风险：仍需在目标板确认初始化峰值 heap 和实际栈水位，并验证 CAN 反馈链是否恢复。
- 限制：当前未连接 GDB Server/目标板，无法读取运行态栈水位，也无法确认刷写后 `feedback_update_status` 是否仍为 `-4`。
- 状态：软件验证完成，待烧录新 ELF 进行上板验证。
