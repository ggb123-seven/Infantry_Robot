# task 与 module 精简计划

记录日期：2026-10-09。本文档只是计划，尚未修改任何源码。

## 现状中的冗余

- `User/task/ozone_debug.c/h` 只做转存：`g_motor_chassis_monitor` 从底盘快照和 CAN 快照逐字段拷贝，`g_fault_detect_monitor` 是 `fault_detect_snapshot` 的整份副本。不再使用 Ozone 后，这一层只剩额外拷贝和三个函数。
- `Chassis_Snapshot_t` 混合了三类数据：初始化结果（`initialized`、`mixer_init_status`、`motor_init_status`）、每周期运行结果、仅调参时查看的中间量（`limited_target_speed_rpm`、`ramped_target_speed_rpm`、逐路 `control_status`）。
- `Chassis_Output_t` 只有 `current_command_a`，与快照中的同名字段重复。
- PID 覆盖机制已过时：ID4 参数已固化为宏，但 `Chassis_Input_t` 仍保留 `pid_tune_override_enable` / `pid_tune_override`，`Task_motor_chassis` 中还有约 50 行零值初始化和覆盖映射逻辑。
- `FaultDetect_Snapshot_t` 中的 `motor_fault_count`、`offline_motor_count` 等计数可由故障位推导，属于调试辅助信息。
- `gimbal` 同时对外提供 `Gimbal_YawSpeedRun` 与 `Gimbal_YawSpeedRunAt`，任务只调用后者，前者仅被后者内部调用，可改为 `static` 并合并接口。

## 实施步骤

1. 删除 `ozone_debug.c/h`。各任务把自身状态集中为一个非 `static` 全局结构体，并在各自头文件中 `extern`，调试时直接查看。同步移除 `CMakeLists.txt` 中的源文件条目，Keil 工程如有引用一并移除。
2. 底盘 PID 改为每路一组参数：`Chassis_Input_t` 只保留四组 PID 参数，删除覆盖开关、电机编号和覆盖数组；默认值由宏初始化，ID4 使用固化参数。
3. 拆分 `Chassis_Snapshot_t`：运行必需字段保留，初始化结果和中间目标值移入调试子结构体，由调试宏控制是否编译；删除 `Chassis_Output_t`，电流直接从快照读取。
4. `FaultDetect_Snapshot_t` 的计数字段纳入同一调试宏，正常编译只保留故障位和 `has_fault`。
5. `gimbal` 将 `Gimbal_YawSpeedRun` 收为内部函数，只对外保留带时间戳的接口；`GimbalYawSpeed_t` 拆分为可写参数与只读反馈两组，字段名沿用用户原定命名。
6. 调试宏仅在 CMake Debug 预设中开启，Release 默认关闭。

预期收益主要是职责清晰、文件变薄；F407 RAM 充足，内存节省很少。

## 待用户确认

- 第 2～5 步需要修改 `User/module`，超出 `AGENTS.md` 默认授权范围；第 1 步可单独先做。
- 新增的全局调试结构体与调试子结构体的名称和字段，需先征询用户设计。
- 调试字段处理方式：用宏裁剪（Release 不可见），或常驻但集中到子结构体（正式固件也可查看）。

## 验证要求

每一步完成后执行 Debug 构建，并按 `AGENTS.md` 检查花括号、行宽、中文注释、Doxygen 完整性和分层依赖。
