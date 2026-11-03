# 云台公开结构精简验证

日期：2026-11-03

## 改动

- `Gimbal_Axis_t` 只公开运行状态、编码器与机械角、惯性角与角速度、实际目标和输出电流
- `base_speed_rad_s` 与 PID、设定值、角度展开历史、反馈时间戳、方向应用副本等内部字段移入 `gimbal.c` 私有状态
- `Gimbal_t` 保留可在线修改的 `config`、`config_valid`、`mode`、两轴观测和 `imu_state`
- `Gimbal_UpdateFeedback` 使用最近一次由 `Gimbal_Control` 接受的配置快照；PID 长期参数指针继续指向模块内持久副本
- `User/module/config.c`、底盘源码、设备与任务代码均未修改

## 验证

- `cmake --build build/Debug --parallel 4`：通过，ARM Debug 全量链接成功；RAM 47112 B，FLASH 125792 B
- 从 `build/Debug/compile_commands.json` 取 `gimbal.c` 的 ARM 编译参数，追加 `-Wextra -Werror -fsyntax-only`：通过，零警告
- `powershell -NoProfile -ExecutionPolicy Bypass -File .auto-embedded/scripts/arch-check.ps1`：通过，0 violations
- `py .auto-embedded/scripts/check.py --hw`：通过
- `py .auto-embedded/scripts/check.py --spec`：通过
- `git diff --check` 及 120 列检查：通过，修改文件无尾随空白、无超宽行
- 全仓源码引用扫描：已删除字段无模块或任务代码残留引用

## 信任边界自查

- 本次未新增判空、`isfinite`、范围检查或保护性 `return`
- EmberProbe 配置仍在每周期 `Gimbal_Control` 入口复制、校验并更新 `config_valid`；反馈处理沿用最近有效快照
- 无删除或重复检查项

## 底盘结构筛查

- `measured_speed_rpm`、`filtered_speed_rpm`、`requested_speed_rpm`、`target_speed_rpm` 分别表示采样、滤波、运动学请求和斜坡/限幅后的控制目标，保留有助于定位控制链阶段，不属于重复字段
- `speed_pid`、`speed_filter`、`current_filter` 属于单轮算法运行态；`applied_wheel_speed_pid` 和 `mixer` 属于模块内部同步/算法状态，可作为后续移出调试公开结构的候选
- `config` 与 `applied_wheel_speed_pid` 当前分别承担 EmberProbe 可写副本和 PID 长期引用的已应用副本，不能直接合并，否则在线写入与 PID 参数指针的周期一致性会改变
- 本轮仅筛查底盘，未修改底盘结构

## 验证边界

- 本轮为结构可见性收敛与软件构建验证，未烧录、未上板运行云台或执行电机测试
- `check.py --arch` 包装调用受 PowerShell 脚本签名策略阻挡；直接以进程级 `ExecutionPolicy Bypass` 运行同一 `arch-check.ps1` 后通过
