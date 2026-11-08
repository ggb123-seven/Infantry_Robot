# 云台电流限幅调整验证

日期：2026-11-08

用户指定现阶段云台电机电流限幅为 1 A，本次据此定向更新 `User/module/config.c` 中的云台电流配置

## 修改范围

- yaw、pitch 的 `max_current_a` 从 0.5 改为 1.0 A
- 保留用户在任务期间更新的 PID 配置，两轴 `speed_pid.out_limit` 当前均为 10.0 A，最终仍经 `max_current_a` 限制为 ±1 A
- 同步更新集中参数说明，不调整 PID 增益、积分限幅、机械零点或机械范围
- 开始任务前已有的配置改动、硬件 spec 改动、采样文件和其他验证记录均保留，不纳入本次提交

## 验证证据

- `python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --list-presets`：列出 Debug 和 Release
- `python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug`：初次配置、编译成功；用户同步更新 PID 后再次编译通过
- 生成器：Ninja；构建目录：`F:/RM/Infantry_Robot/build/Debug`
- `artifact_path`：`F:/RM/Infantry_Robot/build/Debug/Infantry_Robot.elf`；`artifact_kind`：`elf`
- `python .auto-embedded/scripts/check.py`：ARCH、HW、SPEC 全部 PASS；本进程设置 `PSExecutionPolicyPreference=Bypass` 使子进程可执行仓库检查脚本，未更改系统执行策略
- `git diff --check` 通过；针对 `config.c` 的 UTF-8、120 列、行尾空白和 Allman 扫描通过

## 人工复核

- 配置仍由任务传入模块 Init 并复制到 RAM，Control 入口仍按周期复制快照、集中校验和同步 PID 参数
- 速度 PID 结果在模块计算链末端按 `max_current_a` 限幅，`Gimbal_DumpOutput` 仍按轴运行状态唯一放行，否则输出零
- 当前工程 GM6020 使用电流协议，底层 ±3 A 对应 ±16384；1 A 在现有整数换算中对应约 5461 个控制码
- 本次仅修改常量与集中说明，未新增依赖、RTOS/HAL 调用、ISR 共享状态、临界区、结构体或状态机
- 防御性检查自查：新增判空 0、`isfinite` 0、范围检查 0、保护性 `return` 0；无新增下游重复校验，无需删除项

## 验证边界

本次编译验证使用包含用户原有调参改动的当前工作树；本地提交仅包含本次限幅差异和验证文档

尚未烧录或进行实机电流测量；该配置限定软件电流指令，不等同于已测得实际电流峰值
