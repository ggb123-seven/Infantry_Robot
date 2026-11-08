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

## 再次同步电流与控制码

用户选中电流换算表后要求同步，本轮以此前确定的 1 A 为最终电流上限，将工作树中两轴的 `max_current_a` 从 0.5 恢复为 1.0，并在集中说明补充向零取整后的对应关系

| 电流 | 控制码 |
| --- | ---: |
| ±0.5 A | ±2730 |
| ±1 A | ±5461 |
| ±3 A | ±16384 |

- 保留用户当前 PID 配置，两轴速度 PID 输出上限仍为 10 A 候选值，由现有模块末端限幅至 ±1 A
- 继续使用设备层既有安培换算和整数转换，不新增重复的控制码限幅
- 同步修正配置上方已经过时的固定增益说明，以两轴独立参数和周期快照约定描述当前配置
- `python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug`：配置、编译成功，耗时 1.6 秒
- `python .auto-embedded/scripts/check.py`：ARCH、HW、SPEC 全部 PASS
- UTF-8、120 列、Allman、行尾空白扫描及 `git diff --check` 通过
- 已复核模块末端限幅、DumpOutput 零输出闸门、设备层 ±3 A 量程与整数转换链路；本轮不改变硬件资源、ISR 共享状态、临界区或 RTOS/HAL 调用
- 防御性检查自查仍为新增判空 0、`isfinite` 0、范围检查 0、保护性 `return` 0，无新增下游重复校验，无需删除项
- 编译时 `config.c` 的 SHA256：`f1a05d825ae083876fb0622d922a673746a836f72d555765c631bb6a4ff4ebf9`
- 源码中的两轴 1 A 已存在于此前提交 `94e26d4`，本次本地提交仅包含新同步的集中说明及验证证据，用户其他调参继续保留在工作树中
- 本轮未创建临时测试程序，尚未烧录或实测电流
