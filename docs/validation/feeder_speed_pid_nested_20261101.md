# 拨弹速度环调试字段嵌套验证

日期：2026-11-01；对比基线：a68efce

## 用户确认的字段位置

- `feeder.speed_pid.requested_speed_rpm`：可写的输出轴速度请求，上电为 0，正负指定方向
- `feeder.speed_pid.target_speed_rpm`：限幅后的内环目标，同时保留前馈差分历史
- `feeder.speed_pid.measured_speed_rpm`：实际输出轴 rpm，无效反馈为 NaN
- `feeder.speed_pid.controller`：原有通用 PID 运行数据，参数指针指向持久 `feeder.cycle_config.speed_pid`
- 在线增益仍修改 `feeder.config.speed_pid`，控制入口仍复制并校验本周期配置快照

## 修改范围

修改 `User/module/feeder.c/.h` 的字段组织与引用，同步 `User/task/motor_trigger.c/.h` 中的调试说明
通用 `KPID_t`、`config.c`、BSP、设备层、CubeMX 配置和遥控映射保持原样

## 验证证据

- `python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug`：构建通过，产物 `build/Debug/Infantry_Robot.elf`
- ARM 严格检查：复用 `build/Debug/compile_commands.json`，对 `feeder.c`、`motor_trigger.c` 添加 `-Wextra -Werror -fsyntax-only`，均通过
- 四份修改源码格式检查通过：Allman、控制块花括号、120 列、行尾空白和注释末尾
- 主机 GCC 使用真实 feeder、config、PID、filter、user_math 编译基线与当前版本，共享同一测试输入，仅用 CAN 类型替身隔离 HAL
- 每版 8031 项断言，覆盖 1000 周期正负目标、限幅、反馈、停机保留请求、10 ms 超时边界、恢复、NaN/Inf、在线增益更新及前馈
- 两版状态、三个 rpm 字段、电流和 PID 误差逐位轨迹完全相同；轨迹 SHA-256 见同名 JSON
- 测试夹具曾遇到本机 MinGW 不接受 `%a` 以及头文件优先级问题，已改为输出浮点原始位并隔离替身目录，未为测试修改正式底层源码
- `python .auto-embedded/scripts/check.py`：提交前复查 ARCH/HW/SPEC 全部 PASS，`git diff --check` 通过

## 检查自查

本次生产代码未新增或删除判空、`isfinite`、范围检查、保护性 `return` 或兜底赋值，无新增检查条目和重复校验删除项
仅迁移字段路径，控制状态机、配置入口、PID 参数寿命、任务调度、RTOS/HAL 对象和 ISR 共享方式均未变化
DumpOutput 仍为唯一电流输出闸门，停机、失联和故障输出零

## 验证边界

本轮未烧录、未写目标 RAM、未启动真实电机；新字段布局需配套本次固件与 ELF
临时测试目录由 TemporaryDirectory 清理，临时验证脚本在证据保存后删除，保留本记录与 JSON
