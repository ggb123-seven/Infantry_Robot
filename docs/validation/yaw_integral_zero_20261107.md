# yaw 双环关闭积分对照配置

- 用户明确要求直接修改 config 中的 yaw 双环积分系数，本次授权覆盖 config.c 默认禁止修改规则中的这两项
- `gimbal_config.yaw.angle_pid.i` 从 0.005 改为 0，`gimbal_config.yaw.speed_pid.i` 从 0.177 改为 0
- 同步更新配置上方集中说明；原积分限幅和其他参数保留，包含工作区已有的 pitch 机械零点与限位改动
- 初始化仍复制到 RAM config，周期快照和 PID 参数同步路径不变；本次未写目标 RAM、未烧录

## 验证证据

- `python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug` 配置和构建通过
- 产物：`build/Debug/Infantry_Robot.elf`，构建包含当前工作区已有的 pitch 参数改动
- 当前 PowerShell 进程设置 `PSExecutionPolicyPreference=Bypass` 后执行 `python .auto-embedded/scripts/check.py`，ARCH、HW、SPEC 全部通过，未修改系统执行策略
- `git diff --check` 通过；配置文件扫描 120 列、行尾空白和左花括号格式通过
- 对比修改前工作区备份，去除注释后的差异严格只有 yaw 两个积分系数
- 人工复核：未新增结构体、控制分支、函数或接口；分层依赖、RTOS/HAL 对象创建、ISR 共享状态、临界区、main 和 DumpOutput 行为不变
- 新增判空、isfinite、范围检查、保护性 return 均为零；删除项为零，无新增重复校验
- 本次是积分归零对照固件准备完成，尚未验证实机跳动是否消失，不能据此确认根因或最终参数
- 未创建临时测试源码或测试可执行文件；用于差异核对的临时备份在提交前清理
