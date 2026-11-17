# 拨弹遥控默认单发

日期：2026-11-17

## 变更与方向核对

- `User/task/motor_trigger.c` 的 `feeder_remote_fire_mode` 从 `FEEDER_FIRE_MODE_BURST` 改为 `FEEDER_FIRE_MODE_SINGLE`，同步任务头文件集中说明
- 复用现有命令和拨弹状态机：右拨杆下位请求一齿，实际到位后撤驱，持续下位不重复；回中取消或释放后可再次触发，在线模式切换仍保留
- 当前 `feeder_config.direction=-1`，CAN2 ID7 M2006 的 `reverse=false`，工作树与 HEAD 一致，没有检测到待提交的拨弹电机方向差异；保持现有方向，不自行反转
- `User/module/config.c` 已有的工作区改动属于 yaw 摩擦补偿，与本次无关，不纳入提交；其他既有文档变更和删除不纳入提交

## 验证证据

- 修改后逐文件重读 `AGENTS.md` 与 `agent.md`；新增行通过 120 列、Allman 花括号、中文注释和行尾空白检查，`git diff --check` 通过
- 使用 `build/Debug/compile_commands.json` 中的 ARM 编译选项，对 `motor_trigger.c` 添加 `-fsyntax-only -Wextra -Werror`，退出码 0
- 使用 `aemb-build-cmake` 脚本执行 Debug 配置与构建，退出码 0，产物为 `build/Debug/Infantry_Robot.elf`，生成器为 Ninja；本次构建包含工作树中原有的未提交 yaw 配置
- GDB 离线读取 ELF：`feeder_remote_fire_mode=1`、`feeder_config.direction=-1`、`can_devices_shoot_param[2].reverse=false`，确认产物的初始模式与方向
- `python .auto-embedded/scripts/check.py` 的 ARCH、HW、SPEC 全部通过；仅对子进程使用 `PSExecutionPolicyPreference=Bypass`

## 人工检查与验证边界

- 只修改任务层的模式初值和对应说明，复用 `Cmd_ParseFeeder` 与 `Feeder_Control`；总线仍由 CAN 任务统一输出，`Feeder_DumpOutput` 保持状态输出闸门
- 无新增结构体、条件分支、判空、`isfinite`、范围检查、保护性 return、RTOS 对象、ISR 共享状态或临界区；无新增或删除的防御校验项
- 默认值仍为 volatile 在线入口，任务每周期读取一次；没有新增任务栈变量或调用层级
- 拨弹任务静态栈 1024 B；本次构建 `.su` 显示 `Task_motor_trigger=192 B`、`Feeder_Control=216 B`、`Feeder_UpdateSingle=32 B`、`PID_Calc=72 B`
- 未取得运行时栈高水位，未烧录或进行真实机构单发验收；编译与 ELF 初值检查不代表机械到位、弹丸出膛或完整运行时栈峰值已验证
- 本次没有创建临时测试源码或可执行程序
