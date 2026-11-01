# 拨弹右摇杆速度映射验证

日期：2026-11-01，工作区：F:/RM/Infantry_Robot

## 需求与实现

- 用户要求：右拨杆拨到下方时，摇杆上下控制拨弹速度，中间到下面这一段行程映射 0～300 rpm
- 用户确认补充：上半程反向 0～-300 rpm，摇杆中点为零、推到底为 +300 rpm、推到头为 -300 rpm
- 用户确认实现落点：新增命令层 `Cmd_ParseFeeder`，`Feeder_Control` 不再读取遥控快照
- 用户确认结构：新增 `Feeder_Command_t`，字段为 `mode` 与归一化 `speed_ratio`；删除原可写调试字段 `feeder.speed_pid.requested_speed_rpm`
- `User/module/cmd.h/.c`：新增 `Feeder_CmdMode_t` 与 `Cmd_ParseFeeder`，右拨杆下位为运行、上位和中位为待机、非法档位为离线；速度比例复用既有 `Cmd_NormalizeChannel`（满量程 660 计数、中心死区 5%）
- `User/module/feeder.h/.c`：`Feeder_Control` 改为接收 `const Feeder_Command_t *`，命令缺失按离线档处理；目标为 `speed_ratio × cycle_config.max_speed_rpm`，300 rpm 上限仍只在 config.c 定义一处；故障锁存只能由待机档解除
- `User/task/motor_trigger.h/.c`：新增 `feeder_command` 观测入口，每周期由 `Cmd_ParseFeeder` 覆盖；电流发布失败时本周期不提交命令，按离线档撤驱，避免在任务内运行第二次控制
- 顺带修复：`User/module/shoot.c` 删除与外设配置迁移重复的 `shoot_speed_feedforward_gain_a_per_rpm` 定义，该重复导致链接期 multiple definition，属上一轮 config.c 迁移遗留

## 验证证据

| 命令 | 结果 | 验证边界 |
| --- | --- | --- |
| py .auto-embedded/scripts/check.py | ARCH、HW、SPEC 全 PASS | 分层、硬件锁和 spec 完整性 |
| py .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug | 构建成功，产物 build/Debug/Infantry_Robot.elf | ARM GCC 14.3.1、Ninja |
| arm-none-eabi-size build/Debug/Infantry_Robot.elf | text 124800 B、data 184 B、bss 46992 B，FLASH 124984 B / 1 MB（11.92%），RAM 47176 B / 128 KB（35.99%） | 映像占用，非栈高水位 |
| ninja -t commands 复用编译数据库，追加 -Wextra -Werror -fsyntax-only | cmd.c、feeder.c、motor_trigger.c、shoot.c 全部 STRICT PASS | ARM 严格语法检查 |
| gcc 主机编译真实 cmd.c、feeder.c、config.c、pid.c、filter.c、user_math.c（仅替换 bsp/can.h 的 HAL 依赖） | 42 项断言，0 失败 | 临时主机测试，结果记录后删除 |
| git diff --check | PASS | 差异无空白错误 |
| 机械扫描 | 7 个改动文件 0 处超 120 列、0 处行尾空白、0 处 `} else`、0 处同行花括号 | 文件级扫描 |

- 主机断言覆盖：拨杆下位、上位、中位和非法档位；摇杆 +660、-660、+330、+20 与 0 的比例换算；中心死区为零；缓存无效、离线、源时间超过 100 ms 时输出离线档
- 主机断言覆盖：比例 1 对应 300 rpm、比例 -0.5 对应 -150 rpm、比例 0 对应 0 rpm；把 `feeder.config.max_speed_rpm` 改为 600 后比例 1 对应 600 rpm，说明上限仍由实例可调配置决定
- 主机断言覆盖：运行、待机、反馈超时离线、故障锁存只能由待机档解除、空命令不解除故障、反馈恢复自动重回运行、非法周期进入故障、输出闸门非运行状态为零
- 未进行 MCU 烧录、未写目标 RAM、未上板验证摇杆方向、死区和 300 rpm 实际转速

## 校验自查与人工复核

| 检查项 | 处理 |
| --- | --- |
| 新增判空、`isfinite`、范围检查、保护性 return | 无新增 |
| 删除的检查 | 删除 feeder 入口 `isfinite(requested_speed_rpm)`，速度比例改由命令层归一化保证有限；控制出口 `isfinite(current_a)` 仍拦截 NaN |
| 删除的检查 | 删除 feeder 内遥控在线、时间回退和 100 ms 超时判断，原样迁移到命令层 `Cmd_ParseFeeder`，不重复 |
| 删除的检查 | 删除 feeder 内目标 `AbsClip`，比例已由命令层限制在 [-1, 1]，乘上限后自然在范围内；上传限幅不再重复 |
| 信任边界 | 遥控缓存有效性、在线、源时间回退和 100 ms 超时、拨杆档位合法性集中在 `Cmd_ParseFeeder` 一处 |
| 分层 | task -> module 不变；feeder 不再依赖 device/dr16.h，命令由 module/cmd 生成，任务只编排 |

- 人工复核：新增和修改注释均为 UTF-8 简体中文且末尾无句号；Doxygen 参数与返回值已同步；结构体字段说明集中；未改 config.c 数值、BSP、设备层、CubeMX 或摩擦轮控制逻辑
- 临时主机测试源码、替身头和可执行文件位于 build/host_feeder_joy，记录证据后已删除

## 调试入口

- 速度上限：`feeder.config.max_speed_rpm`，当前 300 rpm，也是比例 1 对应的转速；修改后下一周期生效
- 速度环参数：`feeder.config.speed_pid` 的 k、p、i、d 等，前馈入口仍为 `feeder.config.speed_feedforward_gain_a_per_rpm`，初值零
- 观测：`feeder.speed_pid.target_speed_rpm`、`feeder.speed_pid.measured_speed_rpm`、`feeder.output_current_a`、`feeder.state`
- 命令观测：`feeder_command.mode` 与 `feeder_command.speed_ratio`，每周期被 `Cmd_ParseFeeder` 覆盖，调试器写入不作为控制入口
- 原 `feeder.speed_pid.requested_speed_rpm` 已删除，EmberProbe 旧绑定需同步移除并改用上述字段
- 本轮生成新 ELF 但尚未烧录；使用前需确认运行固件与 ELF 一致，摇杆正反、死区和目标转速仍待上板验证
