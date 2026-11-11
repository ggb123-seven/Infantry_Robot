# 底盘跟随云台 PID 软件验证

日期：2026-11-11

状态：用户选择方案 A 并授权实施，软件验证通过；未烧录、未写目标 RAM、未实车验收

## 实现与使用

- `Chassis_Config_t.follow_pid` 统一保存跟随配置，EmberProbe 写入 `chassis.config.follow_pid`
- `Chassis_t` 公开 `follow_relative_angle_deg`、`follow_relative_speed_rad_s`、`follow_wz`；PID 历史和持久配置快照保留在 `chassis.c` 私有上下文，本工程仅支持单一底盘实例
- 初值为 k=1、p=0.5、i=d=0、i_limit=0、out_limit=0.3、d_cutoff_freq=-1、range=2π，属于首轮整定值
- PID 输入弧度，输出归一化转向请求；输出经过既有混控、轮速斜坡和四轮速度环，不直接代表车体 rad/s 或电流
- 跟随采用 sp=0、fb=-相对角、fb_dot=-相对角速度，保留原转向方向、零点与 reversed 约定；直接转速微分不经过当前 PID 组件的微分低通
- `Chassis_UpdateGimbalFeedback` 只负责反馈边界与机械观测，Control 负责 FOLLOW/SPIN 平移转换，命令本身不回写；SPIN 保留命令给定转向
- 改参在 Control 入口快照、集中校验并同步，PID 参数指向持久 `cycle_config.follow_pid`；退出 FOLLOW 时清除历史，持续待机不重复复位跟随 PID
- FOLLOW/SPIN 的 yaw 反馈无效仍进入既有 RELAX 路径，独立零速刹车不依赖 yaw 反馈，最终电流仍由 DumpOutput 按运行状态放行

生产修改范围为 `User/module/chassis.c/.h`、`User/module/config.c`、`User/task/motor_chassis.c/.h`。config.c 仅追加跟随说明及初始化；相对本轮开始时工作树，移除这两处新增文本后逐字一致。已有拨弹、yaw、pitch 等未提交参数保持原状

## 验证结果

| 检查 | 命令或方法 | 结果 |
| --- | --- | --- |
| 模块真实代码回归 | 临时主机驱动链接真实 chassis/config/pid/filter/mixer/user_math | 17,719 条断言通过 |
| 任务完整周期回归 | 真实 motor_chassis/cmd/fault_detect，替身仅提供 RTOS、时间与总线类型 | 17 场景、1,020 完整周期、6,768 条断言通过 |
| ARM 严格检查 | 从 Debug compile_commands.json 提取三个修改 C 文件的实际命令，增加 `-Wextra -Werror -fsyntax-only` | 全部通过 |
| ARM 静态栈 | 实际 ARM 编译命令加 `-fstack-usage` | 已核对下表 |
| Debug 配置和构建 | CMake 技能脚本完成配置；最终 `cmake --build --preset Debug -j 2 --verbose` | 5/5 成功，无编译警告 |
| 机械门禁 | `powershell -NoProfile -ExecutionPolicy Bypass -Command "python .auto-embedded/scripts/check.py"` | ARCH/HW/SPEC 全部 PASS |
| 源码格式与差异 | 逐文件重读 AGENTS.md、agent.md，扫描 Allman、120 列、空白并人工核对注释；`git diff --check` | 通过 |
| 冻结配置保留 | 去除跟随新增文本后与任务开始时 config.c 基线比较 | 相同 |

主机模块覆盖 720 组角度及方向组合的 FOLLOW/SPIN 几何、跨零与 ±180°、正负方向、限幅、命令不回写、输出闸门序号、参数持久地址、P/I/D 在线变化、直接转速微分、退出清积分、周期异常、反馈缺失、90 ms 边界、低位时间回绕、未来时间、丢角、32 类非法参数及单轮离线

任务场景包括跟随、自旋、上位刹车、首帧缺失恢复、反馈超时恢复、yaw 邮箱错误、命令邮箱 Reset/Put 失败、遥控超时、非法档、未来遥控帧、四轮离线、架空调试接管、tick 回绕、非法在线参数恢复、调试 NaN 与单轮恢复

结果原文：

```text
PASS: 17719 assertions; geometry, tuning, modes, feedback, safety
PASS: 6768 task assertions; 17 scenarios, 1020 complete task cycles
```

最终构建产物：`build/Debug/Infantry_Robot.elf`

- SHA-256：`62905BA27E8E307B1E405E0597488D937B5AD70D8C0FD49C42CECA807933BF16`
- FLASH：131,612 B / 1 MiB；RAM：50,616 B / 128 KiB；CCMRAM：0 B
- 产物包含当前工作树既有其他模块修改，SHA 不是本次提交单独检出的产物身份
- 最终技能脚本的增量构建曾无进展，终止该次构建进程后，使用上述直接 CMake 命令成功重编并链接；没有把旧 ELF 作为最终证据
- 初次门禁受 PowerShell 签名策略限制，只对当前进程使用 Bypass，未改持久执行策略

## 静态栈与实测边界

`attr_motor_chassis.stack_size` 当前为 1024 B，本轮未改任务栈配置

| ARM Debug 函数 | `.su` 单帧字节 |
| --- | ---: |
| Task_motor_chassis | 144 |
| Chassis_Control | 208 |
| Chassis_Init | 80 |
| Chassis_UpdateGimbalFeedback | 48 |
| Chassis_ApplyPidConfig | 64 |
| Chassis_ControlWheel | 40 |
| Chassis_RunWheel | 40 |
| Chassis_PrimeWheel | 24 |
| Chassis_IsConfigValid | 32 |
| PID_Calc | 72 |

主轮速路径所列函数帧相加为 144+208+40+40+72=504 B，尚需考虑滤波、libm、RTOS 与异常上下文，不能当作完整最坏栈上界或运行时剩余量。没有新固件高水位，栈安全实测未完成；历史云台栈溢出记录不能替代本底盘固件的运行时证据

## 检查增删自查

| 本次新增检查 | 信任边界与实际触发场景 | 上游是否已检查 |
| --- | --- | --- |
| follow.k 有限且 >0 | Init/Control 配置边界，在线错误写入零、负值或非数 | 无上游在线配置校验 |
| follow.p 有限且 ≥0 | 同上，比例错误写入 | 同上 |
| follow.i 有限且 ≥0 | 同上，积分错误写入 | 同上 |
| follow.d 有限且 ≥0 | 同上，微分错误写入 | 同上 |
| follow.i_limit 有限且 ≥0 | 同上，积分限幅错误写入 | 同上 |
| follow.out_limit 有限且位于 (1e-6,1] | 同上，避免 PID 组件零限幅直通语义以及归一化越界 | 同上 |
| follow.d_cutoff_freq 有限且小于 Nyquist | 同上，保证滤波器配置合法，SET_D 不因此获得输入滤波 | 同上 |
| follow.range 等于 M_2PI | 同上，拒绝误改固定角度单位与环绕周期；相等比较同时拒绝非数 | 同上 |
| 跟随 PID_Init 返回失败即返回 -1 | 初始化边界，组件初始化结果必须处理，initialized 保持假 | 配置预检不能替代组件返回值处理 |

没有新增实例判空或对 PID 返回值的重复 isfinite。反馈 NULL、帧龄、angle_valid、丢角和单圈角范围检查由旧 Resolve 接口迁入新反馈入口，参数源零点仍由调用方保证；实际 period_s、原始轮速尺度与混控返回检查保留，未在下游重验。原始轮速尺度检查必须保留，因为 fminf 会掩盖 NaN，混控看不到被裁剪前的原值

删除或替代项：旧 Resolve 中按命令模式提前 return 的分支随接口职责迁移移除；旧跟随宏及独立 AbsClip 由 PID out_limit 替代；复核中把跟随历史复位集中到 Control，避免改参采纳阶段与模式退出重复执行。未增加下游重复检查或安全联锁

## 分层、硬件与交付复核

- 控制策略在 module，任务只搬运反馈、调度与发布；不改 BSP、设备、通用 PID、CAN 任务、HAL 或 CubeMX
- 复用既有 yaw 邮箱与反馈，无新引脚、总线 ID、DMA、中断或 RTOS 对象，无新增 ISR 共享状态、阻塞临界区或内存分配
- 检查新增 PID 初始化返回值和既有邮箱发布失败撤销路径；非法配置、模式或输入仍由状态转换及 DumpOutput 保证零指令
- 临时主机程序在验证完成后清理，保留本记录作为软件验证证据；原有正式测试资产不删除
- 本地提交只收录本任务差异；config.c 和规范文件按差异块暂存，已有调参和其他工作保留未提交，不推送远端

下一步先架空核对小角差方向和限幅、运行时栈高水位，再落地以相同负载采集 ±10°、±30° 与连续转动的角差、wz、四轮请求/目标/实测速度和电流，对比旧 0.05 比例与新 P；根据响应再决定 I/D。不能用主机回归或架空均速证明跟随手感改善
