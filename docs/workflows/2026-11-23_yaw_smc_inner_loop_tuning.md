# yaw 串级从内到外的整定清单（2026-11-23）

## 1. 为什么改顺序

- [2026-10-29 yaw SMC 移植计划](2026-10-29_yaw_smc_migration_plan.md) 第 10 节的调参顺序默认速度内环已经可用，直接从小角度阶跃调滑模参数
- 仓库没有 yaw 速度环的阶跃或扫频实测记录；唯一实机采样 [yaw governor 汇总](../../data/emberprobe-yaw-governor-20261027-summary.json) 是定点保持，只能说明静态可用
- `GIMBAL_MODE_SPEED` 在模块内已实现，但命令层只产生 RELAX 和 ANGLE，调试器直接写 `gimbal_command.mode` 又会被任务每个周期覆盖，速度环此前无法单独进入
- 滑模外环的近零等效刚度为 `A = C·(1 + K/surface_eps)`，默认 C=5、K=10、surface_eps=0.5 时 A 约 105，它只在速度内环穿越频率明显高于外环时才成立
- 因此本轮先补 yaw 速度内环调试入口，整定顺序改为：反馈与标度核验 -> 速度内环 -> 滑模外环 -> 参考轨迹与限幅 -> 扰动与恢复

内环再往里是 GM6020 驱动内部的电流环，不可调，只做电流标度核验，不把它当作可整定对象

## 2. 新增调试入口

源码位置：[motor_gimbal.c](../../User/task/motor_gimbal.c) 与 [motor_gimbal.h](../../User/task/motor_gimbal.h)

| 变量 | 单位 | 默认 | 作用 |
| --- | --- | --- | --- |
| `gimbal_debug_yaw_speed_enabled` | - | false | 置 true 后强制 SPEED 模式，跳过滑模外环 |
| `gimbal_debug_yaw_speed_target_rad_s` | rad/s | 0 | 速度内环目标，符号与 `gimbal.yaw.speed_rad_s` 相同 |

- 速度调试优先于角度调试；两者同时为 true 时按 SPEED 运行
- SPEED 模式由两轴共用，pitch 同步进入速度环且目标为零速，只提供主动阻尼、不保持姿态
- 速度目标没有参考轨迹平滑，写入后直接进入速度 PID，所以首测必须从小值开始
- 输出发布失败时两个调试使能同时自动关闭，避免故障解除后自动重新起控
- 关闭使能后由遥控命令接管；若已锁存故障，须先让命令回到 RELAX 再重新使能
- 本轮 ELF 中四个调试变量地址为 0x200067C4、0x200067C8、0x200067CC、0x200067D0，地址随固件变化，不跨版本复用

调试器写入顺序：先写目标值，再写使能；关闭时先写使能 false

## 3. 第 0 步：前置核验（不改任何参数）

| 检查项 | 观测 | 通过条件 |
| --- | --- | --- |
| 待机零输出 | `gimbal.mode`、`gimbal.yaw.state`、`gimbal.yaw.output_current_a` | mode=0、state=0、电流为 0 |
| 惯性反馈 | `gimbal.imu_state`、`gimbal.imu.sample_time_us` | imu_state=2 且采样时间持续前进 |
| 电机反馈 | `gimbal.yaw.feedback_time_us`、`gimbal.yaw.encoder_deg` | 时间持续前进，编码器角连续变化 |
| 方向 | 角度调试目标设为当前 `gimbal.yaw.angle_deg`，再 +5 度 | angle_deg 朝目标增大，电流与转速同号 |
| 电流标度 | 驱动 ±3 A 对应 ±16384、轴限幅 `max_current_a=0.5` | 与 `output_current_a` 一致，反馈电流标度已知 |
| 基线记录 | 固件提交、参数、限流、负载、环境 | 后续每次对比同一条件 |

方向不一致时先停机核对符号，不得用增大增益掩盖

## 4. 第 1 步：速度内环

### 4.1 阶跃测试

写 `gimbal_debug_yaw_speed_target_rad_s`，每档保持约 2 秒后回零，间隔至少 2 秒

| 目标 rad/s | 约合 deg/s | 观察重点 |
| --- | --- | --- |
| 0.1 | 5.7 | 死区、静摩擦、稳态误差 |
| 0.2 | 11.5 | 上升时间、超调 |
| 0.4 | 22.9 | 线性区响应、电流占比 |
| 0.8 | 45.8 | 是否接近限流 |
| 1.5 | 85.9 | 限流与饱和行为 |

每档记录：

| 变量 | 用途 |
| --- | --- |
| `gimbal.yaw.target_speed_rad_s` | 实际进入速度环的目标 |
| `gimbal.yaw.speed_rad_s` | INS 欧拉 yaw 角速度，内环反馈 |
| `gimbal.yaw.output_current_a` | 出力与限流占比 |
| `gimbal.yaw.speed_pid.i`、`gimbal.yaw.speed_pid.last.out` | 积分与内环输出 |
| `gimbal.yaw.state`、`gimbal.imu_state` | 是否仍在 RUNNING 与就绪 |

判据先记录实测，不预设硬指标，初步可接受范围为：超调不超过 20%，无持续振荡，1 秒内稳态误差收敛到 0.02 rad/s 以下；稳态电流明显小于 0.5 A，阶跃瞬间允许贴限流但要记录退出时间

同时注意静止噪声：目标为 0 时 `output_current_a` 的峰峰值和 `speed_pid.i` 抖动，是后面外环抖振判断的底噪

### 4.2 扫频测试

- 幅值 0.2 rad/s，频率 0.2、0.5、1、2、3、5 Hz，每个频点约 5 秒
- 由主机脚本或调试器按不低于 50 Hz 更新目标值生成正弦阶梯近似，控制任务是 500 Hz
- 量取幅值比与相位滞后，找到 -3 dB 点，估算内环穿越频率 `w_c`（rad/s）
- 若扫频条件不具备，可先用不同幅值的阶跃对比上升时间，按一阶近似估算带宽，并在记录中注明方法

### 4.3 用内环结果反推外环起点

外环近零等效刚度上限取内环穿越频率的五分之一：

```text
A = C * (1 + K / surface_eps) <= w_c / 5
```

保持阻尼比 `K / surface_eps` 在 2 到 3 附近，例如 K=5、surface_eps=2，则 `A = 3.5·C`：

| 实测 w_c（rad/s） | 外环刚度上限 A | 起始 C（K=5、eps=2） |
| --- | --- | --- |
| 20 | 4 | 约 1.1 |
| 40 | 8 | 约 2.3 |
| 80 | 16 | 约 4.6 |

默认 C=5、K=10、eps=0.5 对应 A 约 105，反推需要内环穿越频率约 500 rad/s，实际不可能达到，说明默认组合只适合理想硬内环，不能作为首次上板参数

## 5. 第 2 步：滑模外环

使用角度调试入口：`gimbal_debug_yaw_target_enabled` 与 `gimbal_debug_yaw_target_deg`，5 度和 10 度小阶跃起步

一次只改一类参数，顺序如下：

1. `C`：按 4.3 的起点设好，观察接近与制动
2. `K`：提高抗扰与趋近速度，先看是否引入抖振
3. `surface_eps_deg_s`：静止抖振先加宽，精度不足再收窄
4. `error_eps`：小角度爬行或到位黏滞时再对比
5. `q`、`p`：最后调，保持 0 < q/p <= 1，改 q 前先确认 p 不变导致 q/p 不越界

判据用实际角对内部参考角：

- 比较 `gimbal.yaw.angle_deg` 与 `gimbal.yaw.target_angle_deg`，判断控制器跟随
- 比较内部参考与外部目标，判断是否被参考轨迹限速
- 记录超调、振荡次数、到位时间、静差、电流峰值与持续限流占比

现象与优先动作：

| 现象 | 优先检查 | 处理 |
| --- | --- | --- |
| 静止抖动或电流毛刺 | 内环噪声底、边界层 | 加宽 `surface_eps_deg_s`，再考虑降 K |
| 到位软、拖尾长 | 外环刚度 | 加 C，但不超过 4.3 的上限 |
| 大角度段慢 | 速度上限 | 先抬 `speed_max_deg_s`，不要猛加 C |
| 超调大、来回摆 | 外环刚度或内环超调 | 先降 C，检查内环阶跃是否已超调 |
| 小角度黏滞、爬行 | 近零过渡 | 对比不同 `error_eps` |
| 长期贴限流 | 目标或电流预算 | 先查 0.5 A 限流与目标是否过激 |

## 6. 第 3 步：参考轨迹与限幅（需要重烧）

| 项目 | 位置 | 当前值 | 说明 |
| --- | --- | --- | --- |
| `GIMBAL_SMC_MAX_ACCEL_DEG_S2` | [gimbal.c](../../User/module/gimbal.c) | 180 | 内部参考加速度上限 |
| `GIMBAL_SMC_REFERENCE_GAIN` | 同上 | 5 | 参考追踪增益，平滑目标跳变 |
| `speed_max_deg_s` | `gimbal_yaw_smc_config` | 60 | 同时限制参考速度和滑模输出 |
| 正弦测试幅值与频率 | [motor_gimbal.c](../../User/task/motor_gimbal.c) | 30 度、6 rad/s | 峰值 180 deg/s，远超 60 deg/s 限速 |
| `max_current_a`、反馈超时、IMU 超时 | [config.c](../../User/module/config.c) | 0.5 A、90 ms、10 ms | 保护边界，稳定前不放大 |

正弦测试当前参数会被限速长期削顶，只能看到限幅行为；如需用它评估动态，先改成峰值不超过 `speed_max_deg_s` 的组合，例如 5 度、2 rad/s 对应峰值 10 deg/s，并重新烧录

yaw 机械限位当前关闭，做大角度测试前先确认线束和行程

## 7. 在线改参的注意点

- 非法值会锁存故障：C<=0、surface_eps<=0、error_eps<=0、speed_max<=0、q/p>1、max_current<=0、PID range 不为 0；恢复时先关闭调试使能，再让命令回到 RELAX
- 逐字段写入期间可能有一到两个周期使用新旧混合值；改完核对 `gimbal.yaw.applied_speed_pid` 等副本
- 改速度 PID 参数会清空积分，对比曲线要保证起点一致
- 在线值掉电丢失，整定完成后写回 [config.c](../../User/module/config.c) 与 [gimbal.c](../../User/module/gimbal.c) 的默认值并重新烧录
- yaw 的 `angle_pid` 在 ANGLE 和 SPEED 下都不参与计算，只参与配置合法性校验，不要在其中写非法值
- 在线改 `gimbal_yaw_smc_config` 只影响外环参数，不清 SMC 历史；`target` 字段由命令与参考轨迹驱动，写入无效

## 8. 本轮软件验证

| 检查 | 命令或方法 | 结果 |
| --- | --- | --- |
| Debug 固件 | `py .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug -j 2` | 通过，构建耗时 4.2 秒，ELF 1923332 字节 |
| ARM 严格语法 | 由 `compile_commands.json` 取 `motor_gimbal.c` 命令并加 `-Wall -Wextra -Werror -fsyntax-only` | 退出码 0 |
| 调试符号 | `arm-none-eabi-nm build/Debug/Infantry_Robot.elf` | 四个调试变量均在 `.bss` |
| 格式扫描 | 同行花括号、else 同行、超过 120 列、行尾空白、注释句号 | 无违规 |
| 规范门禁 | `py .auto-embedded/scripts/check.py` | HW、SPEC 通过；ARCH 因本机执行策略无法加载子脚本 |
| 架构门禁 | `powershell -NoProfile -ExecutionPolicy Bypass -File .auto-embedded/scripts/arch-check.ps1` | 0 项违规 |

本轮改动仅在任务层增加调试命令路由，未修改模块、组件、驱动和 CubeMX 配置

验证边界：未烧录、未连接探针、未运行真实电机；阶跃与扫频行为、限流占比、内环带宽、外环参数效果均需上板测量后才能写入结论

## 9. 下一步

1. 烧录本轮 ELF，完成第 0 步核验
2. 按第 4 节做速度阶跃与扫频，记录内环带宽和噪声底
3. 按 4.3 反推外环起点，按第 5 节逐项回收
4. 稳定后放开 `speed_max_deg_s`，再做正弦、扰动与失联恢复
5. 可选：增加在线可调的速度正弦入口，便于重复扫频
