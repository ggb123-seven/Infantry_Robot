# 云台 yaw 角度环摩擦前馈门控与 500 Hz 片上辨识（2026-11-19）

## 1. 背景

用户反馈 yaw 轴「参数有点软」，要求用 MATLAB 辨识角度外环并压低超调。前一轮（2026-11-17）的角度环结论建立在「无延时级联模型」上，本次先用片上 500 Hz 记录把实测闭环拿准，再定位超调来源。

## 2. 探针链路的局限与片上记录

探针（EmberProbe `variables.sample`）在 6 变量时只能到约 55 Hz，且间隔抖动大：样本间隔中位数 16 ms，p95 = 29 ms，最大 62 ms，**16% 的间隔超过 25 ms**。在这个抖动下，60 ms 量级的上升沿无法可信拟合，闭环辨识最好只到 87.8%（原始）／94.2%（8 Hz 平滑）。

因此新增 `User/task/gimbal_debug_log.{c,h}`：在 `Task_motor_gimbal` 的 Control 之后按 500 Hz 控制周期记录 `{角度指令, IMU 世界角, 世界系角速度, 输出电流}`，4096 样本静态缓冲（8.19 s），写满自动停。导出用 `read.js --snapshot-csv` 分 16 片（每片 256 样本）拼接。

## 3. 角度环实测（500 Hz，无平滑）

8 次 ±4/±3° 阶跃（原参数 Kp=20、Kd=1.0、前馈全开）：

| 指标 | 数值 |
| --- | --- |
| 超调 | 中位 25.9%，范围 18.9–27.3% |
| 上升 10–90% | 中位 66 ms |
| 5% 收敛 | 256–554 ms |
| 稳态误差 | <0.2° |

同时用 12 次阶跃做系综平均（归一化后按指令沿对齐）：超调 20.4%，峰值约 100 ms。IMU 世界角与编码器机械角差值恒定（std 0.033°），排除底座反冲导致的角度假象。

闭环辨识（同批 500 Hz 数据）：3 阶模型拟合 **94.57%**，直流增益 0.984，极点 `-7.842±31.42i`、`-22.8 rad/s`，模型阶跃超调 18.4%、上升 62 ms。残差分解：平台段 RMS 0.113°、过渡段 0.199°——平台残差来自 4.5 Hz 摩擦极限环，过渡残差来自摩擦黏滑，属机构非线性，不随模型阶次改善。

另两项辅助测量：
- IMU 解算角相对「世界系角速度积分」的滞后仅 **5.0 ms**（相关系数 0.9994），角度反馈链路不是滞后源
- 速度内环闭环极点由旧值 33.684 rad/s 提高到约 **40 rad/s（6.4 Hz）**（旧值对应 20 Hz 低通 + 13.5 Hz 陷波，现固件为 40 Hz 低通无陷波）

## 4. 超调根因

`Gimbal_CalcYawFriction` 的输入是**限幅后的闭环目标角速度**（角度环输出），不是实测速度。阶跃时目标速度冲到 1.4 rad/s，前馈立刻给出约 0.26 A、与运动同向的推力并持续整个接近段；等目标速度过零反向时实际速度仍有约 0.9 rad/s。这等价于给大机动加了一记助推，速度环必须额外克服它，表现为 20–26% 过冲。稳态时目标≈实测，前馈才是对的，所以静止与小信号看不出问题。

A/B 验证（关掉前馈，同一组阶跃）：

| 配置 | 超调中位 | 上升 |
| --- | --- | --- |
| 前馈开 Kp20/Kd1.0 | 25.9% | 66 ms |
| 前馈关 Kp20/Kd1.0 | 12.6% | 100 ms |
| 前馈开 Kp16/Kd1.6 | 19.7% | 76 ms |
| 前馈关 Kp16/Kd1.6 | 7.6% | 114 ms |

用户记忆中「之前约 8% 超调」与「前馈关 + 略高阻尼」的 7.6% 吻合，印证超调是 11-16/17 加入摩擦前馈之后引入的。

## 5. 实现：前馈按目标角速度门控

`gimbal.c` 新增 `Gimbal_FrictionFade`：目标角速度 ≤0.2 rad/s 时前馈全量，≥0.6 rad/s 时完全淡出，中间线性过渡；前馈方向仍取目标速度方向，避开编码器差分速度在零速跳变的问题。淡出只作用于前馈幅值，静止补偿保留。

同时试过「按目标与实测速度差淡出」的写法，实测超调只到 15.7%，不如按目标速度门控（10.3%），采用后者。

## 6. 门控后实测与增益扫描（500 Hz 片上，每组 8 次阶跃）

| Kp / Kd（固件 p / d） | 超调中位 | 范围 | 上升 | 5% 收敛 | 静差 |
| --- | --- | --- | --- | --- | --- |
| 20 / 1.0（1.0 / 0.05） | 10.3% | 6.3–16.0 | 113 ms | 407 ms | 0.19° |
| **24 / 1.2（1.2 / 0.06）** | **11.1%** | 1.8–18.1 | **99 ms** | **336 ms** | 0.21° |
| 28 / 1.2（1.4 / 0.06） | 17.4% | 11.9–18.9 | 79 ms | 622 ms | 0.24° |
| 28 / 1.6（1.4 / 0.08） | 12.7% | 2.9–20.4 | 80 ms | 758 ms | 0.14° |
| 32 / 1.6（1.6 / 0.08） | 18.6% | 11.5–24.9 | 68 ms | 798 ms | 0.19° |
| 32 / 2.0（1.6 / 0.10） | 14.7% | 9.0–26.4 | 67 ms | 766 ms | 0.19° |

结论：门控把原参数下的超调从 25.9% 压到 10.3%、静差保持 0.19°；代价是上升 66→113 ms（被拿掉的助推原本也在加快起步）。综合最优测试点为 **Kp24/Kd1.2**：上升 99 ms、收敛 336 ms、超调 11.1%、静差 0.21°，相对原参数既更快也更稳。

## 7. RAM 占用：为什么一度到 88.79%，如何降回 38.79%

- 改动前的正式固件基线：RAM 50840 B / 128 KB（38.79%），其中 `.bss` 约 34.2 kB、堆栈保留 16 kB、`.data` 208 B
- 片上记录缓冲为 `4096 样本 × 16 B = 65536 B`，加上后总量 116384 B（88.79%）。**这 64 kB 全部是辨识专用缓冲**，与控制链无关
- 缓冲设计上遵守「写满即停、不覆盖」，所以无法用环形覆盖换更小容量而保住早期数据；但辨识本身不需要长期驻留
- 处理：`gimbal_debug_log.h` 增加编译开关 `MOTOR_GIMBAL_ENABLE_DEBUG_LOG`，默认 0；`gimbal_debug_log.c` 整个文件与 `Task_motor_gimbal` 中的调用都受该开关控制
- 关闭后重新构建：RAM **50840 B（38.79%）**，FLASH 133864 B（12.77%），与原基线一致，功能不受影响
- 需要做系统辨识时把开关置 1 重新构建即可（RAM 回到 88.79%）；若日后要缩小，可将 `GIMBAL_DEBUG_LOG_CAPACITY` 从 4096 降到 2048（32 kB，4.1 s @ 500 Hz），仍够 6–7 次阶跃

## 8. 验证证据

| 项目 | 结果 |
| --- | --- |
| 辨识固件（记录开） | ELF SHA256 `2fc07f45…6a8e5`，RAM 116384 B，已烧录并采数 |
| 正式固件（记录关） | ELF SHA256 `d63b72734505e5a9044c38a9c8c481957d17087f64e1eed982544f4aef6f3012`，RAM 50840 B / FLASH 133864 B |
| 烧录 | Programming Finished、Verified OK、Resetting Target，`firmwareConsistency: match` |
| 静态栈 | `Gimbal_DebugLog_Push` 48 B；任务链 `Task_motor_gimbal` 368 + `Gimbal_Control` 376 + `Gimbal_ControlAxis` 48 + `Gimbal_RunAxis` 104 ≈ 0.9 kB，对 3072 B 任务栈 |
| 排版 | 5 个改动文件 0 处超 120 列、0 处行尾空白 |

**未验证**：运行时栈高水位；门控阈值（0.2/0.6 rad/s）未做参数扫描；增益扫描只在台架 ±4/±3° 小阶跃上完成，未覆盖打靶、连续跟踪和大信号饱和反向工况。

## 9. 参数现状

- `config.c` 未改动，`gimbal_config.yaw.angle_pid` 仍为 `p=1.0`、`d=0.05`
- 建议值 **`p=1.2`、`d=0.06`**（Kp=24、Kd=1.2）仅在 RAM 中试过，断电即失效；是否固化由用户决定
- 门控阈值固定为编译期常量：`GIMBAL_FRICTION_FADE_FULL_RAD_S = 0.20`、`GIMBAL_FRICTION_FADE_ZERO_RAD_S = 0.60`

## 10. 复现步骤

```powershell
# 1. 打开记录（把 gimbal_debug_log.h 的 MOTOR_GIMBAL_ENABLE_DEBUG_LOG 置 1）后构建并烧录
cmake --build --preset Debug
node .agents\skills\mcu-flash\scripts\program.js --workspace F:\RM\Infantry_Robot --execute

# 2. 进调试角度模式并保持姿态（pitch 关闭避免被机械限位带回）
node .agents\skills\mcu-variables\scripts\write.js --workspace F:\RM\Infantry_Robot --set "gimbal_debug_enabled=1,gimbal_command.yaw.source=1,gimbal_command.yaw.rate_rad_s=0,gimbal_command.yaw.angle_deg=<当前角>,gimbal_command.pitch.source=1,gimbal_command.pitch.rate_rad_s=0,gimbal_command.pitch.angle_deg=0,gimbal.config.pitch.enabled=0"
node .agents\skills\mcu-variables\scripts\write.js --workspace F:\RM\Infantry_Robot --set "gimbal_command.mode=2"

# 3. 清空并启动记录，随后按 0.8 s 驻留写 8 个角度台阶（±4/±3°）
node .agents\skills\mcu-variables\scripts\write.js --workspace F:\RM\Infantry_Robot --set "gimbal_debug_log_count=0,gimbal_debug_log_enabled=1"

# 4. 分片导出（每片 256 样本，区间右开）
node .agents\skills\mcu-variables\scripts\read.js --workspace F:\RM\Infantry_Robot --snapshot-csv --variables "gimbal_debug_log[0:256]" --output data/s00.csv

# 5. 恢复：debug=0、pitch.enabled=1、p/d 回默认
```

## 11. 文件与提交

- 源码：`User/module/gimbal.c`（门控）、`User/task/gimbal_debug_log.{c,h}`（新增，受编译开关控制）、`User/task/motor_gimbal.c`（记录钩子）、`CMakeLists.txt`
- 数据：`data/yaw_angle_onboard_20261119.csv`、`data/ab_gate_*.csv`、`data/ab_sweep_*.csv`、`data/ab_fade_*.csv`
- 分析脚本：`.auto-embedded/tmp/lead_*.m`（一次性分析，未纳入仓库）
- 本地提交：门控与记录实现见 `ede2b1d`；编译开关与本文档见后续提交
