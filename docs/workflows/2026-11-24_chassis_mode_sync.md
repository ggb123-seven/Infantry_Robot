# 底盘模式同步（FOLLOW_GIMBAL / SPIN，无功率限制）（2026-11-24）

## 背景与授权

- 用户要求把父工作树 `F:\RM\Infantry_Robot`（分支 `main`）的底盘模式同步到本工作树
- 明确要求：**功率限制先不加**，因此不引入 `module/power`，也不做限流再分配
- 本工作树云台 yaw 电机反馈已可用（CAN1 ID5 GM6020，0x209），可支撑跟随/自旋所需的云台朝向

## 移植内容

| 文件 | 改动 |
| --- | --- |
| `User/module/chassis.h` | 模式枚举增加 `CHASSIS_MODE_FOLLOW_GIMBAL`、`CHASSIS_MODE_SPIN`；配置增加 `follow_pid`；增加 `ramped_move`、跟随相对角/角速度、`follow_wz` 状态；声明 `Chassis_UpdateGimbalFeedback` |
| `User/module/chassis.c` | 云台反馈校验入口、指令层归一化斜坡、跟随 PID、模式决策、云台坐标平移旋转；输出放行三种运行模式 |
| `User/module/cmd.c` | 左拨杆：上=INDEPENDENT、中=FOLLOW_GIMBAL、下=SPIN；自旋量改为父树 -0.50 |
| `User/module/config.c` | chassis_config 增加 follow_pid（k=1、p=0.6、i=0、d=0、i_limit=1.0、out_limit=1.0、d_cutoff=-1、range=2π）；云台段未改 |
| `User/task/user_task.h` | task_runtime 增加 `chassis_yaw_feedback` 邮箱与初始化错误码 |
| `User/task/init.c` | 创建并回收该邮箱 |
| `User/task/can_task.c` | yaw 反馈帧同时投递云台与底盘邮箱，同一帧只解码一次 |
| `User/task/motor_chassis.c` | 每周期读取 yaw 反馈，用 `gimbal_config.yaw.zero_encoder_deg/reversed` 调用反馈入口 |

功率限制的移除方式：本工作树本无 power 模块，移植时完全不引入 `Power_t`、`Power_Config_t`、功率限幅与回馈；`Chassis_Init` 保持 `(chassis, config, frequency_hz)` 三参数。

## 关键设计决策

1. INDEPENDENT 保留本工作树原有轮速斜坡和换向放行；FOLLOW/SPIN 使用父树指令层斜坡，且不叠加轮速斜坡，避免跟随环相位滞后与自旋换向被置零
2. 跟随符号沿用父树 `2ad07ae` 的实测修复约定（正轮速使相对角增加，正角差产生负转向请求）
3. 上位语义保留本工作树平移（父树该档为零速刹车），作为待确认项
4. 跟随失效安全：yaw 反馈缺失、超时（90 ms）、非法角度或 NaN 零点时，FOLLOW/SPIN 退回 RELAX，INDEPENDENT 不受影响

## 验证

| 检查 | 结果 |
| --- | --- |
| Debug 构建 | 通过，ELF 1,929,140 B |
| ARM 严格语法 | `chassis.c`、`cmd.c`、`config.c`、`motor_chassis.c`、`can_task.c`、`init.c` 全部退出码 0 |
| 格式扫描 | UTF-8 合法、行宽 ≤120、无行尾空白/Tab、Allman 通过 |
| 主机行为测试（临时，已删） | 40 项底盘断言 + 14 项遥控映射断言全部通过 |
| check.py | HW PASS、SPEC PASS；ARCH 由执行策略拒绝脚本，单独执行 arch-check.ps1 PASS 0 违规 |

## 待确认与未验证

1. 拨杆上位语义：父树是零速刹车，本工作树保留平移，需要用户确认
2. 自旋方向与幅值：现为父树 -0.50（顺时针、约 96.6 °/s），原为本树 +0.30（逆时针、约 58 °/s）
3. 跟随方向：符号来自父树同机器人实测；本机底盘/线束方向或 yaw 编码器安装若不同，需要架空确认
4. 跟随增益 p=0.6、out_limit=1.0 为父树最终配置，首次上板建议小角差、小输出验证
5. 未烧录、未上板；yaw 反馈依赖 CAN1 ID5 新帧，离线时中/下位退化为零输出
6. 四轮速度前馈、架空测速调试入口、chassis_gimbal 状态队列等父树独立能力未移植（非模式依赖）
