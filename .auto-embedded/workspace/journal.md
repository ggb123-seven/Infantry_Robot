
## #1  task=解决-cubemx-代码生成后的构建报错  phase=REVIEW
已将当前 M3508 固定 0.8 A 自转控制链路、0x200/0x201 CAN 收发路径及 BSP_CAN_Init 初始化职责整理到 docs/workflows/2026-07/13_analysis_motor_control_can_init.md；当前未修改控制逻辑，后续需补返回值检查、离线零输出和使能/急停状态。

## #2  task=解决-cubemx-代码生成后的构建报错  phase=REVIEW
用户明确新增长期约束：源码默认只允许修改 User/task；分析其他源码可以，若需修改 User/bsp、User/device、applications、Src、Inc、Drivers、Middlewares 或其他源码目录，必须先说明依赖并取得明确同意。该规则已写入根 AGENTS.md。

## #3  task=给底盘电机增加速度环缓启动与调试反馈接口  phase=REVIEW
完成单个 M3508/C620 的输出轴速度环：motor_chassis 作为主运行模块，拥有任务循环、RM 驱动编排、在线判断和电流指令下发；motor_speed_control 只负责这一台电机的目标限幅、150 rpm/s 缓启动、PID 计算、Ozone 调参和目标/真实速度反馈结构体，不包含底盘运动学或多电机同步控制。CMake Debug 构建、Keil XML、ELF 符号与 ARCH/HW/SPEC 门禁均通过。下一步需上板低速确认正负方向，并采集阶跃响应、超调、电流峰值和断线零输出后整定 PID。

## #4  task=can1四个3508电机驱动扩展  phase=REVIEW
已将 CAN1 上的 M3508 从单电机扩展为 C620 ID 1~4：每台独立反馈、目标转速、速度环和 Ozone 监视，四路电流缓存后统一发送 0x200。当前调试固件上电默认开启速度环，暂停 Ozone 前必须先关闭调试使能并等待停机就绪。Debug CMake 构建与 arch/hw/spec 门禁通过；尚未烧录实测，下一步需核对四个物理 ID、安装方向与反馈在线状态。

## #5  task=移植-dr16-遥控器驱动到当前工程  phase=REVIEW
CHASSIS-MERGE-CONTROL-05 已经用户确认并提交本地快照 ebcd75e；当前任务已进入 REVIEW，软件门禁通过：check.py、Debug 构建、DR16/UART/init/chassis CAN 主机回归通过。剩余缺口是 C 板 + DR16 板上实测，包括有效帧率、非法帧、重同步、溢出、UART 错误、最后更新时间、在线状态和任务栈水位。

## #6  task=独立故障检测模块  phase=REVIEW
新增 User/task/fault_detect 独立诊断模块，消费 CANDevices_Snapshot_t 与 Chassis_Snapshot_t 汇总系统级和逐电机故障位，并通过 Ozone g_fault_detect_monitor 发布；CMake Debug 构建、格式扫描、ARCH/HW/SPEC 门禁已通过。

## #7  task=迁移故障检测模块到-user-module  phase=REVIEW
按用户确认将故障检测从 User/task 迁移到 User/module，task 层仅保留 FaultDetect_UpdateMotorChassis 调用和 Ozone 发布；CMake Debug 构建、格式扫描和 check.py 全部门禁通过。

## #8  task=移植通用mixer并接入x型全向轮解算  phase=REVIEW
已移植通用 Mixer，补全四角 X 型全向轮解算并接入 Chassis、Ozone 和 motor_chassis，Debug 全量构建及 ARCH/HW/SPEC 门禁通过，临时测试程序已清理；下一步需上板架空按 ID1 左前、ID2 左后、ID3 右后、ID4 右前验证纯 +vx、+vy、+wz 的轮位和符号

## #9  task=bmi088与ins姿态解算移植  phase=EXECUTE
BMI088 与 INS 移植处于 EXECUTE：第 1 轮 SPI1 10.5 Mbit/s 和 PF6/TIM10_CH1 500 Hz PWM、第 2 轮 User/bsp/imu.h 与 imu.c 板级接口、第 3 轮 BMI088 设备驱动及第 4 轮合并 bmi088_reg.h 均已通过 Debug 构建、git diff --check、ARCH/HW/SPEC；当前正在创建第 4 轮本地快照，完成后进入 Kalman 矩阵组件，最终独立任务名为 INS_Task，1000 Hz 解算、500 Hz 温控，不接入底盘或云台闭环。

## #10  task=移除GM6020电机代码  phase=REVIEW
2026-10-18 按用户明确确认删除 GM6020 专用 task/module 四个文件，清理初始化、独立邮箱、Ozone、CANDevices 和 RM 共用驱动中的专属分支，保留 M3508/M2006 八路槽位与原底盘控制行为。Debug 全量构建、ARCH/HW/SPEC、真实源码前后版本主机回归通过，ELF 无 GM6020/Gimbal 符号，INS/BMI088/BSP/component/.ioc 未改。详细证据见 docs/workflows/2026-10/18_remove_gm6020.md。Keil 原有 RVDS port.c 缺失单独记录，不扩大修复；未提交、未烧录，不据此宣称 Yaw 漂移已解决


## #11  task=yaw系统辨识与pid整定  phase=REVIEW
完成 yaw 速度环系统辨识：探针链路实测上限 73.2/67.3/49.9 Hz，200 Hz 不可达；开环辨识因把含摩擦前馈的 output_current_a 当输入锁死 58.3%，改闭环辨识后拟合 90.1%（T=a/(s+a)，a=33.684 rad/s，被控对象 K=74.853），并按模型整定积分项；上机后大幅动作出现过冲回弹，已回退 i=0 至提交 d870c96，根因是 i_limit=1.0 等于 out_limit，积分可独占全部输出

## #12  task=yaw系统辨识与pid整定  phase=REVIEW
完成 yaw 角度环辨识：对象由速度环闭环级联构造 G=a/(s(s+a))，控制器 C=20+1.0s；闭环为过阻尼（极点 −55.15、−12.21 rad/s，阻尼比 1.30），带宽 3.0 Hz、相位裕度 88.6 度、幅值裕度 9.4 dB、超调 0；实测 ±10/±20/±40 度交替阶跃上升 102–164 ms、稳定 290–550 ms、稳态误差 0.2–0.35 度，与模型一致，角度环参数维持 Kp=20、Kd=1.0、i=0 不动

## #13  task=yaw系统辨识与pid整定  phase=REVIEW
本次结论与踩坑已沉淀：局部 AGENTS.md 新增系统辨识与整定章节（采样上限、输入不可含前馈、闭环辨识、直流增益硬约束、验证工况、i_limit 规则、构建烧录），全局 AGENTS.md 新增闭环辨识通用章节，提交 debd556；分析报告 docs/analysis/yaw_id_final_result_20261117.md 补至 §12。下一步可选：①角度指令 gimbal_command.yaw.angle_deg 不在侧栏采样集，需加入后才能做指令级闭环拟合 ②拟合度冲 95% 需压到 2 变量 67 Hz 或目标侧 1 kHz 环形缓冲 ③角度环若要更快可试 Kp 24–28，但必须按规则实测后再改 config.c

## #14  task=云台小陀螺抖动滤波  phase=EXECUTE
按 docs/validation/gimbal_spin_jitter_20261117.md 第 3 节在 User/module/gimbal.c 实现惯性角速度陷波+低通：Gimbal_Internal_t 增加 yaw/pitch 角速度低通与陷波状态，Init 按控制频率初始化系数，UpdateImu 未就绪时复位滤波器并输出 NAN，就绪时先带阻后低通；未改 config.c、filter.c、BSP 与 CubeMX。A 组（30 Hz/5 Hz）与 B 组（20 Hz/8 Hz）两轮均构建（RAM 50840 B、FLASH 133736 B，Gimbal_UpdateImu 静态帧 144 B）、烧录（J-Link 602710286，Programming Finished + Verified OK，firmwareConsistency match）并以 47~73 Hz 采样复测。静止 yaw 角速度峰峰 0.066→0.0153（A）→0.0099 rad/s（B）；小陀螺同扰动下角度峰峰 A 0.745° 对 B 0.804°（持平），最好窗口 0.487°，均未达 0.2° 目标；起动小陀螺瞬时观察到一次 yaw 被带转约 470°、电流 ±1 A 限幅，需重复验证。已提交 77a9f09、9833de7，板上最终为 B 组。下一步优先机械治理 13.5 Hz 结构共振，或把滤波参数改成 static 可写副本做在线扫参。

## #15  task=云台小陀螺抖动滤波  phase=HANDOFF
2026-11-18 用户判断小陀螺抖动纯属软件问题，另开新对话继续排查，本会话收尾。板上固件为 B 组参数（低通 20 Hz、陷波 13.5 Hz/带宽 8 Hz，SHA256 88073496…，firmwareConsistency match），提交 77a9f09、9833de7、f4d42f8。交接说明写入 docs/validation/gimbal_spin_jitter_20261117.md 第 8 节，含未解问题（同扰动角度峰峰 0.75~0.80°、起动带转 470° 一次）与六个软件候选点：500 Hz 环形缓冲日志、yaw 摩擦前馈改用相对转速、速度环积分、角度环 Kp 提到 24~28、INS 姿态解算抗振、底盘 wz 前馈。

## #16  task=云台 yaw 角度环辨识与摩擦前馈门控  phase=REVIEW
2026-11-19 用户反馈 yaw 偏软、要求 MATLAB 辨识角度外环且拟合度 95 以上。探针链路抖动是天花板（6 变量约 55 Hz，间隔 p95 29 ms、16% 超 25 ms），闭环拟合最好 87.8%；改为在 User/task 新增 500 Hz 片上记录（4096 样本/8.19 s，read.js 分 16 片导出）后，实测 8 次 ±4/±3° 阶跃超调中位 25.9%、上升 66 ms，3 阶闭环拟合 94.57%、极点 -7.842±31.42i/-22.8，平台残差 0.113° 来自 4.5 Hz 摩擦极限环、过渡残差 0.199° 来自黏滑（机构非线性，LTI 上限约 94.6%）。另测 IMU 解算角相对角速度积分只滞后 5.0 ms，速度内环极点由 33.684 升到约 40 rad/s（40 Hz 无陷波滤波）。超调根因定位为摩擦前馈按目标速度施力：关前馈超调降到 12.6%，用户记忆中的 8% 对应「前馈关+略高阻尼」的 7.6%。实现 gimbal.c 的 Gimbal_FrictionFade（目标角速度 ≤0.2 rad/s 全量、≥0.6 rad/s 全关），实测原参数超调 25.9%→10.3%、静差 0.19°；增益扫描综合最优 Kp24/Kd1.2（p=1.2/d=0.06）上升 99 ms、收敛 336 ms、超调 11.1%。RAM 由 50840 B 升到 116384 B（88.79%）全部是 64 kB 记录缓冲，已加编译开关 MOTOR_GIMBAL_ENABLE_DEBUG_LOG 默认 0，正式固件回到 50840 B（38.79%）并烧录（SHA d63b7273…），辨识固件 SHA 2fc07f45…。config.c 未改（仍 p=1.0/d=0.05），1.2/0.06 只在 RAM 试过。证据 docs/validation/yaw_angle_ff_gate_20261119.md，提交 ede2b1d（门控+记录）与本次开关+文档提交。未验证：运行时栈高水位、门控阈值扫描、打靶与连续跟踪工况。
