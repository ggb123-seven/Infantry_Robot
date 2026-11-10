# 思维指南 spec（guides）

> guide = "要考虑什么"的思维清单（区别于 spec 的"怎么做才安全"）。本层入口，列出可用 guide。

## 可用 guide
- [`pin-planning.md`](pin-planning.md) —— 引脚规划：复用矩阵、调试口/STRAP 避让、冲突自查 → 写入 hw-lock.yaml。

## 何时用
- RESEARCH 阶段做引脚/资源规划、或面对"该考虑哪些边界"时，先读对应 guide 再动手。
- 新增 guide：在本目录加 `xxx.md`，并在上面"可用 guide"补一行。

## 沉淀（promote 回流）
> REVIEW 阶段把"某类问题该考虑什么"的清单式经验沉淀于此（`task.py promote guides "..."`）。
- [约定] INS 数据全零时先查 task_runtime.init_status 和 INS.module_state：INS.status 的 OK(0) 可能只是零初始化。2026-10-30 板上确认 TASK_INIT_SHOOT_THREAD_FAILED(27)，15360 字节 FreeRTOS 堆启动最低剩余 832 字节，创建 1024 字节栈失败后 Task_Init 回收全部业务对象，INS 尚未运行。DR16、射击和拨弹任务已提供独立静态控制块与 8 字节对齐的 1024 字节栈，构建及 ELF 布局验证通过，尚未烧录验收；详见 docs/workflows/2026-10-30_ins_startup_heap_fix.md
- [约定] 2026-10-30 yaw 无法遥控的板上只读定位：目标 Flash 与 152d0ae 构建 ELF 一致，Task_Init 为 OK，INS RUNNING 且成功更新计数非零，云台 ANGLE、yaw RUNNING、IMU READY；四个底盘 M3508 注册成功但 motor_online 全为 false、源反馈时间全零，独立底盘观测 valid=false，governor_observation_state=UNAVAILABLE(3)，保护每周期跟随当前 yaw 并将速度和电流清零。当前非零 chassis_yaw_rad_s_per_rpm 会启用对四轮观测的依赖，单独调试云台且未接底盘时无法出力；不可改写配置最终值或伪造观测绕过，需明确独立调试模式后另行实现。此次未修改源码或板上状态
- [坑/gotcha] 云台桌面 ANGLE 排查须区分底盘反馈离线与底座机械约束：离线本身不禁止惯性闭环，仅启用既有带转检测；先读 yaw 轴状态，再区分 TRIPPED 撤驱、FAULT、OFFLINE 和 RUNNING 下的方向或动力学异常。IMU 须测得被控上部的运动，桌面放置不能证明底座固定；无板上状态时，合成反馈复现只证明软件判据，不证明现场根因，禁止据此直接删除保护或盲改 PID
- [坑/gotcha] 云台遥控无响应时须同时核对右拨杆位置和命令上下文状态：上位并不等于 ACTIVE，WAIT_NEUTRAL 必须先观测到中位才能就绪，上电、失联或故障均可能重新要求确认。pitch 的 INS 姿态角与编码器相对水平零点的机械角是不同参考；机械角越界会约束目标和向外电流但保留向内恢复，重新使能前先核对实物水平与机械零点，不能用任意当前编码器值直接覆盖零点或删除限位
- [坑/gotcha] 更换云台电机后需分别核对通信标识、实物设备身份、方向和水平编码器零点；方向正确不能证明旧机械零点仍适用。反馈时间持续推进只证明对应 CAN 通道有新数据，未发送非零电流时不能据零输出判断电机驱动损坏。存在使能联锁时先解释联锁，水平姿态确认前不覆盖零点，也不直接取消机械限位
- [坑/gotcha] M2006 旁路位置外环做直接速度调试时，右拨杆下位仅使能，独立速度目标上电为零；无动作先核对可写目标、限幅后目标、实际转速、状态与反馈有效性，不能把非零使能当作非零速度请求。EmberProbe 解释变量前核对固件与 ELF 一致性，运行态多字段采样不是原子快照
- [坑/gotcha] 云台静止跳动对照：摇杆回中时角度闭环仍会输出电流；固定底盘后应联合观察 INS 与电机编码器，积分归零实验须记录旧值与实机结果，编译通过不能证明振荡根因。
- [坑/gotcha] 分析云台装车响应须先核对采样中的实际增益与输出限幅：双环关闭积分后的长期静态残差不能只归因于惯量，需区分机械阻力和驱动执行；软件电流命令不能当作实际电流，缺失遥控列不能按零命令处理。
- [约定] 云台 EmberProbe 手动调环只启用 gimbal_debug_enabled 并复用 gimbal_command；遥控先中再上提供运行许可，回中或失联清 mode 与速度，恢复后须重新写 mode。固定角度用 source=1、angle_deg 与零 rate_rad_s，最后写 mode=2；mode=1 为速度环。手动调试屏蔽正弦，PID 继续写 gimbal.config，详见 docs/validation/gimbal_emberprobe_command_20261108.md
- [坑/gotcha] pitch 追踪先核对命令来源：gimbal_debug_enabled=false 且 source=INCREMENT 时，手写 angle_deg 不是固定角度命令。2026-11-09 只读实测捕获 RUNNING 下约 16 秒停滞，速度积分状态到 -1，但 Ki=0.177 仅提供约 -0.177 A，总电流约 -0.423 A；机械角 +21.45 度时负电流为回程，不被 +20 度最终限位阻断。CAN2 0x20A 反馈电流与发送缓存有响应，用户操作期间电机有实际运动。需分离重力、摩擦、积分上限与目标来源，禁止直接归因失联或盲目放宽限流；证据见 docs/validation/pitch_tracking_20261109.md
- [可复用模式] 底盘跟随云台：复用 MOTOR_RM_Feedback_t 建立独立 chassis_yaw_feedback 最新值邮箱，CAN 任务分发同一有效 yaw 帧给云台和底盘；左拨杆中位进入跟随、上位待机、下位车体自旋。底盘模块用相对零点最短角差旋转平移坐标，比例增益 0.5、转向限幅 ±0.3、反馈超时 90 ms，失效本周期 RELAX。首版软件证据见 docs/validation/chassis_gimbal_follow_20261109.md，转向符号和增益仍需低速上板确认。
- [坑/gotcha] 单一速度点整定的线性速度前馈必须用多速度和正反向验证，不能据平均误差较小就推广到全速域；同时检查原始速度标准差、零速波动与电流指令。零目标时速度前馈为零，仍有波动应独立排查低速闭环。任务周期覆盖命令时，用默认关闭的 RAM 标量入口在遥控解析许可之后路由，保留模块斜坡、限幅与 DumpOutput，结束读回关闭并区分调试关闭和遥控停机。实例与证据见 docs/validation/chassis_feedforward_multispeed_20261110.md
- [设计决策] 摩擦轮安全档立即失能撤驱，只有仍使能的目标变更走一秒线性过渡；每轮保存斜坡终点和变化率，重复命令不重置，中途改目标从当前平滑目标重新计程。过渡和恒速均保持速度环历史，目标差分前馈使用平滑目标；缺命令、离线和故障优先立即撤驱。替代早期安全档零速闭环和无斜坡的历史行为，验证见 docs/validation/shoot_safe_and_ramp_20261110.md
