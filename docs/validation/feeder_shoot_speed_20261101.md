# 拨弹串级控制与摩擦轮同步验证

日期：2026-11-01，工作区：F:/RM/Infantry_Robot

## 授权与边界
- 用户确认现有 Feeder 结构扩展与 module/feeder.c/.h 修改，内部角度 rad、观察度数，速度内环 rpm
- 用户要求摩擦轮调试注册、前馈与底盘速度 PID；摩擦轮已有设备注册和速度前馈，复用原链路
- config.c、设备层、底盘云台控制源码、CubeMX 不改，不烧录、不写目标 RAM，不增加遥控映射

## 步骤一：拨弹串级
- 500 Hz、右拨杆下位、10 齿/20 Hz、方向 -1、首齿立即和后续 50 ms 行为保留
- 外环输入 rad、输出 rad/s，换算输出轴 rpm 后输入速度内环，输出电流 A
- 外环从 gimbal_config.yaw.angle_pid 复制，range 改 2π、输出限幅按 300 rpm 换算；不沿用旧归一化电流参数
- 内环从 chassis_config.wheel_speed_pid 复制：k=1、p=0.16、i=d=0、i_limit=20、out_limit=4、d_cutoff_freq=20、range=0
- config 可调副本、Control 入口单次读取、cycle_config 持久快照保留，两个 PID 均长期引用快照
- 新增速度与电流上限、speed_pid、speed_feedforward_gain_a_per_rpm，实例增加 rpm 目标和度数观察字段
- 度数与度每秒镜像单向更新，不参与控制；目标差分前馈不除周期，M2006 未标定故默认零
- 起控、恢复与配置变化重置双环，重新捕获角度并预置微分；DumpOutput 是唯一输出闸门

## 步骤一验证
| 命令 | 结果 | 边界 |
| --- | --- | --- |
| python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug | 成功生成 ELF | ARM 编译链接 |
| python build/validate_feeder_shoot_20261101.py | format PASS，ARM strict PASS，20089 项断言 PASS | 实际生产模块与算法，硬件类型头替身 |
| python .auto-embedded/scripts/check.py | ARCH/HW/SPEC 全 PASS | 机械架构、资源锁和 spec |
- ARM 严格检查复用实际 compile_commands.json 命令，追加 -Wextra -Werror -fsyntax-only
- 主机编译真实 feeder.c、config.c、pid.c、filter.c、user_math.c，覆盖首齿、48/50 ms、跨零、单位换算、300 rpm/4 A 限幅
- 覆盖差分前馈与恒目标归零、快照隔离和持久指针、十类新参数非有限、有限系数溢出、故障人工解除
- 覆盖 10 ms 反馈和 100 ms 遥控边界、失联恢复、32 位回绕和连续 5000 周期
- JSON 证据见同名 json，临时主机源码、桩头和可执行程序自动清理

## 校验自查
| 新增或改变检查 | 信任边界及实际场景 | 上游已检查 |
| --- | --- | --- |
| 两环参数有限、增益及积分/输出量程、截止频率范围 | Init 与 Control 入口在线调参，可能写入 NaN、负增益、非法滤波频率 | 本周期仅入口一次，辅助函数只实现入口校验 |
| max_speed_rpm 正值、max_current_a 在 (0,10]、PID 输出限幅与物理上限、range | 在线调参可能改变速度、电流边界和误差语义 | 无，本周期入口一次 |
| 前馈系数有限且非负 | 在线写入 NaN/Inf/负值 | 无，入口一次 |
| 新速度 PID_Init 结果 | 初始化内环失败 | PID 报告结果，由模块聚合 |
| 前馈叠加 current_a isfinite，FAULT 后 return | 新乘加出口，FLT_MAX 系数可导致溢出 | PID 不覆盖后续前馈乘加 |
- 原有目标/微分有限性及时间、反馈、遥控检查保留，未新增实例和句柄判空，没有重复检查 PID 返回值
- 删除旧归一化位置输出乘 10 A 及 (0,1] 输出量程，改为物理速度和电流量程；没有静默兜底参数
- 人工核对 task -> module -> component/device，无新 ISR 状态、RTOS 对象、临界区，总线所有权不变

## CAN 设置号
- 云台 GM6020 设置号 5/6：反馈 0x209/0x20A，电流帧 0x2FE
- 摩擦轮 M3508 设置号 5/6：反馈 0x205/0x206；M2006 设置号 7：反馈 0x207，发射电流统一 0x1FF
- motor_rm.c 对两类控制帧独立缓存和组帧，源码无上述帧冲突；尚未核对实物设置

## 尚待上板
- 未烧录，增益是用户指定参考起点，非已整定参数
- 真实方向、速度响应、电流上限、前馈系数、失联时延、任务栈待上板

