# M2006 速度调试目标为零的诊断

日期：2026-11-01，工作区：F:/RM/Infantry_Robot

## 现场依据

- 用户反馈仅启用速度环时右拨杆下位没有转动，随后确认尚未设置速度目标
- 只读 EmberProbe 采样 timestamp=1793529450317，firmwareConsistency.status=match
- ELF SHA-256：919bd84edb16026ba7183f8e205aaff3176b0a458068e606acbd8c2ba1b74fe0
- feeder_speed_target_rpm=0，feeder.target_speed_rpm=0，实测反馈转速=0，候选电流=0
- feeder.initialized=1、config_valid=1、feedback_valid=1，trigger_output_published=1
- 本次采样时遥控在线、sw_r=3（中位）、feeder.state=0（RELAX）；不把该时刻结果描述为下位实测
- 遥控源时间 last_online_time 的精确值为 221629958 us

## 源码依据与结论边界

- motor_trigger.c 中 feeder_speed_target_rpm 上电初始化为 0，每周期传给 Feeder_Control
- feeder.c 中右拨杆下位只产生 requested 使能，直接速度目标仍来自调用方
- 当前旁路位置环，不再从 feed_hz、tooth_count、direction 生成速度目标
- 因此只切换拨杆而不设目标，不会产生主动旋转的速度请求
- 当前采样和用户确认支持目标为零这一原因；没有执行非零速度的真实转动验收

## 验证与边界

- python .auto-embedded/scripts/check.py：ARCH/HW/SPEC 全 PASS
- python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --list-presets：Debug、Release 可用
- python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug：编译链接成功
- 只读变量操作使用 .agents/skills/mcu-variables/scripts/read.js，返回 exit_code=0
- 人工核对 DR16 独立邮箱发布、拨弹缓存、模块控制及 CAN 统一电流输出链
- 未修改源码、config.c、.ioc、hw-lock；未新增 ISR 共享量、临界区或 RTOS 对象
- 未烧录、未修改目标 RAM；未新增判空、isfinite、范围检查或保护性 return

## 后续结构调整

- 用户要求将目标 rpm 和实际 rpm 归入速度环结构
- 用户随后确认参考底盘变量写法，已按局部 Feeder_t 方案实现
- 新入口为 feeder.requested_speed_rpm，目标与实测分别为 target_speed_rpm、measured_speed_rpm
- 迁移和验证记录见 feeder_rpm_fields_20261101.md，本次诊断采样数值属于迁移前固件
