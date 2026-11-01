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
- [坑/gotcha] M2006 旁路位置外环做直接速度调试时，右拨杆下位仅使能，独立速度目标上电为零；无动作先核对可写目标、限幅后目标、实际转速、状态与反馈有效性，不能把非零使能当作非零速度请求。EmberProbe 解释变量前核对固件与 ELF 一致性，运行态多字段采样不是原子快照
