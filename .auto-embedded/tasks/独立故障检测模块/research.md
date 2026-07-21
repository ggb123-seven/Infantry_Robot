# 研究发现

| 关键词 | 来源 | 摘要 | 可信度 | 状态 |
|---|---|---|---|---|
| 授权范围 | AGENTS.md、agent.md | 源码改动默认限制在 `User/task`；本次故障检测落为任务层纯诊断模块，不改 BSP、Device、Module 和 CubeMX。 | 高 | 已采用 |
| 状态来源 | User/task/motor_chassis.c、User/task/can_task.c | 底盘任务已有 CAN 快照、底盘控制快照和 Ozone 发布点，可在周期内汇总诊断，不需要新增总线访问。 | 高 | 已采用 |
| 故障边界 | User/device/can_devices.h、User/module/chassis.h | 可复用注册、反馈更新、在线、电流写入、CAN 发送、控制初始化和控制运行状态，覆盖离线、反馈异常、配置异常。 | 高 | 已采用 |
| 构建接入 | CMakeLists.txt | CMake 手工列出 `User/task` 源文件，新增 `fault_detect.c` 后需要加入 target_sources。 | 高 | 已采用 |
