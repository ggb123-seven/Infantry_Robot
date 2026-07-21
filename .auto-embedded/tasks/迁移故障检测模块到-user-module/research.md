# 研究发现

| 关键词 | 来源 | 摘要 | 可信度 | 状态 |
|---|---|---|---|---|
| 用户授权 | 当前对话 | 用户明确同意将故障检测从 `User/task` 迁到 `User/module`。 | 高 | 已采用 |
| 分层归属 | AGENTS.md、agent.md | `User/module` 负责完整业务能力，`User/task` 只负责任务创建、周期调度、消息路由和模块编排。 | 高 | 已采用 |
| 引用点 | `rg fault_detect` | 迁移涉及 `CMakeLists.txt`、`User/task/motor_chassis.c`、`User/task/ozone_debug.h` 和故障检测自身 include。 | 高 | 已采用 |
