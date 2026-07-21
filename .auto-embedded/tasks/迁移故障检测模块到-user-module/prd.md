# 迁移故障检测模块到 User module

## 需求 / 验收标准
- 将独立故障检测源码从 `User/task` 迁移到 `User/module`。
- `User/task` 仅保留周期调用、消息流和 Ozone 发布编排，不承载故障判定逻辑。
- 构建清单引用 `User/module/fault_detect.c`，且不存在 `task/fault_detect` include 残留。

## 约束
- 用户已明确授权修改 `User/module`。
- 不修改 BSP、Device、Component、CubeMX `.ioc` 和自动生成代码。
