# 2026-10-31 拨弹 M2006 方向反转

- 用户要求将现有 2006 电机方向反转，对应 CAN2 电调 ID7、反馈 0x207
- 唯一源码改动位于 User/task/can_task.c：Feeder_Init 后设置 feeder.config.direction = 1.0f，相对原默认 -1 反转
- Control 首周期复制并校验配置快照，配置变化按既有逻辑重置 PID、相位和位置目标；EmberProbe 仍可修改 RAM 配置副本
- 右拨杆映射、每秒 20 齿、每圈 10 齿、PID 和 CAN 帧组维持现有配置

## 验证证据

- 修改前后运行 python .auto-embedded/scripts/check.py：ARCH/HW/SPEC 全部通过
- python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug：通过，产物 build/Debug/Infantry_Robot.elf
- 使用 build/Debug/compile_commands.json 中 can_task.c 的 ARM 编译命令追加 -Wall -Wextra -Werror -fsyntax-only：通过
- can_task.c 全文件扫描同行左花括号、同行 else、120 列和行尾空白：通过；git diff --check：通过
- 人工复核：配置覆盖在业务任务创建前完成，依赖方向保持 task 到 module，未增加 ISR 共享状态、临界区、RTOS/HAL 对象或新函数
- 初始化失败仍保留模块故障状态，DumpOutput 按现有非运行状态输出零；下游未新增配置重复校验
- 新增判空、isfinite、范围检查和保护性 return：无；本次删除项：无

## 验证边界

本次仅验证软件配置与构建，未烧录、未运行实物电机，实际机械旋转方向等待上板确认；未创建临时测试源码或可执行程序
