# 底盘 M3508 每轮 10 A 电流边界

用户明确要求将底盘电流上限改为 F:/RM/未命名文件夹 底盘 M3508 的一半。参考工程归一化输出上限为 1，对应 20 A，故本工程每轮设为 10 A

本次按用户参数修改授权调整 User/module/config.c 中 chassis_config.max_current_a 与 chassis_config.wheel_speed_pid.out_limit，均由 4 A 改为 10 A，并更新该配置上方的集中说明。不更改 RAM 配置副本、快照或结构组织。已有底盘 p=0.18、云台参数及其他用户工作树差异不纳入本次提交

## 验证

- 使用 build/Debug/compile_commands.json 的 config.c ARM 编译参数，去掉 -o、-c、-fstack-usage，添加 -Wextra -Werror -fsyntax-only，退出 0
- 核对配置中两个电流边界均为 10.0F；检查修改区 Allman、120 列、行尾空白及 git diff --check，全部通过
- python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug：配置及编译退出 0，产物 build/Debug/Infantry_Robot.elf
- python .auto-embedded/scripts/check.py：ARCH、HW、SPEC 均 PASS，执行策略仅通过进程环境变量 Bypass 影响检查子进程

人工复核：Chassis_RunWheel 的 PID 输出与最终 AbsClip 共用 10 A 边界；M3508 驱动按 20 A 对应 16384 换算，10 A 对应 8192。初始化仍复制配置到实例 RAM，控制入口仍采纳本周期快照，输出闸门仍按运行状态放行或归零。没有改变分层依赖、RTOS/HAL 对象创建、ISR 共享数据、main.c、临界区或硬件资源

本次新增或删除的判空、isfinite、有效性范围检查和保护性 return：无。只更新已有两个限幅阈值，无新增重复校验。未创建临时测试源码或测试程序

本次未烧录、未写板上 RAM、未运行电机验收。生成 ELF 包含工作树中原有的其他改动，构建成功不能代替上板验证
