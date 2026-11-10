# 底盘速度前馈多速度架空验证

日期：2026-11-10

## 调试入口与软件验证

源码仅修改 User/task/motor_chassis.c、motor_chassis.h，不新增结构体，不修改 module/config.c 或前馈系数

- chassis_debug_speed_enabled 为 volatile uint32_t，启动为 0，仅值为 1 接管
- chassis_debug_speed_rpm 为 volatile float，启动为 0，表示四轮同向输出轴 rpm
- 遥控先经 Cmd_ParseChassis 处理在线状态、100 ms 帧龄与拨杆，只有解析结果 INDEPENDENT 才接管；当前命令层仅左拨杆下位产生此模式
- 接管时清零平移，以速度符号生成单位旋转向量、绝对值生成轮速尺度；没有绕过模块混控、斜坡、换向保护、限速和 DumpOutput
- 遥控上位仍停止，中位仍跟随，失联和非法拨杆仍 RELAX；调试关闭后恢复原遥控命令
- 每周期只读取一次两个调试标量，无新增 ISR 共享字段、临界区、队列或任务；先写速度再使能，结束先关闭使能

验证命令与结果：

1. 首次 python .auto-embedded/scripts/check.py 因 PowerShell 签名策略无法执行 ARCH，HW/SPEC 通过；以当前进程 PSExecutionPolicyPreference=Bypass 重跑，ARCH/HW/SPEC 全通过，没有修改系统策略
2. python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug，配置和构建通过
3. 复用 compile_commands.json 中 motor_chassis.c 的编译参数，移除 -o/-c/-fstack-usage，加入 -Wextra -Werror -fsyntax-only，退出 0
4. 主机回归使用真实 Cmd_ParseChassis、Chassis_Control、DumpOutput、Mixer、PID 和滤波源码，调试接管片段直接从任务源码提取，PASS groups=31
5. 覆盖默认关闭、使能非法值不接管、零速、正负 0.5/10/20/30/60/100/150 rpm、正负 400 rpm 受原有 300 rpm 上限限制、离线、无缓存、时间回退、帧龄 100000/100001 us、停止档、非法档、跟随模式不接管，以及 NaN/正负 Inf 经模块入口转 RELAX 且 DumpOutput 四轮零电流
6. Allman、120 列、行尾空白扫描和 git diff --check 通过；人工复核新增代码无缺失控制块花括号或机械断行，阶段注释为中文，无新增函数或结构体
7. 临时测试通过标准输入编译，无落盘测试源码；User/task/.chassis_speed_test.exe 运行通过后已删除，原有 .chassis_ff_host 不属于本任务，保持原状

校验自查：未新增判空、isfinite、范围检查或保护性 return；使能值和已解析模式的判断属于命令路由条件。非法速度沿用 Chassis_Control/Mixer 的既有唯一边界检查，未增加重复校验。删除项：无

本步骤只完成软件验证，尚未代表板上多速度验证。构建包含工作区原有 config.c、gimbal.c/h、motor_shoot.c 等未提交修改，不将整份 ELF 归因于本任务

## 板上验证

待记录 EmberProbe 烧录、RAM 写入、各速度采样和最终关闭读回证据
