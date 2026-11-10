# 参数与 yaw 摩擦补偿提交验证

日期：2026-11-10

## 提交范围

用户要求提交更改区域中选中的 config.c 参数，并删除剩余临时测试文件。本次提交当前 User/module/config.c、gimbal.c、gimbal.h 的既有差异：config.c 已引用 yaw 摩擦补偿字段，因此包含对应配置声明、周期快照校验和控制实现。没有重新设计结构体或调整任何参数数值，只把 config.c 的 pitch 限位集中注释从 ±30° 修正为代码实际的 ±20°

参数相对父提交的主要变化：

| 项目 | 原值 | 当前值 |
| --- | --- | --- |
| 摩擦轮速度环 I | 0 | 0.1 |
| 底盘速度环 P | 0.16 | 0.20 |
| yaw 角度环输出限幅 | 约 33.51 rad/s | 1 rad/s |
| yaw 速度环 P、I | 0.06、0 | 1、0.177 |
| yaw 速度环积分、输出限幅 | 约 2.825、0.5 A | 1、10 A |
| pitch 编码器零点 | 118.08016° | 238.93068° |
| pitch 机械限位 | ±60° | ±20° |
| pitch 角度环 I、积分限幅 | 0.005、1 | 0、0 |
| pitch 角度环输出限幅 | 约 33.51 rad/s | 1 rad/s |
| pitch 速度环 P | 0.06 | 0.3 |
| pitch 速度环积分、输出限幅 | 约 2.825、0.5 A | 1、10 A |
| yaw 摩擦补偿幅值、平滑尺度 | 无接口 | 0 A、0.2 rad/s |

速度 PID 的 10 A 是候选输出限幅，云台两轴最终仍受原有 max_current_a=1 A 约束。yaw 摩擦补偿默认关闭；运行时使用相对底盘的编码器角速度生成 tanh 平滑补偿，与速度环电流相加后统一限流，pitch 不叠加该补偿

RAM config、Control 周期快照、持久 PID 参数引用和 DumpOutput 唯一输出闸门均保留。采样数据、分析文件、硬件 spec 的原有差异不纳入本次提交，硬件 spec 中历史 ±30° 记录不代表本次提交的 ±20° 默认配置

## 验证证据

- python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug：CMake Debug 配置与 Ninja 构建成功，产物 build/Debug/Infantry_Robot.elf
- python .auto-embedded/scripts/check.py：ARCH、HW、SPEC 全 PASS；执行策略仅以 PSExecutionPolicyPreference=Bypass 作用于检查进程及其子进程
- 复用原有 User/task/.yaw_friction_host/test.c 和正式 gimbal.c、pid.c、filter.c、user_math.c，以主机 GCC 的 gnu11、fgnu89-inline、O0、Wall、Wextra、Werror 参数重新编译执行，退出码 0
- 正式 gimbal.c 单独编译时使用原有 hook.h 把 tanhf 接到测试钩子，验证计算期间改变 RAM config 不影响当前周期，下一周期拒绝非法配置；测试文件中的 tanhf 调用仍为真实数学库函数
- 摩擦补偿回归覆盖 yaw/pitch 隔离、正反向、零转速和平滑尺度、底盘运动时编码器方向、速度及角度模式、最终限流、反装方向、在线调参、周期快照、初始化非法参数、运行期非法参数、待机、命令异常与反馈超时安全输出
- 从 compile_commands.json 提取正式 ARM 编译命令，移除 -o、-c、-fstack-usage 后增加 -Wextra -Werror -fsyntax-only，config.c、gimbal.c 全部通过
- config.c、gimbal.c、gimbal.h 的 Allman、120 列、行尾空白和 git diff --check 通过；人工复核中文集中说明、函数参数返回值及配置快照流向

## 三层复核与校验自查

1. 验证门：Debug 构建、真实模块摩擦补偿主机回归、ARM 严格检查及格式检查已通过；本次未烧录、未写目标 RAM、未运行真实电机，不能以此证明新 PID 的动态响应已整定
2. 硬件合规：HW 门禁通过，源码差异没有改变 CAN ID、引脚、时钟、DMA、中断或 CubeMX 配置
3. 代码质量：控制实现仍位于 module；main.c、ISR、临界区和 RTOS/HAL 对象创建不变；新配置字段沿用 RAM 副本和周期快照，不增加跨任务裸共享控制状态

本轮新增判空、isfinite、范围检查或保护性 return 为无。对提交的既有差异复核如下：

- yaw 摩擦电流的有限、非负及不超过 yaw.max_current_a 检查，平滑尺度的有限、正数检查，只在模块配置入口集中进行；实际场景是 EmberProbe 异步写入 NaN、负值、零尺度或超限幅值，上游未校验 RAM 副本
- 合成电流链出口检查由限幅后 output_current_a 改为限幅前 current_a，防止限幅掩盖非有限合成结果；该结果在叠加摩擦补偿后尚无上游出口检查
- yaw_friction 条件只区分轴职责，不是参数重复校验；pitch 不使用补偿
- 本轮未删除任何检查，也未新增下游重复防御检查

## 临时文件清理

清理范围为用户要求删除的 User/task/.chassis_ff_host 和 User/task/.yaw_friction_host，包含测试源码、替身头、基线、对象文件、可执行文件、运行结果及编译参数文件。删除前核对绝对路径、目录内容与 Git 跟踪状态，均为未跟踪临时资产；不删除仓库正式测试文件
