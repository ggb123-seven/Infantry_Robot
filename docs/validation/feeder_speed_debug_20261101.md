# 拨弹单独速度环调试验证

日期：2026-11-01，工作区：F:/RM/Infantry_Robot

## 需求与实现

- 用户要求先关闭位置环以调试 M2006 速度环，沿用此前授权的 module/feeder.c/.h 和 User/task 修改范围
- 现有 Feeder_Config_t、Feeder_t 字段布局不变，未新增结构体；config.c、BSP、设备层、CubeMX 和摩擦轮源码不改
- 旁路位置 PID_Calc，停止逐齿位置与相位推进，保留位置参数和初始化，起控只捕获当前角度供观察
- Feeder_Control 新增直接输出轴 rpm 入参，任务级 volatile float feeder_speed_target_rpm 默认 0，每周期读取一次
- 正负 rpm 直接指定方向，不再乘 config.direction；按既有 max_speed_rpm 限幅，当前初值 300 rpm
- 右拨杆下位使能，上位和中位撤驱；遥控超时、反馈离线、周期非法、故障人工解除及总线发布失败处理保持
- 速度 PID、目标差分前馈、电流限幅与 DumpOutput 唯一输出闸门不变，初始电流上限 4 A
- feeder.config 可写副本和 cycle_config 持久快照保留，PID 长期引用实例内快照
- 原逐齿和位置参数仍参加既有入口配置校验，但不参与当前速度目标生成

## 验证证据

| 命令 | 结果 | 验证边界 |
| --- | --- | --- |
| python .auto-embedded/scripts/check.py | ARCH、HW、SPEC 全 PASS | 实施前和完成后检查分层、硬件锁及 spec |
| python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug | 构建成功 | 产物 build/Debug/Infantry_Robot.elf，Ninja、ARM 工具链 |
| python build/validate_feeder_speed_debug_20261101.py | FORMAT PASS、两份 C 文件 ARM STRICT PASS、30044 项断言 PASS | 临时验证脚本，结果记录后删除 |
| git diff --check | PASS | 修改差异无空白错误 |

- ARM 严格检查复用 compile_commands.json，追加 -Wextra -Werror -fsyntax-only，覆盖 feeder.c 和 motor_trigger.c
- 主机编译真实 feeder.c、config.c、pid.c、filter.c、user_math.c，只替换 bsp/can.h 中宿主机不需要的 HAL 类型依赖
- 5000 周期交替正负目标并改变角度反馈，检查 rpm 直接传递、位置 PID 历史不更新、角度目标固定和相位保持零
- 覆盖零目标、正负 300 rpm 限幅、正负 4 A 限流、FLT_MAX 有限目标限幅、度每秒观察镜像
- 覆盖右拨杆停机、遥控无效及 100 ms 超时、10 ms 反馈边界、离线恢复、NaN/Inf 目标与人工解除
- 覆盖前馈阶跃和恒目标归零、前馈溢出、在线 PID 修改与周期快照隔离、持久 PID 指针、非法周期和非法状态
- 覆盖 32 位低位反馈时间回绕，未进行 MCU 烧录、RAM 写入、硬件运动测试或栈高水位测量
- 临时主机源码、可执行程序与替身头由 TemporaryDirectory 自动清理，临时 Python 验证脚本在记录证据后精确删除

## 校验自查与人工复核

| 新增检查 | 信任边界与实际触发场景 | 上游是否检查 |
| --- | --- | --- |
| Control 入口 isfinite(target_speed_rpm) | 外部在线调试目标可被写入 NaN/Inf，必须进入锁存故障以撤驱 | 无，任务只读一次，不进行重复校验 |

- 未新增判空、范围检查或保护性 return；沿用既有目标限幅和控制出口前馈乘加 isfinite
- 删除不再执行的位置目标及微分出口 isfinite 和保护性 return，因为对应计算链已旁路
- 无新增结构体、RTOS 对象、HAL 操作、ISR 共享状态或临界区；单个 volatile float 由调试器写入、控制任务每周期读取一次
- 人工复核 task -> module -> component/device，任务只采集目标并编排模块，安全输出仍由 DumpOutput 控制
- Allman、120 列、行尾空白机械扫描通过；每个源码文件修改后重新读取 AGENTS.md 和 agent.md
- 新增和修改注释均为 UTF-8 简体中文、无末尾句号，结构字段未变，Doxygen 与控制入口参数已同步

## 调试入口

- 写 feeder_speed_target_rpm 设置目标，单位为减速后输出轴 rpm；上电初值 0
- 修改 feeder.config.speed_pid 的 k、p、i、d 等参数；前馈入口仍为 feeder.config.speed_feedforward_gain_a_per_rpm，初值零
- 观察 feeder.target_speed_rpm、feeder.feedback.rotor_speed、feeder.output_current_a 和 feeder.state
- 本轮生成新 ELF 但尚未烧录；使用新变量前需让运行固件与 ELF 一致，实物方向与闭环响应仍待验证
