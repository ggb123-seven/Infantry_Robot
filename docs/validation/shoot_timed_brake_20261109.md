# 摩擦轮回中后定时制动验证

日期：2026-11-09

## 用户确认的行为

右拨杆由下位拨到中位后，以 0 rpm 目标闭环制动，从首次收到该命令开始计时，经过 3 秒关闭速度环。取消双轮速度均在正负 1 rpm 内连续保持 20 ms 的旧判据，本记录替代 shoot_stop_disable_20261104.md 中的反馈停稳策略

- 下位仍请求双轮 8000 rpm
- 首次零速使能命令记录 BSP_TIME_Get 的完整单调微秒时间，阈值为 3000000 us
- 定时状态在 Shoot_Control 前推进，首次观察到经过时间不小于 3 秒的控制周期直接提交失能，清除 PID 历史并经 Shoot_DumpOutput 发布零电流
- 重复零目标、任意转速、反馈缺帧和空命令邮箱均不重置计时；空命令邮箱仍按既有规则使本周期失能，不能利用缓存命令继续驱动
- 持续中位不重新使能；下位非零目标取消当前制动，下次回中重新计时；上位和遥控失效继续立即撤驱
- 反馈超时、非法配置、周期异常等继续由 Shoot 模块原有信任边界处理，定时逻辑不以故障代替计时，也不解除模块故障锁存

计时起点是摩擦轮任务消费到首个回中命令的周期，不是遥控器物理触点切换的精确时刻。名义任务频率 500 Hz，实际撤驱还受任务调度及 CAN 输出周期影响。定时撤驱只限制零速闭环持续时间，不改变这 3 秒内的控制增益或电流上限

## 现场诊断依据

修改前只读复用了 EmberProbe 已运行采样连接，ELF 与最近烧录记录 SHA-256 均为 5b1764acd938620fa4ad3e5f31159e0c5537a1b144a7d88e0b354368c4524f78

震动期间目标均为 0 rpm，但 shoot_command.enabled=1、shoot.state=SHOOT_RUNNING、shoot_stop_state=BRAKING，采到转速 93 rpm 和 -63 rpm，候选电流反复换向并多次到达正负 3 A。生效参数 k=1、p=0.16、i=d=0。运行时多变量读取不是原子快照，不据此逐条拟合转速和电流的同周期关系

## 变更范围

User/task/motor_shoot.c 删除低速阈值、反馈稳定时间、确认状态和双轮确认起点数组，改为单个制动起点及空闲、制动、已撤驱三态。motor_shoot.h、motor_trigger.c、user_task.h 同步注释。未新增结构体或 RTOS 对象

config.c、gimbal.c、gimbal.h 和硬件 spec 在本轮开始前已有用户工作树修改，本轮没有更改或暂存这些差异。固件构建包含当前工作树已有内容，不能将新 ELF 当成仅含本次停机变更的独立版本

## 验证命令与结果

```powershell
python User/task/.shoot_timer_host/run.py
python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug
$env:PSExecutionPolicyPreference = 'Bypass'
python .auto-embedded/scripts/check.py
git diff --check -- User/task/motor_shoot.c User/task/motor_shoot.h User/task/motor_trigger.c User/task/user_task.h
```

- 主机测试直接包含正式 motor_shoot.c，链接正式 shoot.c、config.c、pid.c、filter.c、user_math.c，使用正式 Task_Runtime_t 和 MOTOR_RM_Feedback_t；仅替换 BSP_CAN_t、FreeRTOS 节拍及 BSP 时间，模拟 CMSIS 邮箱和任务调度
- 主机 GCC：gnu11、fgnu89-inline、O0、Wall、Wextra、Werror、函数与数据分段及链接回收；12 组共 110869 条断言通过
- 覆盖反馈微秒低 32 位回绕、3 秒前 1 us 与恰好 3 秒、提前达到零速仍不早退、转速持续摆动、反馈失联仍按原起点到期、空命令后恢复不重计时、制动中重新运行后再次计时、明确失能、上电中位失能、撤驱后再次启动、到期时空命令不复活、内核节拍回绕以及非法状态安全撤驱
- 真实 Shoot_DumpOutput 发布值在到期周期为两轮 0 A，PID last.out 同期清零，持续零目标不再使能；反馈缺失时模块原有离线输出仍为零
- 两份任务使用 compile_commands.json 中正式 ARM 编译参数执行 Wextra、Werror、fsyntax-only，均退出 0
- Debug 配置与构建退出 0，产物 build/Debug/Infantry_Robot.elf
- ARCH、HW、SPEC 均 PASS；执行策略变量仅作用于本次进程及子进程，没有更改持久系统设置
- 四份源码 Allman、120 列、行尾空白及 git diff --check 检查通过，人工核对阶段中文注释、函数参数返回值和可单行表达的语句

测试完成后清理本任务临时创建的 User/task/.shoot_timer_host，不删除原有 .yaw_friction_host 或其他工作树文件

## 三层复核与校验自查

1. 验证门：真实任务和模块主机回归、ARM 严格检查、Debug 构建及格式通过，未把合成反馈等同于实物测试
2. 硬件合规：没有改变引脚、CAN ID、时钟、DMA、中断或 CubeMX 配置，HW 机械门禁通过
3. 代码质量：任务只编排命令及定时，仍按 task -> module -> component/device 调用；main.c、ISR、临界区及 RTOS/HAL 对象创建不变；新单个时间戳由任务独占，不需要 volatile；实际输出仍由 DumpOutput 一处闸门保证

新增判空、isfinite、参数范围检查及保护性 return：无。唯一新增时间比较 now_us - shoot_stop_since_us >= 3000000 是定时状态迁移条件，上游未检查同一业务截止时间。移除任务内两轮绝对转速范围判断、连续反馈时长判断，以及反馈新鲜度和 SHOOT_RUNNING 对停稳计时的重复约束；模块原有配置、反馈和周期校验保留

本次没有烧录、写目标 RAM、停核或复位。新策略的实际 3 秒撤驱时刻、真实电流和制动期间震动尚待新固件上板测量
