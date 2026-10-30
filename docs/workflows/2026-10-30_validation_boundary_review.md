# 校验边界与防御性编程审查

审查日期：2026-10-30

最终审查基线：`59c69d0c2363f1363c5052a1488f3f485385cc36`

结论：构建和 ARCH/HW/SPEC 门禁通过，但当前代码未满足用户最新的校验位置规则；发现两类已通过主机复现的功能问题，以及需要精简或调整的校验位置

## 范围与版本边界

- 重点检查 User/module、User/task、PID/滤波/数学组件、CAN 电机及 DR16/IMU 输入链、BSP 时间和启动入口
- 依据用户本轮提供的规则：config.c 冻结、保留 Ozone 可写实例副本、Control 周期快照、集中校验、持久 PID 参数、DumpOutput 唯一状态输出闸门
- 审查期间工作区 HEAD 从 ac3a7c8 切换为 59c69d0，shoot/trigger 及对应任务文件随之移除；本报告仅列当前版本仍存在的问题
- 早期针对 shoot/trigger 的复现结果不用于证明当前版本行为，最终测试已移除对这些文件的依赖并重新执行
- 本轮未修改生产源码、config.c、结构体、CubeMX 配置或用户原有工作区文件
- 本报告是审查结果，不代表所列问题已修复

## 发现

### 1. P1：微秒时间在约 71.6 分钟后回绕，返回类型并未提供完整单调时间

位置：`User/bsp/time.c:32`、`:40`、`:42`

`ticks_new` 和 SysTick 相关运算均为 32 位，`ticks_new * 1000 + 1000 - ...` 先发生 32 位溢出，再转换为 uint64_t 返回。当前 configTICK_RATE_HZ 为 1000，约运行 4294.967 秒即触发

直接编译实际 time.c，用替身提供 RTOS tick 和 SysTick 寄存器，固定 LOAD=167999、VAL=84000，得到：

```text
tick 4294966 -> 4294966500 us
tick 4294967 -> 204 us
```

这会破坏上层关于完整时间单调递增的契约，例如 `User/module/cmd.c:52`、`:88`、`:148` 的帧龄检查，以及 `User/module/gimbal.c:723`、`:745` 的反馈时间检查。源时间低 32 位的模减恢复逻辑不能把已经回绕的 now_us 变为完整时间；64 位时间差也会下溢

建议在 BSP 时间来源解决运算宽度、tick 回绕和采样一致性，再回归所有使用方；只给返回表达式强制转换为 64 位仍需考虑 RTOS tick 自身回绕。本轮仅复现时间倒退，未模拟整机运行到该时刻后的全部恢复流程

### 2. P1：底盘、云台缺少周期配置快照；云台输出能在控制后立即被在线方向修改反转

位置：`User/module/chassis.c:124`、`:140`、`:158`、`:334`；`User/module/gimbal.c:114`、`:164`、`:171`、`:194`；`User/module/gimbal_governor.c:421`

底盘校验实例 config 后，控制计算、PID 同步和导出继续读取同一可写副本；云台将其传入各轴，在子函数内校验并继续读取，governor 也读取实例 config。均没有固定本周期读值

云台的风险可直接复现：先用真实 Gimbal_Control 完成 pitch 速度闭环，再模拟调试器把 RAM 中 config.pitch.reversed 改为 true，调用真实 Gimbal_DumpOutput：

```text
修改前 pitch 输出： 0.060353998 A
修改后 pitch 输出：-0.060353998 A
轴状态仍为 RUNNING，applied_reversed 仍为 false
```

正常方向变更应由 `Gimbal_ApplyAxisConfig`（`:290` 起）清除反馈并对运行轴锁存故障；DumpOutput 的直接读取绕过了这次采纳过程。复现使用函数之间的写入模拟 Ozone 时序，不是硬件调试器实测

建议保留 RAM config，在 Control 入口复制并集中校验本周期快照，参数同步和计算只使用校验后的快照；导出仅使用本周期已经确定的方向和电流。PID 保持指向实例内持久参数，新增实例字段须按规则协商，禁止修改 config.c

### 3. P2：DumpOutput 职责尚未统一到唯一状态闸门

位置：`User/module/gimbal.c:176`；`User/module/chassis.c:170`

云台 DumpOutput 不判断轴运行状态，依赖 Control/StopAxis 预先清零，并且读取实时方向参数。底盘已有状态判断，但同时重新判空、验证 max_current_a 的有限性和正值、检查电流并再次限幅，还继续读取实时 config

建议计算链出口负责数值结果有效性并记录运行状态，DumpOutput 只按相应运行状态导出本周期电流或零，删除重复参数校验与限幅。当前正常任务先调用 Control，因此不能仅由这一结构问题推断每周期必然输出旧电流；方向反转的实际复现见上一项

### 4. P2：固定内部对象和已验证结果仍存在重复检查

| 位置 | 现有检查 | 上游依据与处理建议 |
| --- | --- | --- |
| User/task/motor_gimbal.c:96、:110、:185、:277、:289 | 固定邮箱判空 | Task_Init 创建并检查后才放行业务任务，运行期间不删除，删除循环内生命周期重复判断 |
| User/task/motor_chassis.c:117、:140、:152、:171 | 同上 | 同上，保留实际队列操作返回值处理 |
| User/task/can_task.c:50、:91、:108；User/task/ins_task.c:129 | 同上 | 同上 |
| User/module/chassis.c:288、:292、:296 | 以 PID 参数指针或滤波 b0 是否非零推测初始化 | 应依赖明确初始化状态，不能把可调系数作为生命周期标志 |
| User/module/gimbal.c:407；User/module/ins.c:349 | 对直接 PID_Calc 结果再判 isfinite | PID 已在自身出口处理非有限结果；若后续还有额外计算，检查应放在那条计算链的出口 |
| User/module/cmd.c:52、:88；User/task/motor_chassis.c | 同周期两次解析同一遥控快照时重复判在线/帧龄 | 入口一次验证后供两条命令转换使用，保留源时间而非刷新缓存时间 |

底盘、云台公共函数也仍有对任务独占实例的判空和“可为空”契约，需按固定内部调用链整理。不能批量删除所有 NULL 分支：无新反馈、命令失能或可选底盘观测目前确有 NULL 语义；应区分消息缺席与实例非法

初始化失败回收阶段对已创建对象判空属于真正边界，必须保留。DR16/CAN/IMU 的在线、超时、字段合法性、实测周期检查也应保留。BMI088 数据经过浮点校准与换算，不能把其检查一概视为整数协议冗余

### 5. P2：固定节拍约束仍在运行时兜底，云台重复验证两次

位置：`User/task/motor_chassis.c:44`；`User/task/motor_gimbal.c:71`、`:167`；`User/task/ins_task.c:45`

当前 RTOS tick 固定为 1000 Hz（`Inc/FreeRTOSConfig.h:67`），业务频率也是编译期常量，却使用运行时整除检查、退到一个 tick 或零周期；云台任务入口和私有初始化包装还检查两次

建议将固定频率及整除约束改为 _Static_assert，删除对应运行时兜底；保留每周期实际 period_s 的范围检查，两者检查的对象不同

### 6. P2：业务初始化任务本身创建失败没有可观测处理

位置：`Src/freertos.c:120`

`(void)osThreadNew(Task_Init, NULL, &attr_init)` 丢弃创建结果。内存或 RTOS 资源不足时，Task_Init 根本不会运行，其内部创建失败检查无法覆盖自己未被创建的情形，业务系统缺少明确启动失败路径

建议在启动装配入口检查创建结果并进入可观测失败状态。该位置属于生成代码，本轮未修改；后续修复须遵守源码授权和 CubeMX 用户代码区约束

## 已符合及应保留的部分

- 底盘、云台仍保留实例内 RAM config 副本，未改为直接引用 const 配置
- PID 参数指向实例内持久 applied 参数；本轮对云台指针归属的主机断言通过，未发现 PID 引用栈上局部配置的问题
- Task_Init 在锁定调度期间创建并检查固定邮箱与业务线程，失败时回收已创建对象
- main 维持启动编排，通信任务负责总线汇聚和分发
- 抽查 BSP CAN 临界区未发现阻塞等待、Flash 擦写、日志输出或控制算法
- ARCH/HW/SPEC 门禁通过，不等价于上述动态时序和新规则通过

## 验证证据

最终基线下重新执行：

```text
python .auto-embedded/scripts/check.py
ARCH PASS / HW PASS / SPEC PASS，exit 0

python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug
CMake 配置与 Debug 构建成功，exit 0

python build/rule_review_20261030/run.py
云台方向反转复现断言通过，持久 PID 参数指针断言通过，exit 0

python build/rule_review_20261030/run_time.py
真实 time.c 的 32 位溢出复现断言通过，exit 0
```

主机 GCC 测试直接链接当前生产模块和时间函数；替身只提供所需 CAN 类型、RTOS 接口和 SysTick 寄存器。时间复现编译使用 -std=c11 -O0 -Wall -Wextra -Werror，云台模块复现使用 -std=c11 -O0

审查期间源码切换导致一次扩展测试编译找不到已移除的 trigger.c；该次失败未计入验证通过。去除旧版本依赖后已在最终基线分别重新编译并运行两个复现

本轮临时测试文件在记录结果后清理，未删除正式测试资产。未烧录，未进行真实 Ozone 并发写入、电机运行、超时恢复或栈高水位实测

## 本轮检查增删自查

- 新增生产判空、isfinite、范围检查、兜底赋值和保护性 return：0
- 删除生产检查：0，本轮为审查，未实施源码精简
- 待删除或迁移的重复检查及上游依据见发现 3～5；不能把审查建议记为已经删除
- config.c 的参数、字段、结构和组织方式未由本轮改动

修复顺序建议：完整单调时间；周期配置快照与方向采纳；DumpOutput 状态闸门；按调用链删除重复检查及改用编译期节拍约束；补齐启动入口创建结果处理
