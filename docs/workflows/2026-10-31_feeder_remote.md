# 拨弹遥控接入

## 授权与实现清单

用户已确认修改相关 module、device、task 文件，采用 Feeder_Config_t 与 Feeder_t，并以拨弹盘正面观察顺时针。现有 module/config.c 和 CubeMX 配置不变

1. 隔离 RM 驱动帧组缓存，验证并本地提交
2. 加入拨弹模块、独立遥控邮箱与 CAN 通信编排，验证并本地提交

## 步骤 1：帧组隔离

GM6020 两路输出改用驱动私有数组，不再写入 M3508/M2006 的 0x1FF 缓存。保留公开结构体布局和 API，调试快照按实际发送帧组反映槽位电流；额外改动已在 MOTOR_RM_SetOutput 和 MOTOR_RM_UpdateTxDebug 中写中文说明

- 实际 motor_rm.c 的主机 GCC 严格回归：2000 轮，8 路原生 M2006 加两路外部 GM6020，逐槽核对 0x200、0x1FF、0x2FE，发送失败重试和调试快照均通过
- 真实 ARM 编译参数追加 -Wextra -Werror -fsyntax-only 通过
- python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug：成功，产物 build/Debug/Infantry_Robot.elf
- python .auto-embedded/scripts/check.py：ARCH/HW/SPEC 全 PASS；git diff --check 通过
- 本步骤未新增判空、isfinite、范围校验或保护性 return，仅增加按型号和帧组选择缓存的分支；原有驱动返回值、ISR 接收缓存和临界区不变
- 硬件未改动，不新增引脚、总线、中断或 DMA；未烧录和实测
- 临时测试源码及可执行文件已自动清理，机器结果见 docs/validation/rm_frame_isolation_20261031.json

步骤 1 仅隔离缓存，步骤 2 已启用拨弹周期输出


## 步骤 2：拨弹模块与遥控接入

- 采用用户确认的 Feeder_Config_t、Feeder_t 和独立 dr16_feeder_state 邮箱，不新增任务
- CAN2、M2006、电调 ID7、反馈 0x207、36:1 减速比沿用注册步骤
- 右拨杆下位运行，上位、中位及非法位置撤驱，遥控超过 100 ms 或邮箱异常撤驱
- 反馈缓存最多保留 10 ms，单周期无新帧不重置进给；超过时效为 OFFLINE，恢复时重捕获实测位置
- 配置、周期或计算异常进入 FAULT；参数恢复后须由在线遥控上位或中位解除，再拨下运行
- Init 复制默认值到 feeder.config，Control 每周期复制到 feeder.cycle_config 并集中校验，PID 始终引用持久快照
- 配置变化时集中重置 PID 历史和进给相位；DumpOutput 只按 RUNNING 状态返回电流，否则返回零
- CAN 任务统一提交 0x1FF 的字节 4/5，其他槽位为零；云台仍使用独立 0x2FE
- 新源码已登记 CMake 与 Keil，未修改 module/config.c、CubeMX 配置和数学库

### 参考参数及额外改动说明

参考根目录为 F:/RM/未命名文件夹。can.h 的 CAN_M2006_TRIG_ID 为 0x207，config.c 的 shoot 总线为 BSP_CAN_2；默认步兵配置为每圈 10 齿、减速比 36、位置 PID 的 k=12、p=1、i=0、d=0.0450000018，输入输出滤波截止为 -1

shoot.c 连续模式设置 shoot_freq=20，即每 50 ms 以负角度进一齿，平均输出轴 120 rpm、转子约 4320 rpm。本轮同步的是逐齿位置进给目标，不承诺负载下实测恒速或实际射频

额外改动均已在函数中说明：

1. Feeder_Init 使用实测速度微分，避免单圈角度跨零造成位置差分脉冲，PID 系数仍沿用参考值
2. Feeder_UpdateFeedback 检查低 32 位微秒帧龄，正常回绕不误停，缺一个新帧不重新捕获目标
3. Feeder_Control 在运行入口、反馈恢复或参数变化时捕获当前位置，不追赶停机期间旧目标；故障只能通过在线停机档解除
4. Feeder_Control 将实测周期恢复到整数微秒分辨率，使用双精度相位累积；主机回归曾复现约 45.15 秒时 float 累计误差导致整齿边界晚一周期，修正后通过 50000 个变周期检查
5. Task_can 首次先等待完整控制周期再计算实测周期，避免启动时间不足一个周期时无符号减法下溢
6. MOTOR_RM_SetOutput 与 MOTOR_RM_UpdateTxDebug 隔离 GM6020 和拨弹缓存，防止发 0x1FF 时将云台电流带入摩擦轮槽位

### 检查位置自查

本步骤新增的所有判空、isfinite、范围检查及保护性返回如下：

| 位置 | 实际触发场景与边界 | 上游是否已检查相同条件 |
| --- | --- | --- |
| Task_InitCreateObjects 的拨弹邮箱 NULL 判断和失败 return | RTOS 内存不足，创建边界 | 否，创建结果只能在此取得 |
| Task_InitReleaseObjects 的拨弹邮箱非空判断 | 部分创建失败后回收，仅删除已存在对象 | 否，属于失败清理的对象存在性判断 |
| CANDevices_ApplyFeederCurrent 的初始化状态返回 | 拨弹注册失败，设备不可用边界 | 否，模块 initialized 表示控制器初始化，不代表设备注册 |
| Feeder_Init 的频率有限正值和默认配置校验、失败 return | 初始化错误，初始化边界 | 否；PID_Init 现有公共检查保留，本轮不扩大到组件重构 |
| Feeder_ConfigValid 的齿数、频率、方向及 PID 字段检查 | EmberProbe 异步写入非法参数，在线调参边界 | 否，Control 入口一次校验；子函数和 DumpOutput 不再重复 |
| Feeder_UpdateFeedback 的 angle_valid 和微秒时效检查 | 角度连续性丢失、无新帧、超过 10 ms，输入边界 | 驱动只解析协议并以 100 ms 维护设备在线；本模块执行独立的 10 ms 控制帧龄契约 |
| Feeder_Control 的 period_s 有限及 (0,0.1] | 调度停顿、时间异常，实测周期边界 | 任务只测量并在时间倒退时交付 0，未进行范围校验 |
| Feeder_Control 的遥控有效、online、源时间及 100 ms 时效 | 初始无帧、邮箱错误、失联或未来源时间，外部输入边界 | 接收器维护源状态，任务必须防止缓存继续驱动；此后只用该结果 |
| Feeder_Control 的目标角与微分 isfinite 和异常 return | 在线增益乘法溢出，计算链出口 | 否，不在 PID 后重复检查其输出 |
| Feeder_Control 的非运行状态 return | 已由集中状态机确定安全状态，状态流程 | 不属于重复参数检查，DumpOutput 统一决定零输出 |

删除的重复检查：Feeder_UpdateFeedback 最初对整数协议换算的角度、速度执行的 isfinite；DR16 发布循环的邮箱判空；CAN 周期中云台输出邮箱的判空。邮箱创建已经由 Task_Init 检查，下游只处理 RTOS 操作状态。未新增任务独占实例判空、下游配置校验或 DumpOutput 参数校验

### 验证证据

- REVIEW 先运行 python .auto-embedded/scripts/check.py：ARCH/HW/SPEC 全 PASS
- python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug：PASS，Ninja，build/Debug/Infantry_Robot.elf
- 实际 compile_commands.json 的 ARM 命令追加 -Wall -Wextra -Werror -fsyntax-only：feeder.c、can_devices.c、can_task.c、init.c、dr16_task.c 全 PASS
- 主机 GCC -std=gnu11 -O0 -Wall -Wextra -Werror：直接编译实际 feeder/PID/filter/motor_rm/can_devices/CAN任务/init 源码及提取的原始数学函数、DR16 发布函数，只有 BSP 和 RTOS 边界使用替身
- 模块 50078 个控制周期通过，包含 50000 个 1~3 ms 变周期；核对 50 ms 齿目标、方向、电流限幅、在线配置、非法字段、周期边界、反馈时效、低 32 位时间回绕和故障复位
- 全部 14 个业务对象创建失败点均正确清理；三个遥控邮箱独立消费、部分 Reset/Put 失败和读取失败的离线发布通过
- 实际设备层与驱动：CAN2 ID7、36:1 速度换算、0x1FF 非拨弹槽位为零、与 0x2FE 隔离、发送失败后零值重试通过
- 实际 CAN 任务运行 65 周期：启动时间 0 us、右拨杆启停、100 ms 缓存超时、邮箱异常、反馈中断/恢复及每周期重复零输出通过
- 新增/修改行 Allman、120 列、行尾空白和注释句号扫描通过，git diff --check 与 Keil XML/源文件登记检查通过
- 静态栈记录：Task_can 200 B，Feeder_Control 136 B，Feeder_UpdateFeedback 48 B；CAN 任务分配 1024 B，这不是最坏栈占用或上板高水位证明
- 主机临时测试源码和可执行文件已自动清理，机器证据见 docs/validation/feeder_remote_20261031.json

### 尚未实测的边界

未烧录，未验证从拨弹盘正面观察的实际旋转方向，未测闭环负载速度、控制耗时、CAN 压力与任务栈高水位。本轮按参考正常进给方向设置 direction=-1；若实物相反，通过 feeder.config.direction 调整为 +1

本轮只接入拨弹电机，不加入摩擦轮业务和自动卡弹反转。配置/时序故障后回上位或中位再拨下；普通反馈超时在新反馈恢复时自动重捕获运行
