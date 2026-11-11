# 底盘跟随云台 PID 方案 A

日期：2026-11-11

状态：用户已确认方案 A 并授权实施，软件验证通过，实车与运行时栈验收未完成

## 目标与现状

用户要求先规划 follow PID，结构体参考其他模块；先确认按 gimbal/feeder 的公开配置与观测、私有控制历史风格设计，随后选择方案 A 的统一配置入口

实施前链路为：遥控解析 → Chassis_ResolveGimbalFollow → Chassis_Control → 混控 → 四轮速度环 → DumpOutput → CAN 任务

实施前源码证据：

- chassis.c 的 CHASSIS_FOLLOW_GAIN 实际为 0.05，注释和 2026-11-09 验证记录写的是 0.5，存在十倍差异
- 当前算法为 wz = clip(0.05 × relative_rad, ±0.3)，已经属于纯比例跟随
- cmd.c 的常规轮速尺度为 100 rpm，10° 角差对应 wz 约 0.00873、纯旋转请求约 0.873 rpm，尚未计入轮速斜坡和执行误差
- 最短角差绝对值不超过 π，现有最大比例请求约 0.157，无法达到配置在宏中的 0.3 限幅
- 此证据支持优先检查跟随增益与输出尺度，不能据此认定更换 PID 后实车效果已经改善
- 实际拨杆行为以上位零速闭环刹车、中位跟随、下位云台坐标平移并顺时针自旋为准，历史文档部分描述已经过时

## 控制方案

采用现有 KPID_t，跟随角度环输出归一化 wz，再交给现有四轮速度环

```text
yaw 编码器反馈 → 单一反馈边界检查 → 相对车头角度及相对角速度
                                                ↓
遥控命令 → Chassis_Control：配置快照、校验、模式转换、follow PID
                                                ↓
                         云台坐标平移转换 + 归一化 wz
                                                ↓
                         混控 → 既有轮速斜坡与速度环 → DumpOutput
```

1. 跟随目标为相对 yaw 角归零，正角差仍产生当前约定的正转向
2. PID 输入使用弧度，输出使用归一化 wz，禁止将输出直接当作 rad/s 或电流
3. 使用最短角差，保持现有编码器零点和 reversed 方向约定
4. 计算采用 sp=0、fb=-relative_rad、range=2π；误差与当前正向比例控制一致
5. 使用 KPID_MODE_SET_D，fb_dot=-relative_speed_rad_s，由已有 yaw 电机转速提供，避免对跨 ±π 的角度直接差分
6. SET_D 使用直接速度输入，当前 PID 组件不对该输入应用微分低通；首轮 d=0，后续若启用 D，先检查速度噪声，不能把 d_cutoff_freq 当作已生效的滤波
7. follow_pid.out_limit 是跟随 wz 的唯一专用限幅，不再保留 CHASSIS_FOLLOW_MAX_WZ 的重复裁剪
8. 平移转换和跟随 PID 均在 Control 内执行，解析反馈接口不再修改命令；SPIN 只执行坐标转换并使用遥控给定的 wz

首轮规划参数：k=1、p=0.5、i=0、d=0、i_limit=0、out_limit=0.3、d_cutoff_freq=-1、range=2π

这些数值用于首轮低速整定，不是实车最终值。p=0.5 时 10° 角差对应约 8.73 rpm 的纯旋转请求；先记录响应，再决定是否加入 I 或 D。复用 PID 组件现有积分行为，本轮不改 pid.c

## 已确认的结构体设计

采用方案 A，以下为新增字段及职责，既有结构保留

| 位置 | 字段 | 含义 |
| --- | --- | --- |
| Chassis_Config_t | KPID_Params_t follow_pid | 跟随角度 PID 参数，EmberProbe 入口为 chassis.config.follow_pid |
| Chassis_t | float follow_relative_angle_deg | 相对车头角度，单位度，仅观测 |
| Chassis_t | float follow_relative_speed_rad_s | 云台相对底盘角速度，单位 rad/s，仅观测 |
| Chassis_t | float follow_wz | PID 本周期输出的归一化转向请求，非跟随时为零，不代表车体实测角速度 |
| chassis.c 私有 Chassis_Internal_t | Chassis_Config_t cycle_config | 本周期通过校验的持久配置快照 |
| 同上 | KPID_t follow_pid | 跟随 PID 积分、微分与上一输出历史 |
| 同上 | bool yaw_feedback_valid | 反馈边界产生的有效性结果，供 Control 直接使用 |

PID 参数指针指向私有 cycle_config.follow_pid，禁止指向栈上临时快照。参数变更检测在 Control 入口完成，覆盖快照前比较旧参数，随后同步并清理对应历史

沿用现有 Chassis_t.mode 和单轮状态机，不新增重复的跟随运行状态；既有 wheel、mixer、轮速 PID 布局保留，本次只为 follow 引入必要私有上下文

可读性：配置入口与云台、拨弹一致；职责边界：参数和观测公开，控制历史私有；扩展性：后续 PID 调参无需追加离散全局量；兼容性：保留已有字段及四轮观测路径，但结构体尺寸和调试符号布局变化，调试器需加载新 ELF

方案 A 的 config.c 扩展严格限定为 chassis_config 追加 follow_pid 初始化及对应集中说明，既有参数值、字段和组织方式保留。此范围已在方案选择前向用户说明，不扩展到其他配置重构

备选 B 不采用，不新增 chassis.follow_config 这一独立调参入口

## 接口与状态交接

- 以 Chassis_UpdateGimbalFeedback 替代原 Resolve 接口，接收现有 yaw 缓存、当前时间、已标定零点和 reversed，完成反馈检查及观测更新
- 电机反馈仍经底盘专用最新值邮箱传入，不读取云台任务的裸全局运行实例，不新增邮箱或硬件资源
- Chassis_Control 先复制配置、集中校验和同步参数，再决定 next_mode，只有合法 FOLLOW 执行跟随 PID
- 从其他模式进入 FOLLOW：使用初始化或上次退出时已清空的历史，从当前有效机械反馈开始，目标仍为相对角归零，不捕获偏角作为新零点
- 离开 FOLLOW、跟随反馈失效或配置非法：集中清理跟随历史并取消跟随请求；持续待机期间不反复初始化
- FOLLOW/SPIN 的 yaw 反馈失效：沿用当前整组 RELAX 路径；独立零速刹车不依赖 yaw 反馈
- 恢复后重新建立跟随历史，四轮沿用现有从实测转速起步及斜坡行为
- 非法模式、初始化失败、周期异常与反馈失联仍由既有状态转换进入安全输出，DumpOutput 只根据运行状态放行电流
- 参数改变时仅在入口处理积分和必要历史同步；不在辅助函数、状态处理和 DumpOutput 重复检查

## 实施清单与预计范围

| 顺序 | 文件或工作项 | 内容 | 验证 |
| --- | --- | --- | --- |
| 1 | 结构与范围确认 | 用户已选择 A，统一配置入口、必要观测和私有控制历史按上表实施 | 已确认并授权实施 |
| 2 | User/module/chassis.h、chassis.c | 接入反馈入口、私有 PID、快照、模式交接与必要观测 | 主机回归、实际 ARM 严格检查、Debug 构建 |
| 3 | User/module/config.c | 追加 follow_pid 初值与集中说明 | 核对既有配置值零变化，与步骤 2 作为完整改动验证 |
| 4 | User/task/motor_chassis.c、motor_chassis.h | 改为先更新反馈，再由 Control 计算跟随；发布失败仍走既有撤销路径，同步调参入口说明 | 任务调用顺序、邮箱失败及状态发布回归 |
| 5 | 文档与收尾 | 更新调参路径、验证证据，清理本任务临时测试程序 | 格式扫描、ARCH/HW/SPEC、git diff --check |

已按 A 实施 User/module/chassis.h、chassis.c、config.c 和 User/task/motor_chassis.c、motor_chassis.h，其中 config.c 仅追加上述跟随参数及说明。完整验证见 docs/validation/chassis_follow_pid_20261111.md

不依赖修改 BSP、设备协议、PID 组件、CAN 任务、遥控映射、生成代码或 .ioc

## 验收计划

软件回归覆盖：零点和 reversed、正反角差、359°/0° 与 ±179° 跨界、180° 分界、坐标变换长度保持、跟随限幅、非法在线参数、PID 参数指针持久性、参数变更、实际周期变化、90 ms 超时边界及低位时间戳回绕

状态回归覆盖：FOLLOW 进入退出、SPIN 互切、独立刹车、反馈失效与恢复、初始化失败、RELAX、发布失败复位、四轮离线与恢复，确保重复周期不重复初始化

架空先核对小角差正负方向和输出限幅；落地低速对比同一组 ±10°、±30° 目标及连续转动，记录角差、跟随 wz、四轮请求/斜坡/实际转速、电流、模式和时间戳

用相同负载与命令对比旧 0.05 比例、候选 P 和后续 PI/PD，报告收敛时间、超调、稳态角差、持续振荡及限幅占比。数值验收门槛需结合首轮实测确认，不能用架空轮速或构建通过替代落地跟随效果

## 校验边界规划

| 检查 | 实际触发场景 | 唯一位置与上游关系 |
| --- | --- | --- |
| yaw 反馈缺失、过期、角度非法及丢帧 | 邮箱未到帧、CAN 中断、反馈连续性丢失 | 反馈入口，迁移既有边界检查，Control 只用有效性结果 |
| follow PID 参数有限性、范围及固定单位约束 | EmberProbe 异步写入非法参数 | Init 与各次 Control 入口，属于不同生命周期边界，下游不重复 |
| 实际 period_s 合法范围 | 调度延迟或无效间隔 | 沿用 Control 入口检查，follow 不再单独检查 |
| 浮点控制结果 | PID、混控或前馈计算异常 | PID 沿用组件出口，后续有额外计算时只在该链出口检查，不重复检查裸 PID 返回值 |

不新增任务独占实例判空，不对整数电机转速换算量追加 isfinite，不新增任务循环句柄判空，不在 DumpOutput 重验参数；实际新增及删除检查清单由实施后的逐文件审查提供

## 规划阶段证据与限制

- 仅新增本规划文档，未修改生产源码，未新增或删除任何运行时检查
- 当前工作树已有多处未提交修改，方案依据当前文件内容，不覆盖这些修改
- Debug 基线：python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug，配置与增量构建成功，产物 build/Debug/Infantry_Robot.elf
- 首次 check.py 的 ARCH 被 PowerShell 脚本签名策略阻断；通过仅当前进程的 ExecutionPolicy Bypass 重跑完整 check.py 后 ARCH、HW、SPEC 全部 PASS，未修改持久执行策略
- 硬件复核：方案复用既有 yaw 邮箱、CAN 反馈和底盘周期，不申请新引脚、DMA、中断或外设
- 并发复核：规划中的控制器由底盘任务独占，在线配置每周期读取快照，不新增 ISR 共享业务状态或临界区
- 用户选择 A 后再次执行完整 check.py，ARCH、HW、SPEC 全部 PASS；本轮定稿只编辑文档，沿用前述 Debug 基线结果
- 上述构建和门禁仅证明现有基线，未运行新 PID 功能测试、烧录或实车验证；已确认设计与已实施行为分别记录

## 实施结果

- 模块真实代码主机回归 17,719 条断言、任务 17 场景 1,020 完整周期及 6,768 条断言通过，三个 C 文件 ARM 严格检查和最终 Debug 构建通过
- ARCH/HW/SPEC、格式与差异检查通过；config.c 去除本任务新增文本后与任务开始时基线逐字一致
- 跟随入口为 chassis.config.follow_pid，PID 指向持久私有快照，公开三个跟随观测字段；初始 P=0.5、I=D=0、输出限幅 0.3
- RAM 50,616 B、FLASH 131,612 B；底盘任务栈配置 1024 B，Task_motor_chassis 静态帧 144 B、Chassis_Control 静态帧 208 B，尚无本次固件运行时高水位
- 未烧录、未写目标 RAM、未实车测量；软件通过不能证明跟随效果改善，下一步按上方架空及落地验收计划执行
