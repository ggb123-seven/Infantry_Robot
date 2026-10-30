# yaw SMC 移植实施记录

## 基线与授权

- 实施分支：feature/gimbal-smc，起点 59c69d0，与 main 独立，不合并分支
- 用户确认修改 SMC、gimbal、gimbal_governor 及相关接口，优先复用 SMC_t 和原云台变量名
- 不修改 config.c，不修改设备协议、INS、CubeMX 或硬件资源
- 现有两轴 PID 配置仍从 config.c 复制到 gimbal.config，保留 Ozone 调参入口

## 调整后的方案

1. 复用 SMC_t，保持其字段布局，不新增结构体类型或字段
2. gimbal_yaw_smc_config 仅提供 SMC 参数，gimbal_yaw_smc 提供实际计算与参考轨迹观测，均复用 SMC_t
3. Control 入口对 gimbal.config 和 SMC 参数取本周期快照，集中校验与同步；反馈使用最近有效快照
4. yaw ANGLE 通过显式目标角速度、角加速度计算 SMC 电流；pitch 和 SPEED 继续 PID
5. 用连续可导的近零多项式衔接原终端滑模幂函数，避免零误差撤驱和一度边界电流跳变
6. 参考轨迹按实际 period_s 推进，限制参考速度和加速度；保护捕获及模式切换重置历史
7. governor 继续限制最终电流，SMC 只提交实际电流历史，不修改闲置 PID 积分
8. DumpOutput 按初始化、配置和轴状态统一阻断非运行输出，方向取已应用配置

## 参数与兼容性

- 保留原 SMC_t 19 个 float 字段和排列顺序，error_eps 改为近零连续过渡尺度，单位度
- SMC_Tick 显式接收参考角速度和角加速度，不再隐式使用无时间量纲的差分
- 初始试验参数 C=5、K=10、J=0.001、epsilon=0.5、q=21、p=27、error_eps=1、u_max=0.5 A
- 参数仅作为未上板的试验起点，不声明为本机已整定值；SMC 候选电流上限取 u_max 与原轴限流的较小值，governor 独立制动仍服从原轴限流
- 固定参考速度不超过 60 度每秒且服从原轴速度限制，参考加速度不超过 180 度每平方秒
- 保留固定单位边界层 sat(s)，不为可调 phi 扩展结构体
- SMC 参数在 Control 入口校验并实时同步，不恢复旧目标，不沿用旧目标导数
- Ozone 多字段写入不具备事务原子性，快照只保证同一周期不重复读取；联动调整宜在 RELAX 完成

## 实施清单

| 步骤 | 文件与接口 | 验证标准 |
| --- | --- | --- |
| SMC 数值适配 | slidingmodec.c/.h、readme.md：SMC_IsConfigValid、SMC_Tick | 正负误差、过零制动、连续导数、前馈、限流及异常输出测试 |
| 云台接入 | gimbal.c/.h、gimbal_governor.c/.h | ANGLE 使用 SMC，pitch/SPEED 回归，保护与故障零输出，配置快照 |
| 验收记录 | 本文、validation 记录、spec | Debug 构建、严格编译、ARCH/HW/SPEC、格式检查、主机集成测试 |

每个独立实现点完成后重读 AGENTS.md 与 agent.md、检查并记录证据，再提交本地 Git；不推送远端

## 验证边界

不烧录、不驱动真实电机；实机方向、负载稳定性、温升、计算耗时和 PID 对比录波需上板完成

## 第一步验证

- 主机 GCC 严格 C11 编译及实际 SMC 断言测试通过，扫描 40001 组误差，正负过渡边界电流跳变量为 1.14366412e-6 A
- 覆盖正负反馈符号、零位置误差时制动、等速参考零纠偏、解析加速度前馈、限流、非法参数及非有限计算出口
- `python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug -j 2` 通过，SMC 尚未接入时仅证明编译链接
- 新增校验仅为 `SMC_IsConfigValid` 的配置输入边界及 `SMC_Tick` 的浮点计算出口；移除原实例判空及角度死区保护性返回

## 云台接入结果

- yaw 的 ANGLE 路径由 SMC 直接生成安培电流，不再执行原角度环与速度环；pitch 及 SPEED 路径保留 PID
- 通用参考生成放入 `SMC_AdvanceReference`，模块只负责参考捕获、单位换算和保护编排
- 起控、模式切换、电机失联恢复以及保护接管时捕获当前姿态，清除旧参考导数；真实反馈速度仍参与制动
- pitch 失能只复位 pitch，不重置 yaw 的参考轨迹；SMC 参数非法时 yaw 锁存故障，不回退 PID
- 单云台任务独占静态配置快照和上下文，沿用 governor 原有单实例限制；未增加结构体字段
- `gimbal.config` 保留 RAM 调参副本，Control 入口复制并集中校验，PID 引用轴内持久参数副本；反馈使用最近有效快照，DumpOutput 使用已应用方向
- `config.c` 相对 `59c69d0` 无差异，SMC_t 的 19 个 float 字段及顺序保持一致

## 最终软件验证

验证环境为 Windows PowerShell、主机 GCC 和工程 Debug 预设使用的 ARM GCC 14.3.1，生产平台为 STM32F407IGH6

| 检查 | 命令或方法 | 结果 |
| --- | --- | --- |
| SMC 数值 | `gcc -std=c11 -Wall -Wextra -Werror -pedantic -I User build/smc_port_check.c User/component/SMC/slidingmodec.c -lm -o build/smc_port_check.exe`，运行生成程序 | 40001 组扫描及符号、制动、前馈、限幅和异常断言通过 |
| 生产模块集成 | `python build/run_smc_integration.py` | 22242 周期通过，含 12000 周期旧实现对照 |
| 固件编译链接 | `python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug -j 2` | 通过 |
| ARM 严格语法 | 提取 compile_commands.json 中三个修改 C 文件的实际命令，去掉 -o、-c，增加 -Wextra -Werror -fsyntax-only | 通过 |
| 分层、硬件和规范 | `python .auto-embedded/scripts/check.py` | ARCH、HW、SPEC 全部通过 |
| 格式与兼容 | UTF-8、Allman、120 列、行尾空白、`git diff --check`、Keil XML 解析 | 通过，未执行 Keil 编译 |

主机集成编译真实 gimbal、governor、SMC、PID、滤波、数学和展开组件，仅以设备头替身隔离 HAL，反馈类型取自真实 motor.h，默认云台参数取自 config.c；旧模块取自 `59c69d0`

- SPEED 两轴及 ANGLE pitch 与旧实现输出精确一致；yaw SMC 不修改闲置 PID 积分
- 覆盖起控捕获、2/4 ms 变周期、参考速度与加速度约束、过零速度制动、pitch 失能、在线限流及配置周期快照
- 覆盖非法命令、非法 SMC/云台配置、零/负/非有限/超限周期、非法状态、初始化失败、电机与 IMU 失联及恢复
- 覆盖四轮观测无效撤驱、持续异常残差撤驱、残差恢复后的捕获和重新运行，以及 32 位低位时间戳回绕
- 正负 10 度理想二阶模型共 8000 周期，末位置误差约 2.19e-5 度，仅说明该理想模型下闭环计算成立
- ELF 保留 `gimbal_yaw_smc_config`、`gimbal_yaw_smc`、`SMC_Tick` 和 `SMC_AdvanceReference` 符号
- Debug size：text=129152 B、data=232 B、bss=43000 B；静态栈记录 Init=288 B、Control=328 B、RunAxis=104 B，均非调用链总栈或实测高水位

## 校验边界自查

下表包含新增或迁移的判空、有限性、范围及保护返回；普通状态转换和轨迹限幅属于控制行为

| 位置与条件 | 实际触发场景及边界 | 上游是否已检查同一条件 |
| --- | --- | --- |
| Init 配置、SMC 参数和 10～2000 Hz 频率，失败返回 -1 | 初始化传入非法参数，初始化边界 | 未检查；轴初始化不重复校验 |
| Init 两轴 PID 初始化返回值 | 下层初始化失败，初始化结果边界 | 未检查；沿用结果传播 |
| Control 完整配置与反馈超时范围 | Ozone 写入非法 RAM 配置，在线调参边界 | Init 检查不能代表运行期；本周期只检查一次 |
| Control SMC 的 C、K、error_eps、q/p、u_max、J、epsilon | Ozone 写入非有限值、零分母、非法增益或上限，在线调参边界 | 本周期未检查；算法内部信任快照 |
| Control 初始化状态、period_s 和模式枚举，失败转故障并返回 | 初始化失败后任务继续调度、调度周期异常或非法命令，运行与时间边界 | 上游不保证有效；下游重复检查已删除 |
| ResolveSetpoint 命令速度、绝对角度和来源枚举，失败返回 false | 非法命令不得被起控姿态捕获掩盖，命令输入边界 | Control 在此统一解析，下游不重复检查 |
| SMC_Tick 限幅前 current_a 有限性，失败返回 false | 极端有限参数造成浮点溢出，SMC 计算链出口 | 输入合法不能排除运算溢出 |
| RunAxis 最终 output_current_a 有限性 | governor 合成或积分反算后的计算异常，最终合成链出口 | SMC/PID 只检查各自输出，后面还有保护合成 |
| 反馈中的新帧指针及帧龄、数值、连续性检查 | 邮箱本周期无新帧、设备掉线或非法反馈，外部反馈边界 | 保留既有检查，无新增同条件重复检查 |
| DumpOutput 初始化、配置结果、IMU 和轴状态 | 停止、配置失败、失联或故障后禁止输出，最终输出闸门 | 使用既有状态结果，不重读参数及中间量 |

删除了实例判空、轴初始化/配置同步/RunAxis 中重复的配置与周期检查、governor 中重复的输入/周期/绑定检查、原角度死区提前返回，以及 DumpOutput 对可写配置的再次读取。机械限位、零积分增益免除法和保护状态判断保留其实际控制用途

分层依赖保持 module -> component/device，未增加 RTOS/HAL 对象或 ISR 共享状态；初始化结果由既有任务处理，故障输出统一由 DumpOutput 阻断

## Ozone 调参与后续上板

载入本分支生成的 `build/Debug/Infantry_Robot.elf`，通过 `gimbal_yaw_smc_config` 的 C、K、error_eps、q、p、u_max、J、epsilon 调参，其他字段不作为写入入口。观察 `gimbal_yaw_smc` 的 target、target_dot、target_ddot、error、s、u，以及原 `gimbal.yaw` 的 state、angle_deg、speed_rad_s 和 output_current_a

u 是保护前候选电流，实际导出还受 governor、机械边界及输出闸门影响。非正常状态下应观察 DumpOutput 导出的值，不能把残留的内部候选量解释为执行电流。多字段联动调整宜在 RELAX 完成；修正非法参数后仍须按原锁存故障流程重新使能

本轮未烧录、未驱动电机，参数尚未实机整定。下一步低限流确认极性和制动方向，再验证负载稳定性、机械边界、保护恢复、温升、周期耗时和任务栈高水位，最后与 PID 分支录波对照

## 临时验证资产清理

验证完成后清理本任务创建的 `build/smc_port_check.c/.exe`、集成测试源码及运行脚本、三个编辑辅助脚本、`build/smc_integration/` 替身与对象目录和根目录三个 `a-*.su` 临时文件；保留本文、验证 JSON 和正常固件构建产物，不删除正式测试资产
