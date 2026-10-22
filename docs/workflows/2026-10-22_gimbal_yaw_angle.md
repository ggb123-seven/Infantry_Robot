# GM6020 偏航角度外环预留接口

本次仅在 `User/module/gimbal.c`、`gimbal.h` 增加独立角度外环。当前任务未调用角度环接口，速度环目标和 CAN 输出路径没有接入角度环。

复用现有 `KPID_Params_t`，未新增或修改结构体定义。角度外环拥有独立 PID 历史和状态，由单个任务串行调用，参数对象须在整个使用期间有效，不能传入随后失效的局部对象地址。

## 接口与后续接入

| 接口 | 行为 |
| --- | --- |
| `Gimbal_YawAngleInit(sample_frequency_hz, &angle_params)` | 初始化独立外环，返回 0 表示成功，初始化后为禁用状态 |
| `Gimbal_YawAngleSetEnabled(enabled)` | 显式使能或禁用，状态切换清空历史，重复使能保留积分 |
| `Gimbal_YawAngleRun(target_rad, feedback_rad, feedback_rad_s, period_s)` | 计算并返回 rad/s 速度目标，未初始化或禁用时返回零 |

目标角度与反馈角度均为 rad，须使用相同零点和正方向；反馈角速度为 rad/s，与反馈角度正方向一致。控制周期为有限正数，不能小于初始化周期。使用 INS 的度制姿态时，接入方先转换为 rad。

`KPID_Params_t` 各参数的角度环含义：

| 字段 | 约定 |
| --- | --- |
| `k` | 通常为 1，使用并行 PID 增益 |
| `p`、`i`、`d` | 单位分别为 1/s、1/s²、无量纲，由调用方后续整定 |
| `i_limit` | 积分状态绝对值上限，单位 rad·s |
| `out_limit` | 速度目标绝对值上限，单位 rad/s，沿用 PID 组件配置语义 |
| `range` | 单圈角度取 `M_2PI`，目标与反馈须在同一周期区间；多圈连续角度取 0 |
| `d_cutoff_freq` | 外环选择 `KPID_MODE_SET_D`，微分直接使用外部角速度，不使用此参数过滤微分 |

初始化成功后显式使能，再周期调用 `Gimbal_YawAngleRun()`。后续由接入方决定是否把返回值写入 `gimbal_yaw_speed.target_rad_s`；本次未加入该赋值，也未绑定编码器、INS 或遥控器。

按本次需求，只提供使能入口，未新增反馈超时、参数范围巡检、故障锁定或自动恢复。沿用现有 PID 组件自身的计算规则。禁用仅令外环返回零，不会自行停用独立速度环；接入方负责速度环目标和使能的协调。

## 验证记录

- `python .auto-embedded/scripts/check.py`：ARCH、HW、SPEC 全部通过
- `python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug`：配置和构建成功，产物为 `build/Debug/Infantry_Robot.elf`
- 主机使用实际 `gimbal.c`、`pid.c`、`filter.c`、`user_math.c` 链接执行离线测试，28 条断言通过，覆盖初始化、默认禁用、正反误差、跨零点回绕、正负速度限幅、外部角速度微分、重复使能保留积分、启停清历史、多圈连续角度、重新初始化及初始化失败、与速度环隔离
- 主机命令：`gcc -std=c11 -O2 -fgnu89-inline -Wall -Wextra -ffunction-sections -fdata-sections -IUser -IUser/component User/task/.gimbal_angle_verify.c User/module/gimbal.c User/component/pid.c User/component/filter.c User/component/user_math.c '-Wl,--gc-sections' -lm -o build/Debug/gimbal_angle_verify.exe`
- 共用组件有既有告警：`pid.c` 的非法枚举路径微分变量可能未初始化，以及 `user_math.c` 快速平方根的严格别名告警；本次未修改这些组件
- `gimbal.c` 独立使用 `-Wall -Wextra -Werror` 编译通过
- C/H 文件格式扫描、差异空白检查通过，人工复核 Doxygen、单位、状态机、分层依赖及单任务所有权；新增部分没有 ISR 共享对象、HAL/RTOS 对象或临界区
- 临时测试源码、可执行文件和对象文件在记录结果后清理

本次为接口与软件计算验证，未烧录、未接入电机运行，也未进行角度环整定或上板验收。
