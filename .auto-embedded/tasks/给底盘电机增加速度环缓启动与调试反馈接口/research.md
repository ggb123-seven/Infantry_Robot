# 研究发现

| 关键词 | 来源 | 摘要 | 可信度 | 状态 |
|---|---|---|---|---|
| 主控与框架 | `Infantry_Robot.ioc`、`spec/hardware/index.md` | 主控为 STM32F407IGH6；工程使用 HAL、FreeRTOS、CMSIS-RTOS2，CAN1 配置为 1 Mbps。 | 高 | 已确认 |
| 硬件资源 | `spec/hardware/hw-lock.yaml` | 现有 CAN1 使用 PD0/PD1 和 CAN1_RX0_IRQn；本任务只增加任务层算法与封装，不新增引脚、DMA、中断或定时器，硬件锁无需变化。 | 高 | 已确认 |
| 当前任务周期 | `User/task/user_task.h`、`User/task/motor_chassis.c` | `MOTOR_CHASSIS_FREQ` 为 500 Hz，任务通过 `osDelayUntil` 定周期运行，对应名义控制周期 2 ms。 | 高 | 已确认 |
| 当前控制行为 | `User/task/motor_chassis.c`、`docs/workflows/2026-07/13_analysis_motor_control_can_init.md` | 当前每周期下发固定 0.8 A，未根据速度反馈计算控制量，因此不是速度闭环。 | 高 | 已确认 |
| 速度反馈单位 | `docs/datasheets/RoboMaster_C620_ESC_User_Manual_V1.01.pdf` 第 33 页、`User/device/motor_rm.c` | C620 原始速度字段单位为转子 rpm；`Motor_RM_Decode()` 在 `gear=true` 时除以减速比 `3591/187`，现有 `feedback.rotor_speed` 实际为减速箱输出轴 rpm。 | 高 | 已确认 |
| 电流指令单位 | `docs/datasheets/RoboMaster_C620_ESC_User_Manual_V1.01.pdf` 第 31-32 页、`User/device/motor_rm.c` | `MOTOR_RM_SetTorqueCurrent()` 接收转子侧电流 A，并按 `±20 A` 映射到原始值 `±16384`；它是电流指令换算与缓存接口，不是本任务的软件电流环。 | 高 | 已确认 |
| 速度 PID 库 | `User/component/pid.h`、`User/component/pid.c` | 现有库提供 `PID_Init()`、`PID_Calc()`、`PID_Reset()`；支持输出限幅、积分限幅及抗积分饱和，可直接复用。 | 高 | 已确认 |
| 安全边界 | `User/device/motor_rm.c`、`refs/robomaster-motor-can.md` | 设备层 100 ms 无反馈会标记离线；应用层仍需在离线或错误时显式设置并发送零电流，不能只停止更新目标。 | 高 | 已确认 |
| 额定速度 | `docs/datasheets/RoboMaster_M3508_User_Manual_V1.0.pdf` 第 12-13 页 | M3508+C620 输出轴额定转速 469 rpm、空载转速 482 rpm；目标速度限幅必须低于物理边界并保留为可调参数。 | 高 | 已确认 |
| 构建接入 | `CMakeLists.txt`、`MDK-ARM/Infantry_Robot.uvprojx` | CMake 和 Keil 都显式列出 `User/task` 源文件，新增 `.c` 必须同时加入两处才能保证两套构建一致。 | 高 | 已确认 |

## 用户确认的控制边界

- 本任务只有速度环，没有单独的软件电流环。
- 速度 PID 的输出量是给 3508/C620 的电流指令，单位 A。
- `motor_chassis` 负责封装现有设备层电流指令函数，但接口命名和注释不得把它描述为“电流环”。

## 待验证项

- 电机安装方向与速度反馈符号必须通过上板低速测试确认，不能由源码静态分析代替。
- PID 增益、目标速度限幅、电流指令限幅和缓启动斜率缺少实车对象数据，只能先提供保守默认值与集中配置入口，最终值需依据波形和实测整定。
