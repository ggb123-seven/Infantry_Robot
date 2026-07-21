
## #1  task=解决-cubemx-代码生成后的构建报错  phase=REVIEW
已将当前 M3508 固定 0.8 A 自转控制链路、0x200/0x201 CAN 收发路径及 BSP_CAN_Init 初始化职责整理到 docs/workflows/2026-07/13_analysis_motor_control_can_init.md；当前未修改控制逻辑，后续需补返回值检查、离线零输出和使能/急停状态。

## #2  task=解决-cubemx-代码生成后的构建报错  phase=REVIEW
用户明确新增长期约束：源码默认只允许修改 User/task；分析其他源码可以，若需修改 User/bsp、User/device、applications、Src、Inc、Drivers、Middlewares 或其他源码目录，必须先说明依赖并取得明确同意。该规则已写入根 AGENTS.md。

## #3  task=给底盘电机增加速度环缓启动与调试反馈接口  phase=REVIEW
完成单个 M3508/C620 的输出轴速度环：motor_chassis 作为主运行模块，拥有任务循环、RM 驱动编排、在线判断和电流指令下发；motor_speed_control 只负责这一台电机的目标限幅、150 rpm/s 缓启动、PID 计算、Ozone 调参和目标/真实速度反馈结构体，不包含底盘运动学或多电机同步控制。CMake Debug 构建、Keil XML、ELF 符号与 ARCH/HW/SPEC 门禁均通过。下一步需上板低速确认正负方向，并采集阶跃响应、超调、电流峰值和断线零输出后整定 PID。

## #4  task=can1四个3508电机驱动扩展  phase=REVIEW
已将 CAN1 上的 M3508 从单电机扩展为 C620 ID 1~4：每台独立反馈、目标转速、速度环和 Ozone 监视，四路电流缓存后统一发送 0x200。当前调试固件上电默认开启速度环，暂停 Ozone 前必须先关闭调试使能并等待停机就绪。Debug CMake 构建与 arch/hw/spec 门禁通过；尚未烧录实测，下一步需核对四个物理 ID、安装方向与反馈在线状态。

## #5  task=移植-dr16-遥控器驱动到当前工程  phase=REVIEW
CHASSIS-MERGE-CONTROL-05 已经用户确认并提交本地快照 ebcd75e；当前任务已进入 REVIEW，软件门禁通过：check.py、Debug 构建、DR16/UART/init/chassis CAN 主机回归通过。剩余缺口是 C 板 + DR16 板上实测，包括有效帧率、非法帧、重同步、溢出、UART 错误、最后更新时间、在线状态和任务栈水位。
