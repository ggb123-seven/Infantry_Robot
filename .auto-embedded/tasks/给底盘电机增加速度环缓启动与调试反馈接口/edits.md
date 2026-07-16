# 编辑清单

## 文件与职责

- [x] `User/task/motor_chassis.h`：只声明单个 M3508 电机主任务入口。
- [x] `User/task/motor_chassis.c`：作为主要运行文件，保留 `Task_motor_chassis()`、M3508/C620 参数、设备反馈更新、在线判断、电流指令下发和 CAN 控制帧发送；while 循环直接编排速度控制模块 API。
- [x] `User/task/motor_speed_control.h`：集中定义目标速度、速度限幅、缓启动斜率、PID 参数和电流指令限幅宏；定义包含目标速度与真实速度的主结构体和调试反馈结构体。
- [x] `User/task/motor_speed_control.c`：只实现单个 M3508 电机速度 PID 模块的初始化、反馈更新、控制计算和输出导出，不拥有任务入口、底盘运动学或 RM 设备驱动调用。
- [x] `CMakeLists.txt`：将 `User/task/motor_speed_control.c` 加入 CMake 目标源文件。
- [x] `MDK-ARM/Infantry_Robot.uvprojx`：将 `motor_speed_control.c` 加入 `User/MRobot` 分组。

## 默认控制参数

- 目标速度：`0 rpm`，作为当前宏定义接口，上电默认不转。
- 输出轴速度限幅：`±300 rpm`，低于 M3508+C620 的 469 rpm 额定输出轴转速。
- 缓启动斜率：`150 rpm/s`，在 500 Hz 任务中每周期最大变化 `0.3 rpm`。
- 速度 PI：`Kp=0.02 A/rpm`、`Ki=0.20 A/(rpm*s)`、`Kd=0`，复用项目现有 PID 库。
- 电流指令限幅：`±4 A`，低于 C620 的 `±20 A` 协议范围；最终参数需依据实车波形整定。

## 失效处理

- [x] 初始化失败：保持零电流，持续发布错误状态，不执行速度 PID。
- [x] 电机离线：每周期显式下发零电流，并重置 PID 积分和缓启动目标。
- [x] 目标、反馈或 PID 输出出现非有限值：按故障路径清零，不传播 `NaN/Inf`。
- [x] 在线恢复：缓启动目标从 `0 rpm` 重新爬升，避免使用断线前积分和目标。

## 验证

- [x] CMake Debug 构建成功且新增源文件进入编译命令。
- [x] Keil 工程 XML 能解析，新增源文件路径唯一且正确。
- [x] `python .auto-embedded/scripts/check.py` 全部通过。
- [x] 人工检查 `main.c` 未修改、目标速度与真实速度相邻保存在反馈结构体、Ozone 调参结构体位于 RAM、`motor_speed_control.c` 不直接调用 RM 设备驱动。
- [x] 记录无法在本机替代的上板验证项：正负方向、缓启动斜率、稳态误差、超调、电流峰值、掉线零输出。

## REVIEW 证据

- `cmake --preset Debug`：配置成功，生成目录为 `build/Debug`。
- `cmake --build --preset Debug --parallel`：编译和链接成功；最终占用 `FLASH 59408 B`、`RAM 39200 B`。
- `compile_commands.json`：包含 `User/task/motor_speed_control.c`，编译参数启用 `-Wall`。
- 最终代码包含 `Task_motor_chassis`、`g_motor_speed_pid_tune`、`MotorSpeedControl_Control` 和对现有 `PID_Calc` 库函数的调用。
- Keil `.uvprojx` 通过 PowerShell XML 解析，且新增源文件仅登记一次。
- `python .auto-embedded/scripts/check.py`：`ARCH`、`HW`、`SPEC` 均为 `PASS`。
- `git diff --numstat -- Src/main.c User/bsp User/device applications Src Inc Drivers Middlewares Infantry_Robot.ioc` 无输出，受限源码目录和 `.ioc` 未修改。
- 当前环境未连接目标板，未执行烧录、电机方向确认、阶跃响应、堵转或 CAN 断线实测；这些项目不能由主机编译替代。

| 文件 | 改动 | 验证标准 | 结果 | commit |
|---|---|---|---|---|
