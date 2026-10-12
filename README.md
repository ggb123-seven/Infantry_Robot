# Infantry_Robot

基于 DJI RoboMaster C 型开发板的步兵机器人控制工程，主控为 STM32F407IGH6，使用 STM32 HAL、
FreeRTOS 和 CMSIS-RTOS2。工程当前处于功能迭代阶段，底盘控制已初步完善，IMU 姿态反馈尚未接入
底盘或云台闭环。

## 当前功能

- DR16 遥控器数据接收与解析
- CAN 设备反馈采集和电流命令统一发送
- M3508 底盘电机控制、底盘运动学混控和 PID 调节
- GM6020 云台电机基础控制
- 设备故障检测和 FreeRTOS 任务集中创建
- BMI088 与 INS 姿态解算正在移植，尚未用于运动控制反馈

当前公开预发布版本为
[`v0.1.0-alpha.1 - 底盘初步完善`](https://github.com/ggb123-seven/Infantry_Robot/releases/tag/v0.1.0-alpha.1)。

## 工程结构

业务代码按以下方向依赖：

```text
User/task -> User/module -> User/component 或 User/device -> User/bsp -> STM32 HAL
```

主要目录用途：

| 目录 | 用途 |
| --- | --- |
| `Inc`、`Src` | STM32CubeMX 生成的启动代码、外设初始化和中断入口 |
| `User/bsp` | CAN、UART、GPIO、时间和内存等板级接口 |
| `User/device` | DR16、电机和 CAN 设备协议 |
| `User/component` | PID、滤波、混控、姿态算法和数学工具 |
| `User/module` | 底盘、云台和故障检测等业务模块 |
| `User/task` | FreeRTOS 周期调度、消息路由和模块编排 |
| `applications` | 现有指示灯任务 |

## 构建工程

构建环境需要：

- CMake 3.22 或更高版本
- Ninja
- GNU Arm Embedded Toolchain（`arm-none-eabi-gcc`）

工具链可通过系统 `PATH`、`ARM_GNU_TOOLCHAIN_ROOT` 或 `CUBE_BUNDLE_PATH` 提供。配置并构建 Debug
版本：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

构建产物位于 `build/Debug`。Release 构建使用：

```powershell
cmake --preset Release
cmake --build --preset Release
```

## CubeMX 配置

硬件配置保存在 `Infantry_Robot.ioc`。重新生成代码前应先确认本地 STM32CubeMX 版本及改动范围，生成后检查
外设配置、CMake 源文件列表和用户代码是否保持完整。

## 开发状态

该工程用于 RoboMaster 机器人开发和调试，当前版本仍包含未完成模块。上电测试前应架空底盘、限制输出，
并确认急停、失联保护和电机方向符合实际硬件。
