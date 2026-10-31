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

原有 can_devices.c 的初始化阶段仍仅预置零值，待步骤 2 加入周期输出后启用拨弹控制帧
