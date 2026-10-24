# 底盘重构验证记录

日期：2026-10-24；依据根目录《底盘重构方案.md》执行

- 初始工作树已有云台 angle_pid.p=15、i=0.005 的修改、SMC 未跟踪目录及文档删除，本任务保留，不纳入提交
- 初始 `python .auto-embedded/scripts/check.py`：ARCH/HW/SPEC 全部通过

## 步骤 1：通用斜坡

- 新增 `RampTo`，原有数学函数保持原逻辑
- `python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug`：通过，产物 `build/Debug/Infantry_Robot.elf`
- 主机命令：`gcc -std=gnu11 -fgnu89-inline -IUser User/task/.chassis_refactor_test.c User/component/user_math.c -lm -o build/chassis_ramp_test.exe`
- 主机断言：上升、下降、不越过目标、零步长、已达目标、负步长和 NaN 步长共 7 项通过；临时测试文件和可执行文件已清理
- 新增代码人工核对：中文 Doxygen、Allman 花括号、120 列和无行尾空白；`git diff --check` 通过
- 验证边界：仅软件构建和主机运算，不代表上板验收

## 步骤 2～4：底盘、故障与任务接口整体迁移

模块公共类型替换同时影响 fault_detect、motor_chassis 和 init，故文档步骤 2～4 作为一个可编译实现单元验证并提交

- 底盘采用实例及 Init/UpdateFeedback/Control/DumpOutput 四段接口，使用模式和单轮状态机
- config.c 提供四轮公共默认配置，速度环统一 p=0.16，在线参数变更清空旧积分，微分截止频率变更同步更新系数
- 保留限幅、斜坡、换向保护、恢复预置、单轮隔离和两级滤波，非法命令或超过 100 ms 的周期零输出
- 故障检测改读实例，CAN 侧判定保持原逻辑
- 底盘在任务中初始化，按实测节拍差控制；初始化失败持续发布零电流与诊断，发布失败复位本地历史
- 保持 DR16 映射、100 rpm 尺度、660 通道限幅和 5% 死区
- CMake 已包含所需源码；Keil 补齐 9 个 User 文件及 RingBuffer 源码和包含路径，User 清单与 CMake 的 29 项一致

验证命令与结果：

1. `cmake --build --preset Debug --clean-first`：88/88 项通过，零 warning/error；RAM 42544 B、FLASH 110804 B；日志 `build/chassis_refactor_debug_build.log`
2. 主机控制与故障回归编译命令：

   ```powershell
   gcc -std=gnu11 -fgnu89-inline -O0 -g '-IUser/task/.chassis_refactor_stubs' '-IUser' User/task/.chassis_refactor_test.c User/module/chassis.c User/module/config.c User/module/fault_detect.c User/component/mixer.c User/component/pid.c User/component/filter.c User/component/user_math.c -lm -o build/chassis_refactor_test.exe
   ```

   输出 `PASS: chassis regression and fault flags (offline, recovery, init, config, CAN value)`

   覆盖文档 11 类用例，另覆盖空配置/实例、无反馈、NaN 运动量/尺度、零/NaN 周期、不兼容布局、非法轮状态、计算异常恢复和 sequence 保留；连续 1000 周期核对斜坡与电流限幅，换向保护连续 200 周期目标不反向

   故障覆盖健康、单轮离线及计数、待机控制故障屏蔽、配置非法、未初始化、空快照、仅 CAN 上报 VALUE；调参清除旧积分后本周期可正常产生新积分增量

3. RTOS 模拟：同上源码与参数，加 `-IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2` 编译 `.chassis_task_test.c`

   输出 `PASS: DR16 mapping, timeout boundaries, queues, task cycles, tick wrap, init failure, late cycle`

   覆盖上/中/下/非法拨杆、通道限幅与死区、100000/100001 us 边界、时间回退、缺失/离线输入、读队列失败、重置/发布失败、两周期任务循环、节拍回绕、200 tick 延迟、750/0 Hz 非法节拍

4. `gcc -std=gnu11 -Wall -Wextra -Werror -fsyntax-only`：使用上述包含路径检查 chassis/config/fault_detect/motor_chassis，零警告通过

   宿主 Clang 默认环境缺失 math.h，改用可用 GCC 完成严格检查，不将 Clang 记为通过

5. `python .auto-embedded/scripts/check.py`：ARCH/HW/SPEC 全部通过
6. 迁移的 10 个源码文件通过 Allman、控制块、120 列、行尾空白扫描；旧公共类型、调试变量与初始化接口无残留；人工核对中文阶段注释和 Doxygen
7. Keil XML 可解析、文件存在且 User 清单一致；本机未发现 UV4，未执行 Keil 编译

人工审核与验证边界：

- 任务只处理输入、消息和调度，模块不读队列与时钟，公共算法位于 component
- main.c、HAL、BSP、设备源码、ISR 和 CubeMX 配置未改；硬件锁门禁通过
- 没有新增 ISR 共享变量或临界区；底盘运行状态由底盘任务写入，调试器在 Debug 构建中修改公开配置
- 编译器单函数静态栈记录：Task_motor_chassis 为 80 B，Chassis_Init 为 72 B，不能代替 RTOS 栈高水位实测
- 未烧录或执行上板验收；四轮方向、失联停机时延、恢复出力、自旋方向与 p=0.16 响应须按方案第 9 节架空验证
- 临时主机测试源码、替身头文件和测试可执行文件已清理，正式测试资产未删除
