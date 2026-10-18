# GM6020 代码移除验证（2026-10-18）

## 授权与范围

用户确认删除 GM6020 专用任务、云台模块及共用设备驱动中的 GM6020 分支，结构体只删除对应专属字段，不新增运行时结构体

- 删除 User/task/motor_gimbal.c、motor_gimbal.h、User/module/gimbal.c、gimbal.h
- 清理任务创建与失败释放、运行时句柄、独立命令和反馈邮箱、CAN 周期调用、Ozone 调参和监视入口
- 清理 CANDevices 的 GM6020 注册、反馈、快照和电流提交接口
- RM 共用驱动仅保留 M3508/M2006，槽位由 11 缩至 8，保留反馈 ID 0x201~0x208 和控制帧 0x200、0x1FF
- 同步 CMake 源码列表与 agent.md 当前 CAN 调度说明
- 保留 PDF 手册、历史日志与采样数据，不修改 INS、BMI088、BSP 时基、通用组件或 .ioc

## 编译与静态验证

命令：

~~~text
python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source F:/RM/Infantry_Robot --preset Debug --clean
python .auto-embedded/scripts/check.py
git diff --check
rg -n -i 'gm6020|gimbal' User CMakeLists.txt cmake MDK-ARM .vscode
~~~

结果：Debug 全量构建成功，Ninja，构建阶段耗时 3.9 秒；ARCH/HW/SPEC 全部 PASS；差异空白检查通过；源码及构建配置无 GM6020/Gimbal 引用

产物：F:/RM/Infantry_Robot/build/Debug/Infantry_Robot.elf

ELF SHA256：DD15BB99D9C60CBF6D939619239A46A25C26E391025492887A5C0420B45D78F1

使用工具链 arm-none-eabi-nm 检查：GM6020/Gimbal 符号为 0，INS_Task、Task_can、Task_motor_chassis、底盘 Ozone 全局监视及 MOTOR_RM_SetTorqueCurrent 符号仍存在

对全部修改的存续 C/H 文件扫描：无同行左花括号、行尾空白和超过 120 列；补齐缺少的控制块花括号并收回可单行短赋值；motor_rm.c 为 799 行

## 主机回归

临时测试直接组合修改前和修改后的真实设备源码与头文件，仅替换 BSP CAN、内存分配和时间接口，不重新实现被测算法

编译参数：gcc -std=c11 -Wall -Wextra -Werror

两版分别输出 AGGREGATION SAFETY PASS 和 REGRESSION PASS，保留电机的输出记录完全一致，包括：

- M2006/M3508 两个型号、八个 ID、正反向、带减速与直驱模式
- 四帧 0x200/0x1FF 组帧字节和 128 组反馈角度、速度、电流标度及丢角计数
- 四路底盘有效电流统一组帧、空命令归零、NaN 归零、离线归零
- 单路注册失败隔离、发送错误返回、CAN 初始化失败禁止注册和输出
- 修改后只注册四路底盘，四路反馈正常即可取得聚合成功，不再受已移除电机影响
- 修改后 0x209~0x20B 及第三控制帧组返回错误，索引 8 不提供调试槽位

该测试证明本次删除未改变保留型号的既有反馈标度，不代表这些标度已经通过物理单位或实车标定验收

## 其他边界

Keil 工程 XML 可解析且无 GM6020 引用，但其 HEAD 中原有的 Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F/port.c 当前缺失；本次不修改第三方代码或 Keil 工具链配置，不宣称 Keil 构建通过

本轮未提交、未推送、未烧录、未操作目标 MCU；删除文件已受 Git 跟踪，可以从历史提交恢复；不能据本轮删除认定 Yaw 漂移已解决

资料核对：GM6020 PDF 手册仍存在，本轮未修改或删除任何采样数据；用户早先提及的 data/emberprobe-live-20261018-143707.csv 当前路径不存在，不能宣称已找到或验证该采样文件

临时主机测试源文件及可执行文件在记录验证证据后清理，正式工程测试资产保持不变
