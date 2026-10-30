# CAN2 拨弹与发射机构同步

## 参考与范围

- 参考 `F:/RM/未命名文件夹/User/device/can.h`：M3508 摩擦轮 ID5、ID6，M2006 拨弹 ID7
- 参考 `User/module/config.c`：发射机构使用 CAN2，拨弹减速比 36:1
- 当前工程云台 GM6020 反馈为 0x209、0x20A，电流帧为 0x2FE
- 发射机构反馈为 0x205、0x206、0x207，电流帧为 0x1FF
- 用户明确授权同步 device、module、task；不修改 CubeMX、BSP 或 INS

## 已确认的结构体方案

用户要求按已有底盘与云台风格设计，并已明确确认按双模块拆分继续

| 类型 | 字段与职责 |
| --- | --- |
| Trigger_Config_t / Shoot_Config_t | max_speed_rpm、max_current_a、speed_pid，分别使用拨弹输出轴 rpm 和摩擦轮转子 rpm |
| Trigger_Command_t | enabled、target_speed_rpm，连续速度命令 |
| Shoot_Command_t | enabled、target_speed_rpm[2]，两轮连续速度命令，符号独立 |
| Trigger_Output_t / Shoot_Output_t | current_a，分别为单值与两元素数组，单位 A |
| Trigger_t / Shoot_t | config、枚举 state、feedback、反馈有效标志及完整微秒源时间、目标转速、电流、PID、initialized；摩擦轮相关字段使用两元素数组 |
| Task_Runtime_t 扩展 | 两个任务句柄，三路独立反馈邮箱，以及两个独立命令邮箱、两个独立电流输出邮箱 |

四段接口为 Init、UpdateFeedback、Control、DumpOutput，与现有底盘接口一致

- 模块不读 RTOS 邮箱，不采时间；时间和反馈由任务显式传入
- 显式状态包含 RELAX、RUNNING、OFFLINE、FAULT，非法状态与无效命令均零电流
- 拨弹独立处理一只电机，摩擦轮模块处理一对电机，任一摩擦轮反馈失效时两轮归零
- 命令为最新状态，任务每周期消费新命令，缺命令立即失能；暂不接入遥控发射操作
- 默认禁用、零电流；PID 默认零增益，保留调试配置入口，避免把旧机构参数当成本机已调好参数
- Init 集中创建与失败回收任务和邮箱，CAN 任务独占 RM 输出缓存与组帧发送

与旧 Shoot_t 相比，此方案名称和现有项目一致、职责分开且便于扩展

旧 Shoot_t 同时保存三只电机的反馈、PID 和输出，并维护共享射击状态、热量、发射节奏与舱盖
旧代码可以拆分复用：摩擦轮部分归 Shoot，拨弹部分归 Trigger；不能原样复制整个旧结构作为两个独立模块

## 验证记录

- 修改前 `python .auto-embedded/scripts/check.py`：ARCH/HW/SPEC 全部通过

### 步骤 1：设备同步与协议缓存隔离

- 新增 CAN2 摩擦轮 0x205、0x206 与拨弹 0x207 注册，反馈接口复用 MOTOR_RM_Feedback_t
- 三路电流由 CANDevices_ApplyShootCurrent 合并后发送一次 0x1FF，未使用槽位保持零
- GM6020 独立缓存按 CAN 总线保存，原 M3508/M2006 缓存与现有公开结构体布局保留
- 主机命令 `python build/shoot_sync_device_test.py`：编译实际 motor_rm.c 与 can_devices.c，仅替换 BSP 传输、接收和分配，10000 组混合电流测试通过
- 回归覆盖交错两种发送顺序、部分槽位重写后整组重发、9 路设备注册与总线归属、M3508/M2006/GM6020 不同电流量程、保留字节为零、NaN 单路清零、发送失败后安全重试、非法反馈索引与空指针
- `python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug`：构建通过，产物为 build/Debug/Infantry_Robot.elf
- 从 compile_commands.json 取实际 ARM 编译命令，移除 -o/-c 后追加 -Wall -Wextra -Werror -fsyntax-only：两个修改的 C 文件通过
- `python .auto-embedded/scripts/check.py`：ARCH/HW/SPEC 全部通过
- Allman、120 列、行尾空白和 git diff --check 检查通过
- 人工核对：无新增底层到上层依赖，不修改 main、ISR、临界区、BSP 或 CubeMX；私有输出缓存由 CAN 通信链独占
- 本步骤未烧录、未测真实电机方向或总线时延；模块与任务接入仍待结构体方案确认
- 临时测试源码、可执行文件与运行脚本已清理，正式源码和固件产物保留

### 步骤 2：双业务模块

- 新增 Trigger 与 Shoot 独立实例、配置、命令和输出类型，均采用 Init/UpdateFeedback/Control/DumpOutput 四段接口
- 两个模块使用 RELAX/RUNNING/OFFLINE/FAULT 显式状态机，非法参数、非法状态或周期异常锁存故障，先失能后才能恢复；离线收到新反馈后可恢复
- Shoot 任一轮反馈失效时两轮撤驱，Trigger 独立；缓存反馈使用完整微秒源时间检查 100 ms，禁止低位回绕复活旧反馈
- 默认配置集中于 config.c，Trigger 为输出轴 300 rpm、2 A，Shoot 为转子 8000 rpm、每轮 3 A；两者 p/i/d 均为零，限幅不是实机整定结果
- CMake 与 Keil 均登记模块源码
- `python build/shoot_sync_modules.py`：直接编译实际模块、配置、PID、滤波和数学组件，默认零电流、正反转目标、限幅、在线 PID 变更、100000/100001 us 边界、单摩擦轮离线、自动恢复、32 位时间回绕、故障锁存与解除、非法配置和空指针检查通过
- Debug 构建、三个修改 C 文件的实际 ARM 严格检查、ARCH/HW/SPEC 与格式检查通过
- 模块不依赖 RTOS，不访问 HAL、总线或 task_runtime；尚未烧录或运行真实电机
