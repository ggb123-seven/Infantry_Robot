# 云台 yaw 速度环模块

用户确认使用集中全局结构体，速度单位为 rad/s，保留原字段名称。

- `User/task/motor_gimbal.c/.h` 提供任务空壳，未创建任务，也未调用控制模块
- `User/module/gimbal.c/.h` 复用现有 PID，提供初始化与单周期计算接口
- `struct gimbal_yaw_speed` 的类型别名为 `GimbalYawSpeed_t`，全局变量为 `gimbal_yaw_speed`
- `kp`、`ki`、`kd`、`target_rpm` 为可调输入，`actual_rpm` 由模块更新
- 两个带 `_rpm` 后缀的字段实际都使用 rad/s，驱动反馈输入仍使用 rpm，在模块入口换算
- 所有初值为零，模块输出转子侧电流 A，不访问 CAN，也未接入 pitch 控制

## 调用约定

`Gimbal_Init(sample_frequency_hz)` 初始化控制器，频率范围为 10 至 2000 Hz。
`Gimbal_YawSpeedRun(feedback_rpm, feedback_age_ms, enabled, period_s)` 返回单周期电流 A。
尚未收到反馈时，调用方必须将反馈年龄设为 `UINT32_MAX`；反馈年龄应由最新有效帧时间计算。
模块由单个任务串行调用，调试器只修改增益及目标值，禁止其他任务并发写入。

电流限幅为 ±1 A，目标角速度限幅为 320 rpm 对应的约 ±33.51 rad/s。
反馈超过 100 ms、参数非法、禁用或未初始化时返回零电流。
无效反馈在观察字段中标为 NaN，禁用时有效反馈仍可更新。
增益改变及停机后重新运行时清空控制历史，并用当前反馈预置微分。
PID 微分作用于反馈，改变目标不会直接产生微分冲击。

## 验证证据

- `py .auto-embedded/scripts/check.py`：ARCH、HW、SPEC 全部通过
- `cmake --build --preset Debug --parallel`：通过，RAM 42192 B，FLASH 102480 B
- 主机 GCC 对新增模块启用 `-Wall -Wextra -Werror`：通过
- 主机行为回归通过 10053 次断言，其中包含 10000 周期积分及限流检查
- 覆盖正反向响应、rpm 到 rad/s 转换、目标和电流限幅、增益在线生效、禁用后积分重置
- 覆盖微分预置、目标变化无微分冲击、反馈 100/101 ms 边界、无反馈、NaN、无穷值、非法周期和频率
- 临时测试源码、对象和可执行文件已由临时目录上下文清理
- `arm-none-eabi-nm -S` 确认 ELF 中存在 20 字节 RAM 符号 `gimbal_yaw_speed`
- Keil 已注册模块与任务源文件并配置符号保留，XML 解析通过，未执行 Keil 编译

本次使用 aemb-check 和 aemb-build-cmake 工具流程进行软件验证，未烧录、未进行电机实测。
CAN2 通信与反馈队列已在后续补齐，见下节；速度环模块仍未接入任务调度，尚不能直接在实机运行闭环。

## 后续补齐 CAN2 与两轴反馈

用户确认新增具名字段及两个独立邮箱，并明确允许手动同步 CubeMX 配置和初始化代码。

- `MOTOR_RM_MsgOutput_t.gm6020.yaw_id209` 与 `output[4]` 共用存储
- `MOTOR_RM_MsgOutput_t.gm6020.pitch_id20a` 与 `output[5]` 共用存储
- 联合体大小保持 16 字节，保留 M3508/M2006 原命名及槽位
- CAN2 使用 PB5 接收、PB6 发送，42 MHz APB1、分频 3、时间段 6/7，速率 1 Mbps
- CAN1/CAN2 共用时钟通过引用计数管理，滤波器组分别使用 0 和 14
- CAN2_RX0、CAN2_TX 中断优先级均为 5，发送完成回调继续处理软件队列
- `.ioc`、HAL 初始化、启动调用和硬件资源锁同步更新
- CAN 设备集合在 CAN2 注册 ID5 yaw 与 ID6 pitch，反馈 ID 分别为 0x209、0x20A
- CAN 任务每周期统一发送一帧 0x2FE，当前强制两轴电流为零，剩余字节也为零
- `task_runtime.msgq.gimbal_yaw_feedback` 和 `gimbal_pitch_feedback` 均为容量 1 的最新值邮箱
- 两个邮箱均复用 `MOTOR_RM_Feedback_t`，速度单位 rpm，`last_update_time` 为微秒时间戳
- 仅新鲜的完整标准数据帧进入邮箱，没有新帧时保留旧值，消费者必须按时间戳判断时效
- 所有邮箱在任务运行前创建并检查，失败时回收已创建的邮箱和任务
- 云台任务仍为空壳，未创建运行，未将非零速度环输出交给 CAN

补齐后的验证：

- ARCH/HW/SPEC 机械门禁通过
- Debug 构建通过，RAM 42256 B，FLASH 104212 B
- 使用真实 RM 驱动、CAN 设备集合及任务代码，配合模拟总线和 RTOS 的六种场景通过 298 项断言
- 覆盖反馈解析与轴间隔离、联合体别名、非零指令组帧、强制零电流、CAN1 帧保持、设备注册失败及发送失败
- 覆盖所有五个邮箱与四个任务创建失败的清理，以及邮箱覆盖、复位失败、写入失败及恢复
- BSP 初始化关键函数直接从当前源码提取，以 `-Wall -Wextra -Werror` 编译并通过六种正常或故障场景
- 覆盖双总线滤波器分配、回调分发、滤波器失败、启动失败、通知失败、发送队列失败、互斥锁失败与恢复
- ELF 中 CAN2_RX0_IRQHandler 和 CAN2_TX_IRQHandler 均为强符号，hcan2 与 gimbal_yaw_speed 均存在
- 临时测试源码及可执行文件已清理，未烧录、未进行电机实测

CubeMX 自动生成仍受第三方软件包索引 HTTP 418 阻塞，本次按用户明确授权手动同步。
本次构建与主机测试不证明 CubeMX 可重新生成，也不证明板上总线或电机反馈正确。
手册明确给出电流指令比例，反馈电流的实际标度仍需实测核对，当前不使用该字段闭环。

## 2026-10-21 速度环任务接入

- 增加 `Gimbal_YawSpeedRunAt`，由模块按 64 位微秒时刻检查反馈，拒绝未来时刻、无反馈和超过 100000 微秒的反馈
- 保留原有 `Gimbal_YawSpeedRun` 接口和全部调参字段，两个带 `_rpm` 后缀的字段仍实际使用 rad/s
- 主机 GCC 使用 `-Wall -Wextra -Werror` 编译真实模块、PID、滤波和数学组件，行为回归通过
- 覆盖一万周期积分限流、正反向输出、禁用、非法参数、微分预置，以及 100000/100001 微秒边界和超过 32 位回绕周期的过期反馈
- `aemb-build-cmake` 脚本使用 Debug 预设构建通过，产物为 `F:/RM/Infantry_Robot/build/Debug/Infantry_Robot.elf`，生成器为 Ninja
- `python .auto-embedded/scripts/check.py` 输出 ARCH/HW/SPEC 全部通过，未烧录或进行电机实测
- 用户要求简化命令，不增加命令结构体和时间戳；只扩展统一运行时中的任务句柄与命令邮箱句柄
- 将原来的 `CANDevices_KeepGimbalStopped` 直接改为 `CANDevices_ApplyGimbalCurrent(float yaw_current_a)`，复用同一缓存和发送路径，不保留重复的停机接口
- `Task_Init` 统一初始化云台模块、创建容量 1 的浮点电流邮箱和 500 Hz 云台任务，并检查全部返回值；失败时统一清理对象
- `Task_motor_gimbal` 消费已有 yaw 反馈邮箱，缓存最近反馈，调用模块计算电流，再向命令邮箱发布最新 float 值，单位 A
- `Task_can` 每周期消费一次云台电流命令，空邮箱或读取失败时使用零电流；pitch 始终为零，两轴每周期共用一次 0x2FE 发送
- 命令不携带时间戳，因此不提供按命令生成时刻判断过期的功能；停止生产后，已有的一条命令最多被消费一次，后续空邮箱周期归零
- PID 增益和目标初值均为零，当前通过调试器设置 `gimbal_yaw_speed.kp/ki/kd/target_rpm`；尚未接入遥控或 INS 角速度反馈
- 真实模块、任务、初始化与设备集合配合模拟 RTOS 和总线完成 372 项断言，覆盖全部六个邮箱和五个任务创建失败清理、正反向输出、无反馈、超时、空命令、发布失败、连续缓存和低位微秒计数回绕
- 真实 RM 驱动与设备集合完成 89 项断言，核对正反向 1 A 对应原始值 ±5461、协议量程限幅、pitch 与保留字节清零、单帧发送、非法值归零、发送失败及 CAN1 的 0x200 输出保持
- 最终 Debug 构建通过，RAM 42360 B，FLASH 106356 B；云台任务入口静态栈用量 144 B，配置栈为 1024 B，未做板上栈水位测量
- 本轮未修改 BSP、生成源码和 CubeMX 配置；本地提交同时收录已有且必需的 CAN2/GM6020 配套改动，排除工具变化及其他文档删除
- 临时测试源码与可执行文件在记录证据后清理，软件验证不代表已烧录或完成电机实测
