# 云台惯性反馈迁移记录

当前行为以末尾“移除静止使能门槛”记录为准，步骤二的连续静止检查属于已被替代的历史实现

用户已确认：沿用当前 INS 朝向，新增反馈结构仅包含姿态角、三轴陀螺仪角速度、源采样时间戳及有效标志
保留电机反馈结构、现有 CAN 电流输出和编码器机械限位

## 步骤一：同帧快照与邮箱

- `Gimbal_ImuFeedback_t` 包含 `angle_deg[3]`（Roll、Pitch、累计 Yaw）、`gyro_rad_s[3]`（X、Y、Z）、`sample_time_us` 和 `valid`
- INS 任务在完整更新后一次发布，时间戳在传感器读取前记录；云台只读邮箱，不裸读全局 INS
- 初始化先发布失效通知；采集失败尽力发布失效通知；发布失败关闭加热器并退出，消费者通过源时间识别停更
- `Task_Init` 在启动任务前创建邮箱并检查，任何后续创建失败均纳入统一回收
- 本步骤云台只缓存惯性快照供观测，尚未切换编码器闭环

验证命令及结果：

- `python .auto-embedded/scripts/check.py`：ARCH、HW、SPEC 全通过
- `python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug -j 2`：成功，产物 `build/Debug/Infantry_Robot.elf`
- `gcc -std=c11 -Wall -Wextra -Werror -O0 -Ibuild/imu_migration_test -IUser build/imu_migration_test/transport_test.c -o build/imu_migration_test/transport_test.exe`，执行测试通过
- 主机测试编入真实 `init.c` 与 `ins_task.c`，模拟 RTOS 和 INS 设备：逐一注入 7 个队列及 5 个线程创建失败，验证已创建对象全部回收；正常创建和回收通过
- 验证同帧三轴姿态/角速度、源采集时刻而非发布时刻、初始化失败、采集失败、队列重置/写入失败及非法节拍频率
- 按文件重新读取根协作规则并检查花括号、120 列、行尾空白、注释和依赖；`git diff --check` 通过
- 编译器静态栈报告：`INS_Task` 72 B、快照发布 72 B；不是实际线程栈高水位

验证边界：本轮未烧录，尚未测量实际姿态收敛、安装方向、电机响应、失联停机时延及栈高水位

## 步骤二：惯性串级闭环与机械保护

- `Gimbal_UpdateFeedback` 接收电机及惯性快照，module 层完成时效、数值、连续性及启动静止检查
- yaw 使用累计 Yaw/Gyro Z，pitch 使用 Pitch/Gyro Y，沿用当前 INS 朝向
- `angle_deg`、`speed_rad_s` 改为惯性反馈，新增 `mechanical_angle_deg`、`motor_speed_rad_s` 独立保留机械观测
- 共用惯性故障两轴停机并锁存，回 RELAX 后重新静止检查；单电机异常仅隔离本轴，不回退编码器闭环
- `Gimbal_Control` 命令参数改为输入输出，从待机或速度模式进入 ANGLE 时预置当前姿态并清零前馈
- pitch 按预计机械角约束目标增量，速度模式、前馈和边界电流均受保护，保留向内恢复和积分保护
- pitch 零点 118.08016°、机械范围 ±60°，yaw 机械限位默认关闭；正弦测试中心改为启动时当前 yaw
- 初始化保存配置后清零实例，避免整实例临时副本占满任务栈，并支持传入实例自身配置重新初始化
- 保留用户已有未提交的 yaw.reversed=true、yaw 角度环 P=15/I=0.005，只选择性提交本步骤配置差异

主机验证：

- 真实 `gimbal.c` 联合 PID、滤波、数学和编码器展开组件：25345 次断言通过，包含启动阶段逐周期零输出检查
- 覆盖轴向、单位、惯性与机械速度分离、负 pitch、累计 yaw 跨零、10000/10001 us 时效边界、重复/倒退/未来时刻、非有限数据、断流与姿态跳变、RELAX 解锁
- 覆盖底座倾斜下的预计机械目标、两侧限位、速度模式、前馈、向外电流阻断、向内恢复、积分保护及电流限幅
- 覆盖单电机超时、共用源故障、运行中零点/方向修改、非法状态/配置/周期、自身配置重新初始化与低 32 位时间戳回绕
- 真实 `motor_gimbal.c` 联合模块模拟 RTOS：6 个场景共 41706 次断言通过，包括闭环入口、IMU 停更、输出 reset/put 失败、缺失 IMU 邮箱、非法节拍及当前姿态中心正弦
- 步骤一 transport 测试重新编译执行通过，确认创建、清理、发布及退出路径

控制测试命令；将 control_test 替换为 task_test 可执行任务联合测试：

```powershell
gcc -std=c11 -Wall -Wextra -Werror -O0 -ffunction-sections -fdata-sections -Ibuild/imu_migration_test -IUser build/imu_migration_test/control_test.c User/module/gimbal.c User/component/angle_unwrap.c User/component/pid.c User/component/filter.c User/component/user_math.c '-Wl,--gc-sections' -lm -o build/imu_migration_test/control_test.exe
./build/imu_migration_test/control_test.exe
```

## 实机检查入口与边界

1. 载入本轮 ELF，保持 `gimbal_command.mode=GIMBAL_MODE_RELAX`，先观察 `gimbal.imu` 和两轴机械角
2. `gimbal.imu_state`：0 等待、2 就绪、3 故障；有效首帧直接就绪，不再使用状态值 1
3. 手动抬头核对 Pitch/Gyro Y 与机械角正方向，转 yaw 核对累计 Yaw/Gyro Z 与电机方向，源码定义不能替代实测
4. pitch 水平时机械角核对为 0°，上下行程检查 ±60°；yaw 改过 reversed 后重新核对机械零点
5. 就绪后先小电流核对内环方向；进入 ANGLE 后先捕获当前目标，下一周期后再修改目标
6. 故障后先回 RELAX，再等待有效新帧后使能

尚未烧录或运行真实电机；10 ms 超时须通过板上观测核定，云台的 1 秒静止检查及其变化率门槛已删除
直接陀螺仪轴分量不等同于任意姿态下的欧拉角导数，六轴 yaw 漂移不会因反馈迁移消失
预计机械角是局部近似，不包含制动距离，不能阻止惯性、外力或掉电越界
未加入重启代号，依赖初始化失效通知、校准断流与姿态连续性检查，不支持带电直接修改姿态内部状态

## 最终复核记录

- CMake Debug 构建成功，产物 `build/Debug/Infantry_Robot.elf`；主机测试使用 `-Wall -Wextra -Werror`
- ARCH、HW、SPEC 全通过；9 个相关源码文件的 Allman、120 列、行尾空白及控制块花括号扫描通过
- 人工复核任务/模块边界、同帧所有权、RTOS 创建与失败清理、故障状态默认零输出和注释；未更改 main、ISR、HAL、BSP、设备协议、CAN ID 或 CubeMX
- 编译器静态栈：Gimbal_Init 208 B，Task_motor_gimbal 216 B，Gimbal_UpdateImu 120 B；初始化整实例临时副本修复前为 968 B
- 静态栈报告不是运行时高水位，仍需板上观测中断嵌套及 RTOS 开销
- 设计决策通过 `task.py promote` 回流 architecture/conventions，明确惯性失效不回退编码器及机械目标增量约束
- 测试证据记录后删除本轮专用 `build/imu_migration_test` 下的临时测试源码、模拟头、检查脚本及可执行文件，不删除仓库正式测试资产
- 第一步本地提交 `e96a24e`；第二步通过验证后独立提交，未推送远端

## 移除静止使能门槛

- 用户要求移除云台额外增加的静止检查，删除 1 秒计时、低角速度及低姿态变化率门槛和对应运行字段
- `Gimbal_UpdateImu` 收到有效首帧立即从 WAITING 进入 READY，保留 READY=2、FAULT=3 的调试数值
- 故障依旧锁存至 RELAX，解除后等待有效新帧即可就绪；不再重新等待静止
- 保留超时、无效标志、NaN/Inf、时间倒退/重复、姿态异常跳变、单轴电机在线和机械限位保护
- 本次只删除云台使能条件，BMI088 驱动初始化中的原有零偏校准不在本次修改范围

验证证据：

- `python .auto-embedded/scripts/check.py`：ARCH、HW、SPEC 通过
- `python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug -j 2`：成功，生成 `build/Debug/Infantry_Robot.elf`
- 真实 Gimbal/PID/滤波/数学/编码器展开组件的主机回归：140 次断言通过
- 覆盖非零角速度首帧立即就绪、运动中的后续帧、角度模式入口捕获及下一周期目标修改产生电流、源时间 10000/10001 us 边界
- 覆盖启动无效帧、NaN/Inf、过期/未来/重复/倒退时刻、异常姿态跳变、故障锁存及 RELAX 后首帧恢复、非法状态、单电机故障隔离及机械边界保护
- 编译使用 `gcc -std=c11 -Wall -Wextra -Werror -O0 -ffunction-sections -fdata-sections -Ibuild/gimbal_ready_test -IUser build/gimbal_ready_test/ready_test.c User/module/gimbal.c User/component/angle_unwrap.c User/component/pid.c User/component/filter.c User/component/user_math.c '-Wl,--gc-sections' -lm -o build/gimbal_ready_test/ready_test.exe`，执行该程序通过
- 逐文件重读协作规则并复核格式、中文注释、分层和默认输出；测试记录后清理本轮临时测试文件
- 尚未烧录或进行硬件验证，不能据此确认此前无法控制的唯一原因
