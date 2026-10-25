# 云台惯性反馈迁移记录

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
