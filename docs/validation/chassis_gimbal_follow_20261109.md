# 底盘跟随云台模式验证

日期：2026-11-09

## 实现范围

- 复用 `MOTOR_RM_Feedback_t` 新增 `chassis_yaw_feedback` 最新值邮箱，CAN 任务将 yaw 有效反馈同时发布给云台和底盘
- `Chassis_ResolveGimbalFollow` 在底盘模块内完成反馈时效、编码器角度和丢帧校验，按 yaw 相对零点计算最短角差
- 跟随模式将遥控平移从云台坐标旋转到底盘坐标，并使用比例增益 0.5、转向归一化限幅 ±0.3
- 左拨杆上位为待机，中位为底盘跟随云台，下位保持原车体坐标小陀螺；底盘原有独立模式和四轮速度闭环不变
- 跟随反馈超过 90 ms、角度无效、丢帧或邮箱无有效帧时，本周期转为 RELAX 并由既有 DumpOutput 输出零电流

## 验证证据

- 主机合成回归：`User/task/.chassis_follow_check/test.exe`，2549 项断言通过
- 覆盖：云台零点、±179° 角度、跨单圈边界、正反方向、平移向量长度保持、转向限幅、90 ms 与 90.001 ms 超时边界、32 位时间戳回绕、NULL 反馈、非法角度、丢帧、模式切换、非法命令和输出限幅
- Debug 构建：`python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug`，CMake 配置和 Ninja 构建成功，产物为 `build/Debug/Infantry_Robot.elf`
- 修改文件使用 Debug `compile_commands.json` 中的 ARM 编译命令追加 `-Wextra -Werror -fsyntax-only`，`chassis.c`、`cmd.c`、`init.c`、`can_task.c`、`motor_chassis.c` 全部通过
- `python .auto-embedded/scripts/check.py`：ARCH、HW、SPEC 全部 PASS；`git diff --check` 通过

## 实物边界

- 未烧录、未写目标 RAM、未在真实底盘上验证转向正负、零点偏差、跟随增益、机械响应和 90 ms 实际停机时延
- 0.5 增益和 ±0.3 限幅沿用参考工程与当前小陀螺幅度，仅是首版软件起点，需低速上板确认
- 本轮没有新增判空、`isfinite`、范围检查或保护性 `return`；新增反馈检查属于外部 CAN 输入信任边界，命令和周期检查仍由既有底盘模块入口集中完成

## 临时资产

验证完成后已删除 `User/task/.chassis_follow_check` 下的测试源码、替身头文件和可执行文件，未删除正式测试资产
