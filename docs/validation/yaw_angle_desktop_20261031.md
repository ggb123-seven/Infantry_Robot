# 桌面云台 ANGLE 异常只读排查

## 现场信息与结论边界

- 用户报告 yaw 切入 ANGLE 后异常，二维云台放在桌面，底盘电机未连接
- 尚未取得具体异常表现、IMU 安装位置及本次板上状态，不能确认现场根因
- 本次未修改生产源码、配置参数、CubeMX 或板上状态，未烧录
- 初始工作树已有 `.auto-embedded/spec/conventions/index.md` 修改，本次保留

## 当前源码行为

- `User/module/gimbal.c:225`：正常从 RELAX 切入 ANGLE 时捕获当前 INS 姿态，并清零速度前馈
- `User/module/gimbal.c:263`：底盘四轮离线持续 1 秒后才启用带转检测；离线本身不禁止闭环
- `User/module/gimbal.c:280`：yaw 运行且编码器与惯性转速差绝对值大于 1 rad/s、惯性转速绝对值小于 0.3 rad/s，持续 0.15 秒后进入 TRIPPED
- `User/module/gimbal.c:294`：TRIPPED 的 yaw 输出为零，pitch 独立运行
- `User/module/gimbal.c:530`：TRIPPED 锁存至 RELAX，重新进入 ANGLE 时捕获当前姿态
- `User/module/gimbal.c:661`：角度环反馈来自 INS 累计 yaw；速度环使用欧拉 yaw 角速度，不使用电机编码器转速作为闭环反馈
- `User/module/config.c:175`：电机反馈超时为 90 ms，IMU 反馈超时为 10 ms；本次未改变

桌面放置不能证明底座固定。上部被约束、底座反转或 IMU 不随上部运动均需现场核实，不能仅凭软件判据断言物理悬空；正反馈方向、供电与任务异常同样未获实测排除

## 验证证据

1. `python .auto-embedded/scripts/check.py`：ARCH、HW、SPEC 全部 PASS
2. `python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source F:/RM/Infantry_Robot --preset Debug`：配置与增量构建通过，产物为 `build/Debug/Infantry_Robot.elf`
3. 主机 GCC 链接实际 `gimbal.c`、`pid.c`、`filter.c`、`user_math.c`，配置原样提取自当前 `config.c`；使用正式 `motor.h` 的反馈类型，仅用临时头文件隔离 HAL 包含依赖，反馈按 500 Hz 合成，不模拟真实电机动力学
4. GCC 参数包含 `-std=gnu11 -fgnu89-inline -O0 -g -Wall -Wextra`，编译退出码 0，无诊断；运行退出码 0，以下 6 项通过

| 用例 | 结果 |
| --- | --- |
| 离线待机超过 1 秒后切 ANGLE，姿态 23 度且速度为零 | 捕获当前姿态，yaw 电流为零，无目标跳变 |
| 离线静止持续运行 2 秒 | 保持 RUNNING |
| 离线时编码器与惯性转速均为 2 rad/s | 保持 RUNNING，不触发带转检测 |
| 离线确认后编码器 2 rad/s、IMU 为零 | 第 76 个周期即 0.152 秒进入 TRIPPED，yaw 零电流且 pitch RUNNING |
| TRIPPED 后反馈恢复静止 | 继续锁存 |
| RELAX 后再次 ANGLE | 解除锁存并捕获当前姿态 |

临时测试源码、替身头文件及可执行文件已由 TemporaryDirectory 清理；未创建新结构体或修改现有结构体

## 板上读取限制

`mcu-variables/scripts/read.js` 请求读取模式、轴状态、IMU 状态、离线计时、带转计时、目标角、惯性角和两类转速，两次均返回 `BRIDGE_UNAVAILABLE`；诊断为当前工作区找不到可用 Agent Bridge，不能据此认定探针断开或目标板故障，也不能将软件复现当作现场捕获

## 人工复核与后续定位

- 分层仍为 task 调度、module 控制、component PID；本次没有源码变动
- 未变更 main、RTOS/HAL 对象、ISR 共享状态或临界区
- 输出闸门仍是 Gimbal_DumpOutput，配置 RAM 副本、周期快照和 PID 持久参数指针保持原状
- 新增判空、isfinite、范围检查、保护性 return 及删除检查项均为零
- 需要现场核对 `gimbal.yaw.state`：4 表示带转撤驱，2 表示故障，3 表示电机反馈超时，1 表示闭环仍在运行
- 如仍为 RUNNING 而出现猛转或振荡，先在 RELAX、固定底座条件下核对上部转动时编码器与 IMU 角度及角速度方向，再决定是否需要修改控制方向；不盲改 PID 或删除保护
