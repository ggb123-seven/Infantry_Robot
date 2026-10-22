# GM6020 偏航角度与速度串级接入

日期：2026-10-22

## 控制与调试接口

`gimbal_yaw_angle.target_deg` → 角度 PID → `gimbal_yaw_speed.target_rad_s` → 速度 PID → 电流邮箱 → CAN 任务

用户确认的类型为 `GimbalYawAngle_t`，全局变量为 `gimbal_yaw_angle`：

| 字段 | 用途 |
| --- | --- |
| `kp`、`ki`、`kd` | 角度外环比例、积分、反馈角速度增益，初值均为 0，供后续调参 |
| `target_deg` | 目标单圈角度，单位度，范围 [0, 360)，与编码器采用相同零点和方向 |
| `actual_deg` | 最新有效编码器单圈角度，单位度，反馈失效时为非数值 |
| `enabled` | 默认 true，置 false 时串级目标和电流归零，并清空两环历史 |

角度与角速度反馈当前分别取编码器单圈角度和转速；转速在模块内从 rpm 换算为 rad/s
角度外环使用 2π 环绕误差，输出 rad/s；速度内环输出转子侧电流 A
公开目标角度和实际角度使用度，编码器 rad 反馈在模块边界转换，进入 PID 前再转换为 rad
零增益初值不代表已调参，改变目标前须自行设置角度环增益；本轮未进行上板整定
独立的角度计算接口保留，后续仍可传入 INS 融合角度和陀螺仪角速度

## 分层与资源归属

- `User/task/motor_gimbal.c` 为 55 行，仅负责启动绑定、实际调度间隔和绝对节拍
- `Gimbal_BindQueues` 接收系统已创建的反馈邮箱与 float 电流邮箱句柄，不创建或销毁对象
- `Gimbal_Update` 完成最新反馈读取、缓存、完整时间戳恢复、两环计算和电流发布
- `Gimbal_YawRunAt` 完成两环串联；模块不引用 `task_runtime`，CAN 帧仍由通信任务统一发送
- 沿用已有反馈时效和控制周期约束，未新增保护策略或结构体字段
- 模块反馈缓存只由云台任务访问，全局调试量使用 volatile；未新增 ISR 共享状态或临界区

## 验证

- ARCH/HW/SPEC 全部通过，源码花括号、120 列、行尾空白和控制块检查通过
- 当前工作树 Debug 构建通过，RAM 为 42536 B，FLASH 为 107540 B
- 工作树与拟提交源码分别通过 87 项数值断言及 10000 周期串级仿真
- 覆盖角度正反跨零、rpm 换算、反馈微分符号、外环速度限幅、内环电流限幅
- 覆盖禁用/恢复、增益变更清历史、初始化失败、无反馈、未来时间、反馈超时及精确超时边界
- 覆盖模块邮箱收发、空邮箱缓存、低 32 位微秒计数回绕、长期失联及邮箱读取/重置/发布失败
- 仿真采用一阶电机模型，仅验证串级连接与符号，不代表真实 GM6020 动态或硬件整定结果
- 保留此前未提交的速度环默认增益、电流限幅与注释调整，本次提交只包含角度串级和任务职责收拢；拟提交版本按自身限幅重新执行主机回归
- 主机 GCC 两次均报告既有 `pid.c` 微分变量和 `user_math.c` 严格别名告警，未修改组件
- 临时测试采用模拟邮箱和时间服务，没有调用硬件；本轮未烧录

主机回归输出：

```text
PASS cascade and rpm conversion
PASS shortest angle across encoder zero
PASS angle derivative sign and units
PASS outer speed and inner current limits
PASS disable and reenable without old integration
PASS eleven stop paths and exact feedback age boundary
PASS module mailboxes, feedback caching, timestamp rollover and long dropout
PASS mailbox read, reset and publish failures reset both controllers
PASS runtime outer gain update resets old integration
PASS initialization preserves global request and failed initialization stops output
PASS simulated cascade convergence after 10000 cycles angle=1.00001132 speed=-0.00002970
PASS total numeric assertions=87
```

当前工作树固件：`build/Debug/Infantry_Robot.elf`
SHA256：`1e8d0fc49621b9c07d0f5bd3ddc38714ed0f776b9700469b8e2f4a0af0442848`
该固件包含工作树中单独保留的调参值

拟提交源码 SHA256：

| 文件 | SHA256 |
| --- | --- |
| `User/module/gimbal.c` | `d76297acb6a30b91ceb8fc110ae78513679d0d612ec413d99d1f5f440f2c974e` |
| `User/module/gimbal.h` | `dd014f207142913a8e1c59afa365cfbdb060bd26f7bdc6ba274a09727ba8597c` |
| `User/task/motor_gimbal.c` | `04da7155c8536738e2e85e0d4ac391aacf41b97fd96638edf62ea6d98e983862` |
| `User/task/motor_gimbal.h` | `c04f973c26ff520fbc86396d5abba0223c763fca139e697bcf9ed8ee86ebacab` |

测试源码、可执行文件、模拟头文件和本轮临时快照在证据记录后清理

## 角度接口单位调整（2026-10-22）

- `gimbal_yaw_angle.target_deg`、`actual_deg` 和 `Gimbal_YawAngleRun()` 的目标/反馈角统一使用度，单圈范围为 [0, 360)
- 编码器 `rotor_single_angle` 在模块边界由 rad 转为度，进入角度 PID 前再转为 rad；角度 PID 内部参数、2π 环绕与 `rad/s` 速度目标保持原单位
- `python .auto-embedded/scripts/check.py` 的 ARCH、HW、SPEC 全部通过，CMake Debug 增量构建通过
- 主机使用实际 `gimbal.c`、`pid.c`、`filter.c`、`user_math.c` 验证 90° 转换及 10°/350° 正反跨零最短角差，全部通过；临时测试文件已清理
- yaw 电流限幅为 0.5 A；速度上限仍为 320 rpm（约 33.51 rad/s），该值来自 GM6020 空载转速资料。电流限制扭矩输出，不能按比例推导速度上限
