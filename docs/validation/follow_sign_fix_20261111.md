# 底盘跟随反馈符号修复与调试变量

日期：2026-11-11

用户已在 `User/module/chassis.c` 中去掉 follow PID 的角度反馈及速度反馈负号，本轮核对、验证并提交该改动
修复前现场证据见 [跟随晃动诊断](follow_oscillation_20261111.md) 及其 CSV/JSON，不能作为修复后的实车验收

## 当前控制约定

- `PID_Calc` 输入为 `sp=0`、`fb=relative_rad`、`fb_dot=chassis.follow_relative_speed_rad_s`
- `relative_rad=chassis.follow_relative_angle_deg * π/180`，角差来自 yaw 编码器反馈、已标定零点与方向配置
- 误差为环绕后的 `0-relative_rad`，目标是云台与车头相对角归零；不保存上一时刻相对角作为目标
- 组件 SET_D 输出为 `k*p*err-d*fb_dot` 加积分项；两处符号一起修正，避免后续启用 D 时方向不一致
- 当前实测执行方向下，正轮速使相对角增加，正角差应产生负转向请求；真实车头零点及动态收敛仍需实测
- 本次未修改共享 yaw reversed、平移坐标变换、配置参数、RAM 副本或持久周期快照

## 调试观测

| 用途 | 变量或常量 | 单位及含义 |
| --- | --- | --- |
| 跟随角度目标 | 常量 `0.0F` | 内部为 rad，画图可用 0° 参考线，无独立目标变量 |
| 跟随角度实际 | `chassis.follow_relative_angle_deg` | °，最短相对角 |
| 相对角速度反馈 | `chassis.follow_relative_speed_rad_s` | rad/s，作为 D 输入 |
| 跟随 PID 输出 | `chassis.follow_wz` | 归一化转向请求，不是实测角速度 |
| 轮速环目标 | `chassis.wheel[i].target_speed_rpm` | rpm，斜坡后的目标，i=0..3 |
| 轮速环实际 | `chassis.wheel[i].filtered_speed_rpm` | rpm，参与 PID 的滤波后反馈 |
| 轮速原始反馈 | `chassis.wheel[i].measured_speed_rpm` | rpm，排查滤波与反馈噪声 |
| 在线参数入口 | `chassis.config.follow_pid` | 控制周期复制、集中校验并应用 |
| 跟随生效条件 | `chassis.mode=2`、`chassis.config_valid=1` | 仍应结合反馈在线与轮状态观察 |

先观察零点附近正负小角差是否回到 0°，再看超调与振荡；角差请求平稳而轮速抖动时检查轮速内环，角差与请求反复换向时检查外环响应

## 验证证据与边界

- `python .auto-embedded/scripts/check.py`：ARCH/HW/SPEC 全部 PASS，执行策略仅对当前进程使用 Bypass
- `python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug`：配置与构建成功
- 按 `compile_commands.json` 的真实 ARM 编译参数，对 chassis.c 追加 `-Wextra -Werror -fsyntax-only`：PASS
- 本次构建使用当前工作树；其他未提交源码的既有改动不属于本次提交，也不据此声称干净提交的独立完整构建
- ELF：`build/Debug/Infantry_Robot.elf`，SHA256：`e6d167b9c2a20d138d4429a1fa9510cd0b528bd2e460949235c4d28104f22922`
- `.su`：Task_motor_chassis 144 B、Chassis_Control 208 B、Chassis_ControlWheel 40 B、Chassis_RunWheel 40 B、PID_Calc 72 B；任务栈配置 1024 B
- 符号修改未增加局部变量或调用；上述轮速路径所列静态帧合计 504 B，不含其他库调用和上下文，不代表最坏栈占用；暂无修复后运行时高水位，栈安全未完成实测
- 复核两行改动的 Allman、行宽与行尾空白，以及依赖方向、返回值路径、对象创建、ISR 共享状态和 DumpOutput；本次均未新增或改变相关机制
- 新增判空、isfinite、范围检查、保护性 return 均为 0，删除项为 0；未新增结构体、状态或临时测试程序
- 本轮未烧录、未写目标 RAM、未采集修复后实车响应；编译通过不代表实车晃动已消除
