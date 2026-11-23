# 同步另一工作树的云台遥控映射（2026-11-23）

## 来源与范围

- 参考工作树：`F:\RM\Infantry_Robot`（`main` 分支）的 `User/module/cmd.c/.h` 与 `User/task/motor_gimbal.c/.h`
- 只同步云台部分，底盘、拨弹和功率限制的差异不引入
- 修改文件：`User/module/cmd.c`、`User/module/cmd.h`、`User/task/motor_gimbal.c`、`User/task/motor_gimbal.h`

## 同步的行为

| 项目 | 同步前（本工作区） | 同步后（与参考一致） |
| --- | --- | --- |
| 云台使能 | 右拨杆须先回中确认，再拨上才运行；中位/下位/非法档位都待机 | 遥控在线即进入 `ANGLE`，两侧拨杆不参与使能 |
| 使能状态 | `WAIT_NEUTRAL`、`READY`、`ACTIVE` 三态 | `INACTIVE`、`ACTIVE` 两态 |
| 环路选择 | 无遥控环路开关 | 新增 `Cmd_SelectGimbalLoops`：速度环关闭时整体待机，角度环关闭时降为速度模式 |
| 遥控速度 | 右摇杆左右 yaw、上下 pitch，满杆 6 rad/s，方向 -1，4 ms 平滑 | 不变 |
| 失联/故障 | 撤销运行并待机 | 不变 |

新增的遥控环路开关（默认都开启，行为与同步前默认一致）：

- `gimbal_remote_speed_loop_enabled`：关闭后遥控云台整体待机
- `gimbal_remote_angle_loop_enabled`：关闭后遥控角度模式降为速度模式

两个开关只改写遥控生成的角度模式，失联和故障产生的待机命令不受影响；调试命令入口优先于环路选择。

## 未同步的参考差异

- 拨弹命令解析、功率限制、`gimbal_debug_log` 等属于参考工作树的其他功能，本工作区不引入
- 参考树的 `gimbal_debug_enabled` 整体接管和高频调试记录不引入，本工作区继续使用既有的角度/速度调试入口
- 正弦测试仍保留在本工作区的云台任务内，不迁移到命令模块

## 验证

| 检查 | 命令或方法 | 结果 |
| --- | --- | --- |
| Debug 固件 | `py .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug -j 2` | 通过，ELF 1918680 字节 |
| ARM 严格语法 | 由 `compile_commands.json` 取 `cmd.c`、`motor_gimbal.c` 命令并加 `-Wall -Wextra -Werror -fsyntax-only` | 两个文件退出码 0 |
| 遥控映射主机测试 | 主机 GCC 链接实际 `cmd.c`，HAL 用反馈类型替身隔离 | PASS，0 失败 |
| 映射覆盖 | 右拨杆四个位置都进入 ANGLE；首次使能清平滑、第二周期起满杆约 ±6 rad/s；失联、旧帧、故障回待机；环路开关 ANGLE/SPEED/RELAX 选择；待机不被覆盖；复位清上下文 | 全部断言通过 |
| 格式扫描 | 同行花括号、`else` 同行、超过 120 列、行尾空白、注释句号 | 新增内容无违规 |
| 规范门禁 | `py .auto-embedded/scripts/check.py` | HW、SPEC 通过；ARCH 因本机执行策略无法加载子脚本 |
| 架构门禁 | `powershell -NoProfile -ExecutionPolicy Bypass -File .auto-embedded/scripts/arch-check.ps1` | 0 项违规 |

主机测试的临时源码、替身和可执行文件在记录结果后已清理。

## 验证边界

- 未烧录、未连接遥控器实测；摇杆方向、使能时机、模式切换和失联停机都需要上板确认
- yaw 电机反馈当前未接入（`gimbal.yaw.feedback_time_us` 为 `UINT64_MAX`），yaw 闭环测试仍需先恢复该路反馈
- pitch 机械角与配置的 ±60° 范围不一致，使能 pitch 前需要重新核对零点或范围

## 下一步

1. 烧录本轮固件
2. 遥控实测：右摇杆方向、首次上线捕获姿态、失联停机、故障恢复
3. 把 `gimbal_remote_angle_loop_enabled` 置 false 验证速度模式，再恢复为 true
4. yaw 反馈恢复后继续滑模整定
