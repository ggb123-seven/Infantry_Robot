# 电机角度连续性改为逐帧判定并恢复拨弹相对参考

日期：2026-11-02；基线：`aa3ebf2`；模块：`User/device/motor_rm.c`、`User/device/motor.h`、`User/module/gimbal.c`

## 问题

`angle_valid` 原语义为“上电以来从未出现疑似跨零/丢帧”，由 `angle_lost_count == 0` 推导且该计数只增不减，
一旦出现一帧可疑增量，角度永久无效：

- 拨弹 `feedback_valid` 直接取该标志，进入 `FEEDER_OFFLINE` 后无法自动恢复，只能重启，现场已多次出现
- 云台两轴同样永久失效，必须重新对零或重启
- 实际触发场景只是 CAN 丢帧或反馈间隔变大导致单帧增量超过半圈阈值，并非编码器损坏

## 改动

- `motor_rm.c`：Decode 内单独记录本帧增量是否可信，`angle_valid = delta_plausible`；疑似丢帧只让本帧无效，
  下一帧增量合理即恢复，`angle_lost_count` 仍保留为累计诊断量
- `motor.h`：更新两个字段注释，明确“逐帧可信”与“累计次数供上层决定是否重新对零”
- `gimbal.c`：`frame_valid` 增加 `motor->angle_lost_count == 0U`，保持云台原有保守行为（出现丢帧后机械角可能整圈偏移，
  重新对零前不采信），本改动不改变云台对外行为
- 未改 `MOTOR_RM_t` / `MOTOR_RM_Feedback_t` 字段与结构布局，未改接口签名，未改 `config.c` 与 `.ioc`

## 行为变化

- 拨弹：可疑帧当周期进入 `FEEDER_OFFLINE` 并撤驱，下一帧恢复 `FEEDER_RUNNING`，按状态进入动作重新捕获当前齿位目标；
  恢复后的角度只保证从恢复点起算的相对连续性，多圈绝对值不再可信
- 云台：与改动前一致，累计丢失次数非零即保持轴故障；恢复需要重新对零或重启
- 其他消费方：`motor.c` 的 `MOTOR_IsAngleValid` 无调用点

## 验证证据

- ARM 严格检查：取 `build/Debug/compile_commands.json` 中两个文件的真实编译命令，追加 `-Wextra -Werror` 后编译通过，均 0 警告
- 目标文件重编：`motor_rm.c.obj`、`gimbal.c.obj` 由本轮源码重新生成，`OBJ_EXIT=0`
- 链接：本机 Ninja 子进程在沙箱内挂起（历史已知问题），改用工程同一条 gcc 命令手动链接 93 个目标文件，`LINK_EXIT=0`，
  产物 `build/Debug/Infantry_Robot.elf`（2026-11-02 15:51:10）
- 内存占用：RAM 47168 B / 128 KB（35.99%）、CCMRAM 0 B、FLASH 126336 B / 1 MB（12.05%）；比重构前 126328 B 多 40 B，
  与新增一个布尔判断及注释无代码影响一致
- 格式自查：三个修改文件无同行左花括号、`} else`、超过 120 列、行尾空白与注释末尾句号
- 分层门禁未运行（本机 PowerShell 执行策略拒绝未签名脚本）；本次未改动 include 方向、文件规模与导出接口

## 新增检查自查

- 未新增判空、`isfinite`、范围检查或保护性 `return`；`delta_plausible` 只在本帧内使用，恢复判定不引入额外分支

## 未验证项

- 上板验证：CAN 人为丢帧后拨弹是否按预期一周期撤驱、下一帧重捕齿位并继续射击
- 上板验证：连续多次丢帧时拨弹不会出现齿位累计错位（绝对角度已不可恢复，只能保证相对一致）
- 云台在丢帧后的对零流程未实测；本次只是保持原有“丢失即故障”行为
- `gearbox_total_raw_count` 仍为 int32，长时间连续运行约 24 分钟后溢出的风险本次未处理，需结构体字段变更后才能修
