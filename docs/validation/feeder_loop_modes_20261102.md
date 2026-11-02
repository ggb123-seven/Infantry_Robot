# 拨弹环路结构拆分：连发仅速度环、单发角度速度串级

日期：2026-11-02；基线：`61c1a76`；模块：`User/module/feeder.h`、`User/module/feeder.c`、`User/task/motor_trigger.h`

## 用户确认的语义

- 新增全局开关 `volatile uint8_t feeder_pid_mode`：`0` 为仅速度环（上电默认），`1` 为角度速度串级，EmberProbe 直接写
- 连发（`feeder_fire_mode = 0`）：只走速度环，摇杆倍率直接映射输出轴转速目标
  `target_speed_rpm = direction × |进给倍率| × max_speed_rpm`，角度外环不参与、目标角不更新
- 单发（`feeder_fire_mode = 1`）：单发起发与执行强制串级，忽略 `feeder_pid_mode`
- 单发档内推动摇杆进入连续进给时，按 `feeder_pid_mode` 决定环结构，不强制串级

## 结构体变更

- 无。环结构用全局开关传递，速度反馈有效性用既有的非数值哨兵表示，未新增字段
- 未改 `config.c`、未改字段名 `position_pid`

## 行为变更

- 连发路径不再推进 `target_angle_rad`、不调用角度外环 `PID_Calc`，目标转速每周期由倍率给出
- 反馈判据按环路区分：串级要求角度连续且帧龄不超过 10 ms；仅速度环只要求帧新鲜且转速为数值
  - 角度丢帧时连发不再撤驱，`measured_angle_rad/deg` 为非数值，`measured_speed_rpm` 保持可用
  - 帧龄超过 10 ms 时转速同样为非数值，两种模式都进入 `FEEDER_OFFLINE`
- `feeder_pid_mode = 1` 时自动档恢复改造前的逐齿串级，可作为回退路径
- 单发逻辑不变：拨杆上升沿推进一个齿距、4° 到位窗口、0.3 s 超时

## 验证证据

- ARM 严格检查：`feeder.c`、`motor_trigger.c` 用真实编译命令追加 `-Wextra -Werror`，均 0 警告
- 全量重编：35 个 `User/*` 编译单元重编，`OBJ_FAIL=0`
- 链接：手动链接 93 个目标文件 `LINK_EXIT=0`，产物 `build/Debug/Infantry_Robot.elf`（2026-11-02 17:25:05）
- 内存占用：RAM 47200 B（36.01%，无新增字段故不变）、FLASH 127632 B（12.17%，比上一版 +376 B）
- 分层门禁：`arch-check.ps1` 以 `Invoke-Expression` 方式运行，`PASS: 0 violations`
- 编码检查：四个改动文件无替换字符，UTF-8 有效；排版扫描无超 120 列、行尾空白、`} else`

## 新增检查自查

- `feedback_drive_valid`：按环路结构选择驱动使能判据，属状态机条件，不是防御性兜底
- `isfinite(feeder->measured_speed_rpm)`：读取模块自己维护的非数值哨兵，写入点唯一在 `Feeder_UpdateFeedback`，
  属于模块内部边界，未在上层重复判空
- `single_config_valid` 保持原有语义，仅在单发模式参与判断

## 未验证项

- 上板连发速度环的爬升、超调和稳态误差；方向符号是否与 `direction` 一致
- 满杆 300 rpm 输出轴是否合适，是否需要单独连发上限
- 角度丢帧不撤驱在实车上是否安全（目前只影响定位模式）
- `feeder_pid_mode = 1` 的串级回退路径未实测
