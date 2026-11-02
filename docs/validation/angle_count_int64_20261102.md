# 多圈角度计数扩宽为 int64

日期：2026-11-02；基线：`aa2911c`；模块：`User/device/motor_rm.h`

## 问题

`MOTOR_RM_t` 的 `gearbox_total_raw_count`、`gearbox_round_count` 为 `int32_t`。M2006 减速比 36、反馈 500 Hz，
输出轴 300 rpm 时转子约 1.47 M counts/s，累计约 24 分钟后达到 `INT32_MAX`，属于有符号溢出（未定义行为），
会让多圈角度与圈数跳变，且无法通过重启以外的方式恢复

## 改动

- `gearbox_total_raw_count`、`gearbox_round_count` 由 `int32_t` 改为 `int64_t`
- 补充字段上方集中说明：累计计数用 int64 以避免长时间运行后 int32 溢出
- 每帧增量 `delta` 仍为 `int32_t`（单帧最大半圈，8192 counts 内），累加与除法自动提升为 int64
- 未改字段顺序以外的语义、未改对外接口、未改 `MOTOR_RM_Feedback_t` 布局；`float gearbox_total_angle` 保持 float

## 验证证据

- 全量用户代码重编：`build/Debug/compile_commands.json` 中 35 个 `User/*` 编译单元全部重编，`OBJ_FAIL=0`
  （头文件字段布局变更，所有包含 `device/motor_rm.h` 的 TU 必须同时重编）
- 链接：本机 Ninja 子进程在沙箱内挂起（历史已知问题），改用工程同一条 gcc 命令手动链接 93 个目标文件，
  `LINK_EXIT=0`，产物 `build/Debug/Infantry_Robot.elf`（2026-11-02 15:56:16）
- 内存占用：RAM 47168 B / 128 KB（35.99%）、CCMRAM 0 B、FLASH 126408 B / 1 MB（12.06%）；
  静态 RAM 未变化，因为 `MOTOR_RM_t` 实例由 `BSP_Malloc` 在堆上分配，每个电机多占 16 B 堆空间
- 格式自查：头文件注释为简体中文、无句号、未超 120 列、无行尾空白
- 分层门禁：`arch-check.ps1` 以 `Invoke-Expression` 方式运行通过，0 违规

## 遗留限制

- `gearbox_total_angle`、`feedback.rotor_total_angle` 仍为 float；多圈累计很大时 float 尾数精度下降，
  单圈角在该量级下约 0.3~0.6 度误差。拨弹齿距 45 度，当前可接受；若要彻底消除需改为周期性折算或换 double
- 实际堆占用增量未在目标板测量

## 未验证项

- 上板长时间连续运行（> 24 分钟）不再出现角度与圈数跳变
- 单圈角在长时间运行后的实际误差与拨弹逐齿定位精度
