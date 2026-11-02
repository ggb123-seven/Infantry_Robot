# 拨弹单发模式（全局开关，EmberProbe 切换）

日期：2026-11-02；基线：`cdab958`；模块：`User/module/feeder.h`、`User/module/feeder.c`、`User/module/cmd.c`

## 用户确认的语义

- 新增全局变量 `volatile uint8_t feeder_fire_mode`，`0` 为自动连发（上电默认，行为与改造前一致），`1` 为单发，
  由用户在 EmberProbe 中直接写入切换
- 单发触发为右拨杆由非下位拨到下位的上升沿，保持下位不连发，再拨一次才发下一发
- 只在摇杆回中时接受单发触发；摇杆推着时仍按自动连续进给，并取消正在执行的那一发
- 位置外环、速度内环参数与限幅不变；`config.c` 未改动
- 摩擦轮链路本批不接（用户选择先不接），单发只完成拨弹定位

## 结构体变更（方案 A，用户已确认）

- `Feeder_Command_t` 增加 `bool trigger_level`：右拨杆下位电平，模块内部做上升沿
- `Feeder_t` 增加 `bool trigger_level_last`、`bool single_active`、`float single_elapsed_s`、
  `float single_in_position_rad`、`float single_timeout_s`
- 实测 RAM 增加 32 B、FLASH 增加 848 B

## 行为变更

- 单发模式下触发沿当周期把目标角推进 `config.direction × 2π / tooth_count` 并按当前实测角重捕起点，
  进入位置外环等待到位；到位（环绕误差不大于到位窗口）当周期回待机零输出
- 超过超时时间未到位进入 `FEEDER_FAULT`，沿用“只有待机档能解除故障”的既有语义
- 反馈失效、配置变化或任何非运行状态会终止当前单发，恢复后需要重新拨杆触发
- 自动模式路径保持不变：倍率大于零走原相位推进，倍率为零仍回待机撤驱，不引入位置锁定

## 参数默认值

- `single_in_position_rad` 默认 `0.06981317` rad（4°，约占 45° 齿距的 8.9%），可在 EmberProbe 在线调整
- `single_timeout_s` 默认 `0.3` s，可在 EmberProbe 在线调整
- 单发参数与配置快照分离，但同属在线调参入口，在 `Feeder_Control` 入口每周期校验一次，非法值按故障处理

## 验证证据

- ARM 严格检查：`feeder.c`、`cmd.c`、`motor_trigger.c` 用真实编译命令追加 `-Wextra -Werror` 编译，均 0 警告
- 全量重编：`build/Debug/compile_commands.json` 中 35 个 `User/*` 编译单元全部重编，`OBJ_FAIL=0`
- 链接：本机 Ninja 子进程在沙箱内挂起（历史已知环境问题），改用工程同一条 gcc 命令手动链接 93 个目标文件，
  `LINK_EXIT=0`，产物 `build/Debug/Infantry_Robot.elf`（2026-11-02 16:41:17）
- 内存占用：RAM 47200 B / 128 KB（36.01%）、CCMRAM 0 B、FLASH 127256 B / 1 MB（12.14%）
- 分层门禁：`arch-check.ps1` 以 `Invoke-Expression` 方式运行，`PASS: 0 violations`
- 排版自查：三个改动文件无同行花括号、`} else`、行尾空白、制表符和超过 120 列；新增注释为简体中文且无句号

## 新增检查自查

- `single_config_valid`：属于在线调参信任边界，在 `Feeder_Control` 入口只校验一次，用于拒绝 EmberProbe 写入的
  非有限值、非正值和超过一圈的到位窗口；自动模式下不参与判断
- `trigger_known`、`trigger_level`、`single_start`：命令语义与边沿判定分支，不是防御性兜底
- 未新增判空和 `isfinite`；`command == NULL` 的处理沿用改造前逻辑

## 未验证项

- 上板单发逐发定位成功率与重复精度；4° 到位窗口和 0.3 s 超时是否适合实车（可能需要按实际负载调整）
- 连拨防漏发：单发执行期间的触发被丢弃，属设计取舍，操作手快速连拨会漏发
- 反馈中断时正在执行的一发被终止，恢复后需要重新拨杆，是否可接受未上板确认
- 摩擦轮链路未接，弹丸实际发射与初速未验证
- 全局量在线切换与 500 Hz 控制周期的时序：每周期只读一次快照，未做双缓冲
