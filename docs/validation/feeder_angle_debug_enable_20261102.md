# 拨弹角度环手动目标调试验证

日期：2026-11-02

## 当前约定

- `feeder_fire_mode = 0` 保持连发速度环，右摇杆倍率映射输出轴目标转速
- `feeder_fire_mode = 1` 暂时只表示角度环调试模式，不代表已确定的单发策略
- 模式 1 只有在右拨杆下位时使能角度与速度串级，右摇杆输入不参与控制
- 进入模式 1、右拨杆下位或反馈恢复时先捕获当前输出轴角度，不自动推进 45°
- 使能后通过 EmberProbe 写 `feeder_angle_target_deg`，单位度，控制周期入口只读取一次
- 右拨杆离开下位后进入待机并由 `Feeder_DumpOutput` 输出 0 A

本约定替代 `feeder_single_joystick_isolation_20261102.md` 中模式 1 自动执行 45° 单发的行为，最终单发策略留待后续确定

## 软件回归

临时主机回归直接编译真实 `feeder.c`、PID、滤波和数学组件，使用最小电机反馈类型桩，编译参数为：

```text
gcc -std=gnu11 -fgnu89-inline -Wall -Wextra -Werror
```

断言结果：

- 只写模式 1、右拨杆不在下位时保持 `FEEDER_RELAX`，输出 0 A
- 右拨杆首次下位时将 90° 实测角捕获到 `feeder_angle_target_deg`，目标与反馈相同，输出接近 0 A
- 写入 135° 后目标角同步为 135° 并产生角度环输出，右摇杆负倍率不改变控制路径
- 反馈持续离线时保持 `FEEDER_OFFLINE` 且不重复重置 PID，输出 0 A
- 反馈在 100° 恢复时重新捕获 100°，恢复首周期输出接近 0 A
- 右拨杆离开下位后回到 `FEEDER_RELAX`，输出 0 A

临时测试源码、桩头与可执行文件已清理

## 编译与门禁

- ARM 严格语法检查：`feeder.c`、`cmd.c`、`motor_trigger.c` 使用 `-Wall -Wextra -Werror -fsyntax-only`，通过
- Debug 全量构建与链接通过：RAM 47200 B / 128 KB，FLASH 126120 B / 1 MB
- ARCH 门禁通过：0 violations
- HW 门禁通过
- SPEC 门禁通过
- `git diff --check`、120 列、Allman 与行尾空白检查通过
- `main.c`、ISR、临界区、BSP、设备层、CubeMX 和 `config.c` 均未修改

完整 `python .auto-embedded/scripts/check.py` 首次执行时，ARCH 子脚本被本机 PowerShell 签名策略阻止；随后使用 `ExecutionPolicy Bypass` 运行同一 ARCH 脚本并通过，HW 与 SPEC 由原命令及独立命令复核通过

## 防御性检查自查

新增检查：

- `fire_mode_valid`：EmberProbe 在线写入边界的枚举范围检查，上游未检查同一条件
- `angle_target_valid`：角度环已使能后对 EmberProbe 目标快照执行有限值检查，上游未检查同一条件

删除检查：

- 删除 `single_in_position_rad` 的有限值与范围检查，因为当前模式不再执行单发到位判定
- 删除 `single_timeout_s` 的有限值与正值检查，因为当前模式不再执行单发超时判定

本次未新增判空或保护性 `return`，未重复下游已有检查

## 未验证

- 未烧录固件
- 未使用真实 M2006 验证进入角度环时的静止效果、目标阶跃响应、方向、电流峰值或机械负载行为
- 最终单发触发、45° 步距、到位窗口、超时与连续发射策略尚未确定
