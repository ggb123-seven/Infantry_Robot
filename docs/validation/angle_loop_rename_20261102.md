# 拨弹“位置环”术语改名为“角度环”

日期：2026-11-02；基线：`d316069`；模块：`User/module/feeder.h`、`User/module/feeder.c`、
`User/task/motor_trigger.h`、`User/task/motor_trigger.c`

## 动机

该环的实际控制量是输出轴角度与环绕误差，输出为角速度目标，叫“位置环”容易与绝对位置混淆，统一改称“角度环/角度外环”

## 范围

- 只改注释与函数说明中的术语：`位置外环` 改为 `角度外环`、`位置环` 改为 `角度环`
- 替换数量：`feeder.h` 8 处、`feeder.c` 5 处、`motor_trigger.h` 4 处、`motor_trigger.c` 1 处
- 不改控制逻辑、不改结构体、不改字段名 `position_pid`、不改 `config.c`
- 未改 `config.c:8` 的既有注释（该文件参数冻结，需要单独授权才能改注释）

## 调试入口不变

EmberProbe 仍按 `feeder.config.position_pid` 调整角度外环参数（`k/p/i/d/i_limit/out_limit/d_cutoff_freq/range`），
本周期实际生效值为 `feeder.cycle_config.position_pid`，运行状态见 `feeder.position_pid.last` 与 `feeder.position_pid.i`

## 验证证据

- 严格检查：`feeder.c`、`motor_trigger.c` 用真实编译命令追加 `-Wextra -Werror` 编译，均 0 警告
- 全量重编：35 个 `User/*` 编译单元重编，`OBJ_FAIL=0`
- 链接：手动链接 93 个目标文件 `LINK_EXIT=0`；RAM 47200 B、FLASH 127256 B 与改名前一版完全一致，符合“纯注释改动”
- 残留检查：四个文件中已无 `位置环`/`位置外环` 字样
- 排版自查：四个文件无超过 120 列、无行尾空白
- 临时改名脚本已删除，未留在仓库

## 未验证项

- 纯注释改动，无功能影响，无需上板验证
