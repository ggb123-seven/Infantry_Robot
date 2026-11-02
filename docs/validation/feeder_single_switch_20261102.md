# 拨弹模式开关合并为 feeder_fire_mode 一个接口

日期：2026-11-02；基线：`28db86d`；模块：`User/module/feeder.h`、`User/module/feeder.c`、`User/task/motor_trigger.h`

## 用户确认的语义

- 只保留一个全局开关 `feeder_fire_mode`，删除 `feeder_pid_mode` 与 `FEEDER_PID_MODE_*`
- `0` 连发：只走速度环，摇杆倍率直接映射输出轴转速，角度环不参与
- `1` 单发：右拨杆中位拨到下位触发一发，起发与执行走角度速度串级；档内推动摇杆按连发处理
- 环路结构与触发方式由模式唯一决定，不再存在“环结构开关与模式开关不一致”的组合

## 改动

- 删除全局 `feeder_pid_mode`、`FEEDER_PID_MODE_SPEED_ONLY`、`FEEDER_PID_MODE_CASCADE` 及声明
- `cascade` 只由 `feeder->single_active || single_start` 决定，其余情况全部走速度环
- 更新模式开关、文件头与拨弹调试入口的注释，说明连发只走速度环
- 未改控制算法、未改结构体字段、未改 `config.c`

## 验证证据

- ARM 严格检查：`feeder.c`、`motor_trigger.c` 追加 `-Wextra -Werror` 编译，均 0 警告
- 全量重编：35 个 `User/*` 编译单元重编，`OBJ_FAIL=0`
- 链接：手动链接 93 个目标文件 `LINK_EXIT=0`，产物 `build/Debug/Infantry_Robot.elf`（2026-11-02 17:29:24）
- 内存占用：RAM 47200 B（不变）、FLASH 127616 B（比上一版 127632 B 少 16 B，符合删除一个开关与一次比较）
- 残留检查：`User` 目录已无 `feeder_pid_mode` / `FEEDER_PID_MODE` 引用
- 排版与编码：改动文件无超 120 列、行尾空白、`} else`，UTF-8 有效

## 未验证项

- 与上一版相同：连发速度环爬升、方向符号、满杆 300 rpm 是否合适，需上板确认
- 单发档内推杆连续进给现在固定只走速度环，操作手手感未上板验证
