# 电机输出轴角度环绕改单次取模

日期：2026-11-02；基线：`a7ad1a3`；模块：`User/device/motor_rm.c`

## 问题

`MOTOR_RM_WrapAnglePositive` 用两个 `while` 逐圈加减小圈，M2006 减速比 36 时输出轴一圈对应转子 36 圈，
输出轴 300 rpm 下转子约 180 圈/秒，`gearbox_total_angle` 随运行时间线性增长，单帧环绕循环次数也线性增长，
长时间运行后 500 Hz 反馈解析的 CPU 开销不可忽略

## 改动

- 改为一次 `fmodf(angle, M_2PI)` 取模，负值补一圈，返回值域仍为 `[0, 2π)`
- 显式包含 `<math.h>`，不再依赖 `component/user_math.h` 的传递包含
- 不改函数签名、不改 `MOTOR_RM_t` 字段、不改 `angle_valid` 语义；调用方（拨弹、云台）行为不变

## 验证证据

- ARM 严格检查：取 `build/Debug/compile_commands.json` 中 `motor_rm.c` 的真实编译命令，追加 `-Wextra -Werror` 后编译通过，0 警告（输出到临时目标文件，检查后删除）
- 目标文件：`motor_rm.c.obj` 由本轮源码重新生成
- 链接：本机 Ninja 子进程在沙箱内挂起（历史已知问题），改用工程同一条 gcc 命令手动链接 93 个目标文件，`LINK_EXIT=0`，产物 `build/Debug/Infantry_Robot.elf`（2026-11-02 15:48:22）
- 内存占用：RAM 47168 B / 128 KB（35.99%）、CCMRAM 0 B、FLASH 126296 B / 1 MB（12.04%）；FLASH 比上一版 126328 B 少 32 B，符合删除逐圈循环的预期
- 格式自查：新增注释为简体中文、无句号、无行尾空白、未超 120 列、花括号符合 Allman
- 分层门禁未运行（本机 PowerShell 执行策略拒绝未签名脚本，历史同因）；本次未改动 include 方向、文件规模与导出接口

## 新增检查自查

- 未新增判空、`isfinite`、范围检查或保护性 `return`；`fmodf` 对有限浮点输入与原有逐圈减法语义一致

## 未验证项

- 上板长时间运行后的单帧耗时与输出轴角度在跨零点附近的一致性
- 逐圈减法与取模在极端大角度下的数值差异未做主机对比测试
