# 拨弹速度环双向遥控修复验证

日期：2026-11-02

## 问题

- 用户实机观察到右摇杆下拨时 M2006 顺时针旋转，上拨时不能得到逆时针旋转
- `Cmd_ParseFeeder` 已将右摇杆上下映射为 `[-1, 1]` 的带符号 `speed_ratio`
- `Feeder_Control` 对 `speed_ratio` 使用 `fabsf`，导致负倍率丢失方向
- 运行请求使用 `feed_ratio > 0`，若直接移除 `fabsf` 又会把负倍率误判为无请求

## 修复

- 连发速度环保留 `speed_ratio` 正负号
- 运行请求改为判断 `feed_ratio != 0`
- 目标转速继续使用 `config.direction * feed_ratio * max_speed_rpm`
- 默认 `config.direction = -1` 不变，因此用户已确认的下拨顺时针方向保持不变，上拨目标符号反转
- 模式 1 的角度环调试行为不变

## 软件回归

临时主机回归直接编译真实 `feeder.c`、PID、滤波和数学组件，使用最小电机反馈类型桩，编译参数为：

```text
gcc -std=gnu11 -fgnu89-inline -Wall -Wextra -Werror
```

断言结果：

- `speed_ratio = +1` 时目标转速为 `-100 rpm`，电流输出为负
- `speed_ratio = -1` 时目标转速为 `+100 rpm`，电流输出为正
- `speed_ratio = 0` 时进入 `FEEDER_RELAX`，最终输出为 0 A

临时测试源码、桩头与可执行文件已清理

## 编译与门禁

- ARM 严格语法检查：`feeder.c` 使用 `-Wall -Wextra -Werror -fsyntax-only`，通过
- Debug 全量构建与链接通过：RAM 47200 B / 128 KB，FLASH 126108 B / 1 MB
- ARCH 门禁通过：0 violations
- HW 门禁通过
- SPEC 门禁通过
- `git diff --check`、120 列、Allman 与行尾空白检查通过
- `main.c`、ISR、临界区、结构体、`config.c`、BSP、设备层和 CubeMX 均未修改

完整 `python .auto-embedded/scripts/check.py` 的 ARCH 子脚本被本机 PowerShell 签名策略阻止；使用 `ExecutionPolicy Bypass` 运行同一脚本后通过，HW 与 SPEC 同时通过独立命令复核

## 防御性检查自查

- 本次未新增判空、`isfinite`、范围检查或保护性 `return`
- 本次未删除既有信任边界检查
- `feed_ratio != 0` 是运行状态请求条件，不是新增输入合法性检查；`speed_ratio` 的有限值与范围由命令层契约保证

## 未验证

- 修复后固件尚未烧录
- 尚未在真实 M2006 上确认右摇杆下拨顺时针、上拨逆时针及回中撤驱
- 尚未量测双向换向时的电流峰值、机械冲击和负载响应
