# M2006 拨弹速度环参数同步记录

日期：2026-11-01

## 用户提供的参数

来源：用户消息“2006电机速度环参数 p=0.5 i=0.12 d=0 同步一下信息”

| 在线调参字段 | 用户提供值 |
| --- | ---: |
| `feeder.config.speed_pid.p` | 0.5 |
| `feeder.config.speed_pid.i` | 0.12 |
| `feeder.config.speed_pid.d` | 0 |

本记录仅适用于 M2006 拨弹速度环，输入使用减速后输出轴 rpm，控制输出为转子电流 A
用户未提供其他参数变更，不据此变更 k、积分限幅、输出限幅、滤波、限速或前馈

## 与源码默认值的区别

- 当前 `Feeder_Init` 仍将 `chassis_config.wheel_speed_pid` 复制到 `feeder.config.speed_pid`
- 该源码默认值为 P=0.16、I=0、D=0，本次没有将用户参数固化到固件
- `trigger_config.speed_pid` 的源码默认值仍为 P=0、I=0、D=0，当前拨弹初始化不取这组增益
- 在线参数由 Control 入口复制到持久 `cycle_config.speed_pid` 后校验使用

## 验证与边界

- 同步前运行 `python .auto-embedded/scripts/check.py`，ARCH/HW/SPEC 全部 PASS
- 核对记录中的 P/I/D 与用户消息及在线入口一致；同步后复查文档差异与机械门禁
- 仅修改项目进度、调参记录及可复用记录约定，没有源码改动，无需重新编译
- 未读写目标 RAM、未烧录、未测量响应，不将用户提供值表述为本轮实测或已验证整定值
- 未新增或删除判空、`isfinite`、范围检查、保护性 `return`，无重复校验删除项
- 无硬件资源、分层依赖、ISR 共享、临界区或安全输出逻辑变更，无临时测试程序
