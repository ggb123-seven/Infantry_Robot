# yaw 电机 CAN 总线注册修正（2026-11-23）

## 现象

- 云台遥控映射对齐后，pitch 电机反馈正常，yaw 一直 `feedback_time_us = UINT64_MAX`、`encoder_deg = NaN`、轴状态 RELAX
- 用户指出另一工作树 `F:\RM\Infantry_Robot` 的固件烧录后能正常读到 yaw 反馈，说明硬件和接线没问题

## 根因

对比两个工作树的 `User/device/can_devices.c`：

| 轴 | 参考工作树 `main` | 本工作区修改前 |
| --- | --- | --- |
| yaw GM6020 | `BSP_CAN_1`，ID5，反馈 0x209 | `BSP_CAN_2`，ID5，反馈 0x209 |
| pitch GM6020 | `BSP_CAN_2`，ID6，反馈 0x20A | `BSP_CAN_2`，ID6，反馈 0x20A |

实际硬件是 yaw 接 CAN1、pitch 接 CAN2。本工作区把 yaw 注册到了 CAN2，CAN2 上只有 pitch 的 0x20A 反馈，yaw 的 0x209 永远不会被路由，因此表现为电机反馈缺失而不是硬件故障。

CAN 过滤器为全接收掩码（`User/bsp/can.c`），CAN1 本身可以收到 0x209，问题只在注册总线。

## 修复

`User/device/can_devices.c`：

- `can_devices_gimbal_param[0].can` 由 `BSP_CAN_2` 改为 `BSP_CAN_1`
- 注释改为“yaw 使用 CAN1 ID5，pitch 使用 CAN2 ID6”，说明两轴分属不同总线

重建后 ELF `0207bcdc…`，烧录 `Verified OK`，复位运行。

## 验证

| 检查 | 结果 |
| --- | --- |
| 固件一致性 | `match` |
| `init_status` | 1（OK） |
| yaw 反馈 | `feedback_time_us` 持续前进，`encoder_deg = 98.39°` |
| yaw 轴状态 | RUNNING，SMC 生效 |
| SMC 观测 | `error=0.45°`、`s=5.41`、`u=-0.186 A`，实际输出 `-0.178 A` |
| 3 秒保持 | 角度 0.288°~0.334°，电流 -0.14~-0.19 A，速度 ±0.027 rad/s，无振荡 |
| 构建与语法 | Debug 构建通过，`can_devices.c` ARM 严格语法退出码 0 |
| 格式扫描 | 无违规 |

## 验证边界与遗留

- 仅台架验证，未做目标角阶跃和负载测试
- yaw SMC 的 J 仍是假定值 7.0757e-4 A/(deg/s²)，未按固定电流测角加速度标定
- 静止保持约有 0.3° 稳态偏差，来自摩擦阻力和 SMC 无积分项；参考工作树的 yaw 摩擦前馈约 0.2648 A，可作为后续补偿方向
- 未提交电源、线束方向等硬件结论

## 下一步

1. 标定 yaw 的 J：固定电流测角加速度
2. 用角度调试入口做 5°/10° 阶跃，评估超调与稳定时间
3. 视需要增加摩擦前馈或调整 C/K/epsilon
4. pitch 侧继续补重力前馈

## 追加修复：电流帧按轴所在总线分别发送

修好注册总线后，yaw 反馈恢复，但用速度调试模式打 0.5 A 恒流脉冲时 **yaw 完全不动**（速度仍在 ±0.03 rad/s，角度只变化 0.015°），说明电流没有真正下发。

根因在 `CANDevices_ApplyGimbalCurrent`：旧实现写完全部轴缓存后，只用最后一个注册轴（pitch，CAN2）的 `flush_param` 发送一次 0x2FE。yaw 在 CAN1，因此永远收不到电流控制帧；此前看到的 yaw 保持电流只是软件缓存值，电机实际没有出力。

修复方式与参考工作树一致：保存两轴 flush 指针，两轴同总线时只发送一次，分属 CAN1 与 CAN2 时分别提交各自的 0x2FE 帧。

修复后烧录 `85ae87df…`，复位后 yaw 反馈正常、轴进入 RUNNING，输出电流变为非零真实值。此前基于脉冲得到的 J 结果无效（电机未收到电流），需要重新标定。

### 追加修复的验证

| 检查 | 结果 |
| --- | --- |
| Debug 构建 | 通过，ELF 1919052 字节 |
| ARM 严格语法 | `can_devices.c` 退出码 0 |
| 烧录 | `Verified OK`，复位运行 |
| yaw 运行状态 | RUNNING，反馈持续更新，输出电流非零 |
| 格式扫描 | 无违规 |
| 边界 | yaw 动态与 J 标定尚未完成 |

