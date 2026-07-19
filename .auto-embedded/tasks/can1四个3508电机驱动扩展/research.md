# 研究发现

| 关键词 | 来源 | 摘要 | 可信度 | 状态 |
|---|---|---|---|---|
| 工程基线 | `.auto-embedded/spec/hardware/index.md` | STM32F407IGH6 + HAL + FreeRTOS/CMSIS-RTOS2，CAN1 已配置为 PD0/PD1 | 高 | 已确认 |
| 硬件资源 | `.auto-embedded/spec/hardware/hw-lock.yaml` | CAN1_RX/CAN1_TX 和 CAN1_RX0_IRQn 已锁定，增加同总线电机不需要新引脚、DMA 或中断 | 高 | 无变更 |
| 电机 ID | `User/device/motor_rm.c` | M3508/C620 反馈 ID `0x201~0x204` 映射到低四槽，共用 `0x200` 控制帧 | 高 | 已确认 |
| 组帧方式 | `User/device/motor_rm.c` | `MOTOR_RM_SetTorqueCurrent()` 分别写入发送缓存，`MOTOR_RM_FlushGroup()` 将四个槽位一次打包为 `0x200` 帧 | 高 | 已确认 |
| 分层归属 | `User/task/motor_chassis.c` | 现有任务已拥有电机注册、反馈更新、速度环和电流下发编排，扩展为四实例应保持该边界 | 高 | 已确认 |
| 安装方向 | 现有单电机参数 | 目前只能确认单电机不反向；四电机先全部配置为不反向，最终方向需低速上板核对 | 中 | 待实测 |

## 研究结论

- 本次仅需修改 `User/task/motor_chassis.c` 和 `User/task/motor_chassis.h`。
- 使用 C620 电调 ID 1~4，对应反馈 ID `0x201~0x204`。
- 每台电机拥有独立目标转速、PID 状态、在线状态和调试监视字段。
- 每个控制周期先写入四个电流槽位，再统一发送一次 `0x200` 控制帧。
