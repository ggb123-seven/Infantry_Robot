# 右拨杆下位启动摩擦轮

日期：2026-11-03

## 行为变更

- `Task_motor_trigger` 以已校验的 `feeder_command.mode == FEEDER_CMD_RUN` 作为右拨杆下位信号，摩擦轮不依赖拨弹状态或拨弹电流邮箱发布结果
- 右拨杆上位、中位、非法档位或遥控失效时，`Cmd_ParseFeeder` 不会给出运行档，摩擦轮命令保持失能
- 摩擦轮命令自身的邮箱发布失败时，模块仍按缺命令失能
- `Shoot_Control` 保留两只摩擦轮反馈均有效且新鲜才输出电流的既有条件

## 验证

- `cmake --build build/Debug --parallel`：通过，修改任务编译并完成 ELF 链接
- `python .auto-embedded/scripts/check.py`：ARCH、HW、SPEC 全部通过
- `powershell -ExecutionPolicy Bypass -File .auto-embedded/scripts/arch-check.ps1`：0 violations；未找到硬件资源表，硬件锁专项检测按脚本提示跳过
- 本次未烧录固件或运行真实摩擦轮，未验证实物方向、启动电流和转速响应

## 运行时追查

- EmberProbe 只读采样时，当前 ELF SHA 为 `dc1d5e935333c00ab6495c4f0eee84b5f8870b1ca89e1aa1f56349762a3b0cc5`，记录的已烧录 SHA 为 `edd5b9222e72d846ce1717edaa0ec52a3367b34a2af5244b647c0304926bec99`；固件不匹配，变量地址读数不作为可靠结论
- 该次采样的遥控在线标志为 0，右拨杆值为 0（错误档），不能验证拨杆下位时的运行链
- 第二次调整移除摩擦轮使能对拨弹输出发布状态的依赖；当前仍需在遥控在线且右拨杆保持下位时复核摩擦轮反馈、`shoot.state`、目标转速及电流输出
