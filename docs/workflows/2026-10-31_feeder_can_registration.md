# 拨弹 M2006 的 CAN 注册

## 授权与参数来源

- 用户针对 CAN 设备集合要求注册拨弹电机，确认型号 M2006、总线 CAN2，并要求同步 `F:/RM/未命名文件夹` 的 ID
- 参考工程 `User/device/can.h:75` 定义 `CAN_M2006_TRIG_ID = 0x207`，对应电调 ID7
- 参考工程 `User/module/config.c:198`、`:343` 的射击总线均为 `BSP_CAN_2`
- 复用现有 `MOTOR_RM_Param_t` 与 `MOTOR_RM_t`，启用驱动内置 36:1 减速比；`reverse=false` 尚未实物确认

## 实现与消息流

`Task_can_Init -> CANDevices_Init -> MOTOR_RM_Register` 将拨弹加入现有 CAN2 管理器，反馈由既有 BSP 最新帧入口汇聚

`Task_can -> CANDevices_UpdateFeederFeedback -> MOTOR_RM_Update` 在原 500 Hz 通信周期中更新拨弹反馈和在线状态，不新增任务、队列、结构体或结构体字段

拨弹注册与零缓存初始化结果纳入 `can_devices_snapshot.init_status`，失败保留其他电机通信能力。底盘四路快照的字段和含义保持原样，拨弹的独立诊断入口为：

- `can_devices_feeder_init_status`：拨弹注册和缓存初始化结果
- `feeder_feedback_status`：本周期反馈更新返回值
- `can_devices_feeder_motor->feedback`：最近成功解码的反馈，`rotor_speed` 为减速后的输出轴 rpm
- `can_devices_feeder_motor->motor.header.online`：驱动在线状态，初始为假，按既有 100 ms 超时判断

以上变量分别位于 can_devices.c 和 can_task.c，调试器遇到局部符号歧义时应按编译单元限定。反馈错误时保留最近成功值，观察时必须结合在线状态、时间戳与本周期返回值

## 输出边界与额外处理

初始化只写零缓存，本轮不发送拨弹控制帧，也未新增遥控、射击控制、反转或卡弹处理

现有驱动的 CAN2 输出槽位 4、5 同时用于 GM6020 的 `0x2FE` 数据和 M3508/M2006 的 `0x1FF` 前四字节；M2006 ID7 使用槽位 6，注册本身不冲突。但是直接调用拨弹的 FlushGroup 或 FlushCAN 会把云台非零缓存带入 `0x1FF` 的 ID5、ID6 位置。参考工程这两个位置为摩擦轮，因此本轮不增加该帧发送，未来启用射击前应先确定帧组隔离方案

这一处理原因已经写在 CANDevices_Init 的拨弹初始化阶段注释和参数集中说明中，CANDevices_UpdateFeederFeedback 的函数注释明确仅做反馈轮询

## 检查自查

新增判空、isfinite、范围检查均为零，没有新增兜底赋值或下游参数重复校验

新增的初始化结果检查包括注册返回值判断、聚合结果判断，以及反馈入口对已记录初始化状态的判断。前两项属于初始化信任边界，后一项用于注册或缓存初始化失败时隔离该设备，只使用已记录结果，不重复调用注册或校验参数

新增保护性提前返回仅 CANDevices_UpdateFeederFeedback 中的初始化失败路径，属于初始化信任边界的状态使用；上游没有针对拨弹重复检查同一条件

下层已保证注册成功后实例存在，因此未对 GetMotor 再判空；未在设备集合重复检查反馈帧格式、超时、浮点或减速比，均复用原驱动。无需删除本轮重复校验项

依赖为 task -> device -> bsp，新增实例指针只在启动装配阶段写入，反馈仅由 CAN 任务更新；ISR 继续经原 BSP 接收缓存交接，不新增 ISR 共享状态或 RTOS 对象。既有底盘和云台电流提交路径保持原样

每次修改源码文件后复读 AGENTS.md 和 agent.md，并检查中文注释、Allman 花括号、120 列、行尾空白、阶段目的、返回值和安全默认输出

## 验证证据

1. `python .auto-embedded/scripts/check.py`：ARCH、HW、SPEC 全部 PASS
2. `python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug`：配置与构建成功，采用 Ninja
3. 按 `build/Debug/compile_commands.json` 的真实 ARM 参数，对 can_devices.c、can_task.c 运行 `-Wall -Wextra -Werror -fsyntax-only`：无诊断
4. 主机 GCC `-std=c11 -Wall -Wextra -Werror`：在临时目录包含并执行真实 motor_rm.c、can_devices.c、can_task.c，仅替换 BSP、内存、时钟和 RTOS 边界
5. 四组场景通过：正常七设备、拨弹接收 ID 注册失败、拨弹内存分配失败、总线初始化失败；前三组各运行真实 CAN 任务三个周期，失败不妨碍原六台电机通信
6. 实际驱动验证 3600 转子 rpm 换算为 100 输出轴 rpm、100000/100001 us 离线边界、扩展帧/远程帧/短帧拒绝及反馈恢复
7. 验证初始零缓存和无初始化发送；云台非零电流不改变拨弹缓存，0x2FE 保留字节全零；实际任务仅发送底盘与云台帧，未发送 0x1FF
8. 临时构造 0x1FF 帧而不提交发送，证实现有缓存共用现象；临时源码与可执行程序已通过 TemporaryDirectory 自动清理
9. `git diff --check` 与源码格式扫描通过，具体机器记录见 `docs/validation/feeder_can_registration_20261031.json`

## 工程画像

```yaml
project_root: F:/RM/Infantry_Robot_smc
mcu: STM32F407IGH6
build_system: cmake
build_preset: Debug
generator: Ninja
build_dir: F:/RM/Infantry_Robot_smc/build/Debug
artifact_path: F:/RM/Infantry_Robot_smc/build/Debug/Infantry_Robot.elf
artifact_kind: elf
compiler: arm-none-eabi-gcc 14.3.1+st.2
```

本轮未烧录、未上板验证，未确认实物 ID、安装方向、总线负载或电机动作；参考参数同步与软件测试不代表实物验收
