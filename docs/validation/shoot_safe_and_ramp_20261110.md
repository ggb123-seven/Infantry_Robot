# 摩擦轮安全撤驱与一秒斜坡

## 用户确认

2026-11-10 用户要求安全档关闭速度环，并要求启动及变为零速时有一秒缓启动、缓停止
随后明确：安全档立即关闭速度环，一秒缓停止仅用于其他仍保持使能的零速命令；遥控失联和故障保持立即撤驱
用户随后授权修改 `User/module/shoot.c`、`shoot.h`，并选择在既有 `Shoot_t` 中增加
`ramp_target_rpm[2]` 和 `ramp_rate_rpm_s[2]` 两个数组字段

## 最终行为与实现

- `User/task/motor_trigger.c`：安全档不再生成 enabled=true，改为失能且零速目标
- `User/task/motor_shoot.c`、`motor_shoot.h`、`user_task.h`：同步安全档语义的集中注释
- `User/module/shoot.c`、`shoot.h`：复用实例当前目标，保存各轮斜坡终点和变化率，调用现有 `RampTo`
- 目标经过本周期配置快照限幅，终点变化时按 `abs(终点 - 当前平滑目标) / 1 s` 计算独立变化率
- 每周期按实测 `period_s` 推进；重复相同命令不重置，两轮互不影响，中途改目标重新用一秒过渡
- 追加 `SHOOT_RAMPING=4`，原有枚举数值保持不变；过渡阶段和 `SHOOT_RUNNING` 均执行速度闭环并允许输出
- 进入或离开斜坡不重置 PID；只有恢复起控或公共参数改变才沿用原逻辑同步参数与重置历史
- 失能、缺命令和反馈失联立即撤驱，清除平滑目标、终点、变化率和 PID 历史；故障继续要求有效失能解除
- `Shoot_DumpOutput` 是唯一输出闸门，故障、离线、待机和非法状态均输出零电流
- 使能零速斜坡结束后仍保持零速闭环；安全档不属于这个零速命令路径
- 目标差分前馈使用平滑后的逐周期差分，继续保留原系数、公式及每轮 3 A 限幅
- 两个数组追加到 `Shoot_t` 末尾，既有字段偏移保持，实例增加 16 字节；无新增结构体类型、任务、队列或 ISR 共享状态
- `module/config.c` 未修改，SHA-256：`92ef6b0b9f499d0b0d3cead64e00c7d8d9513d8e0acf5120ef81651283d4c8ce`

## 状态约定

| 状态 | 周期行为与退出条件 |
| --- | --- |
| RELAX | 零输出；有效使能且双轮反馈在线后重新起控 |
| RAMPING | 按实测周期推进目标并执行闭环；两轮到达各自终点后转 RUNNING |
| RUNNING | 保持目标闭环；新终点请求重新开始一秒过渡 |
| OFFLINE | 双轮撤驱并清理历史；恢复新反馈后从零目标重新缓启动 |
| FAULT | 零输出并锁存；只有有效失能命令才能解除，非法状态也进入本状态 |

安全档和命令缺失优先于斜坡，不等待一秒；反馈超时继续使用既有 100 ms 输入边界
一秒指目标轨迹时间，不承诺实物电机恰好一秒到速或停稳
离散周期及浮点舍入允许终点落在相邻控制周期，标称八千转启停在 500 Hz 下为 500 周期

## 验证

- 先前任务层安全补丁已通过 1007 条主机断言并提交 `d77ac2e`
- `cmake --build --preset Debug` 成功，最终 RAM 50408 字节、FLASH 129956 字节
- `compile_commands.json` 的真实 ARM 编译命令增加 `-Wextra -Werror -fsyntax-only`，检查 shoot.c、motor_trigger.c、motor_shoot.c 全通过
- 临时主机程序提取正式 `MotorTrigger_ParseShootCommand` 函数原文，链接正式 shoot.c、config.c、pid.c、filter.c、user_math.c，仅替代 CAN 总线编号以隔离硬件头文件
- GCC 主机回归：22 组初始化场景、41968 条断言通过，逐周期对比直线参考的最大误差 0.006568176 rpm
- 测试覆盖完整启停、499/500 周期边界、重复命令、半程停止、停止中重启、独立双轮改目标、正反向、非整转速、限幅、1/3 ms 交替周期
- 异常覆盖安全与缺命令在加减速期间撤驱、1000 周期持续安全、单轮超时恢复、非法配置/命令/前馈/周期、故障锁存解除、非法状态、前馈计算溢出
- 回归还覆盖在线 PID 改参不重启斜坡、PID 参数引用寿命、使用平滑目标的差分前馈、微秒低位回绕、双实例交错运行
- 主机测试源码 SHA-256：`6782a9e0f262acbd22ba75564c8f4e3f27c43ae3789da9a149fe981458cac1fb`
- ARM 静态栈用量：Shoot_Control 112 字节，Task_motor_shoot 152 字节；本次未增加任务栈内数组或阻塞延时
- 已逐项重新读取 AGENTS.md 和 agent.md，检查 Allman、120 列、行尾空白、控制块花括号、注释及 git diff --check
- 分层仍为 task 调用 module、module 调用 component，无反向依赖；未修改 main、外设、CubeMX 或 RTOS 对象
- ARCH、HW、SPEC 机械门禁全部通过
- 新 ELF SHA-256：`07f85a3cbb96cd08f0eecf0cc2ad3855ffb3626c43e0196743f7593b278ed334`
- 未烧录、未写目标 RAM，板上原固件不包含本次修改；震动消除、实际转速曲线和停机响应待上板确认

主机命令：

```powershell
gcc -std=gnu11 -fgnu89-inline -O0 -ffunction-sections -fdata-sections `
  -I User/task/.shoot_ramp_host/include -I User -I User/component `
  User/task/.shoot_ramp_host/test.c User/module/shoot.c User/module/config.c `
  User/component/pid.c User/component/filter.c User/component/user_math.c `
  '-Wl,--gc-sections' -lm -o User/task/.shoot_ramp_host/test.exe
User/task/.shoot_ramp_host/test.exe
```

测试记录完成后删除本次临时目录 `User/task/.shoot_ramp_host`，不保留临时测试源码或可执行文件

## 防御性检查自查

- 新增判空：无
- 新增 `isfinite`：无；继续使用 Control 入口配置快照与命令校验、合成电流出口检查
- 新增范围检查：无；目标限幅复用原有边界，RampTo 的步长检查由既有组件负责
- 新增保护性 return：无；非运行状态沿用原有清理与返回分支
- 新增的终点比较属于命令变更事件，阶段比较属于状态转换，不是重复输入校验
- 删除重复检查：无，未引入下游配置重验或 PID 输出重复检查
