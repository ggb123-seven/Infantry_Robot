# 拨弹逐齿位置环迁移验证

日期：2026-10-31，工作区：F:/RM/Infantry_Robot

## 授权和范围

- 用户明确授权迁移 _smc 的逐齿位置环至正在运行的 Infantry_Robot，并复用 Feeder_t、Feeder_Config_t 的原有字段
- 新增 User/module/feeder.c、feeder.h；修改 User/task 的任务路由、邮箱类型、头文件与说明
- CMake 和 Keil 登记 feeder.c；module/config.c、device、bsp、生成代码、第三方源码和 .ioc 均无改动
- 未启动新调试会话，未暂停目标，未烧录、未写 RAM 增益或外设寄存器

## 控制链与参数

DR16 任务 -> trigger_command 单槽遥控快照邮箱 -> Task_motor_trigger -> Feeder_UpdateFeedback/Control/DumpOutput -> trigger_output -> CAN 任务合并 0x1FF

- 邮箱句柄与创建失败枚举沿用既有对象；trigger_command 元素由 8 字节速度命令改为 48 字节 DR16_State_t，新增有效载荷 40 字节
- 事件驱动发布遥控快照，拨弹任务 500 Hz 缓存后每周期调用模块，空邮箱保留快照，读取异常撤销有效性
- 模块按源时间检查遥控在线与 100 ms 帧龄，不刷新历史帧的时间
- 右拨杆下位运行，上位、中位、非法位置及遥控失联撤驱；反馈超过 10 ms 或角度连续性无效进入 OFFLINE
- 恢复运行或参数变化重新捕获实测位置与相位，首齿立即执行，其后每 50 ms 前进一齿，不追赶停机期间目标
- 默认 10 齿、20 Hz、方向 -1，位置 PID k=12、p=1、i=0、d=0.0450000018、i_limit=1、out_limit=1、d_cutoff_freq=-1、range=2π
- 参数来源为 _smc/User/module/feeder.c，与用户指定的未命名文件夹/User/module/config.c 步兵位置环一致
- PID 使用输出轴 rad 和实测速度换算的微分，归一化输出乘 10 A，允许达到参考工程的 ±10 A；旧速度模块的 2 A 不再约束此新实例
- 旧 trigger_config 保留原样，但不再驱动拨弹任务；不把位置环增益填入 rpm 速度环
- Feeder_t feeder 和 DR16_State_t trigger_remote 为观测入口，feeder.config 可调，cycle_config 为持久周期快照
- Control 入口只读一次 volatile 配置副本，集中校验并处理变更；PID 参数指针始终指向 feeder.cycle_config.position_pid
- 故障须在线遥控切换上位或中位解除；配置无效期间不使用其参数计算，DumpOutput 仅按运行状态导出电流或零
- 电流发布失败后下一周期撤销一次请求清除控制历史，任务每周期只调用一次 Control

## 自动验证

- 实现前和实现后 python -B .auto-embedded/scripts/check.py：ARCH、HW、SPEC 全部 PASS
- python -B .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug：成功
- 复用 compile_commands.json 的 ARM 命令，追加 -Wall -Wextra -Werror -fsyntax-only
  - feeder.c、motor_trigger.c、init.c、dr16_task.c、motor_chassis.c、user_task.c 六项通过，无诊断
- GCC 主机回归编译真实 feeder、PID、filter、user_math，纳入 DR16 分发、Task_InitCreateObjects、拨弹任务及 CAN 任务生产代码
  - 10522 项断言通过，结果见 feeder_migration_20261031.json
  - 覆盖首齿、48/50 ms 齿距边界、10 秒 200 个后续齿距与长时间相位、跨零位置和电流限幅
  - 覆盖右拨杆上下中及非法值、100000/100001 us 遥控边界、未来时间、离线、无缓存
  - 覆盖 10000/10001 us 反馈边界、低 32 位时间回绕、angle_valid=false、恢复重捕获
  - 覆盖 15 类非法在线参数、配置快照更新、方向变更、持久 PID 指针、非法周期和非法状态
  - 覆盖 FAULT 在使能和无效遥控下保持，在线停机档解除
  - 16 个邮箱逐个注入创建失败，均在创建线程前返回；成功路径创建 7 个业务线程
  - 覆盖遥控三路分发一路 Put 失败后继续其他路，拨弹读取错误、空邮箱、输出发布失败后撤驱
  - CAN 任务收到拨弹电流时合并发送，后续没有新输出时输出零，不沿用旧电流
- 最初主机夹具类型声明、环绕角度期望和超时步长设置错误已修正；正式源码无需以兜底分支适配夹具
- Keil XML 解析和 feeder.c 路径检查通过；环境探测未发现 UV4/ARMCC/ARMCLANG，未执行 Keil 编译
- ELF 符号验证 feeder 大小 216 字节、trigger_remote 48 字节，旧 trigger/trigger_command 全局符号不存在
- 编译器局部栈报告：任务 168 字节、Init 240 字节、Control 136 字节，不代表含库函数、中断嵌套的完整栈峰值
- 临时主机测试源码、桩头文件和可执行程序由 TemporaryDirectory 清理，未删除正式测试资产

## 校验自查与人工复核

| 新增或迁移的检查 | 信任边界与触发场景 | 上游是否检查同一条件 |
| --- | --- | --- |
| Init 配置与 frequency_hz，失败 return -1 | 初始化参数无效或 PID 初始化失败 | 首次建立实例时检查 |
| Control 配置字段 isfinite、齿数、频率、方向、PID 限幅和 range | EmberProbe 写入 RAM 后本周期快照可能非法 | 只在入口集中校验，下游信任结果 |
| received、angle_valid、10 ms 帧龄和 now_us>=age_us | CAN 新帧缺失、角度跟踪失效、源时间异常 | 驱动解码整数物理量，模块接收边界检查连续性与时效 |
| remote_valid、online、源时间和100 ms 帧龄 | 遥控读取错误、断链、过期或未来时间 | 拨弹模块独占该消费链的时间校验，不再调用底盘命令校验 |
| period_s isfinite 和 (0,0.1] | 调度延误、周期异常 | 在 Control 入口检查一次 |
| 目标角度和微分 isfinite，失败进入 FAULT 并 return | 极端有限 RAM 参数在换算中溢出 | 本层计算链出口检查，未重复检查 PID_Calc 返回值 |
| 非 RUNNING return | 状态机禁止控制，阻止使用无效快照 | 输出闸门仍仅为 DumpOutput |
| RTOS Get/Reset/Put 返回值 | 空邮箱、读取异常及发布失败 | 每次操作仅在发生处判断 |

未新增实例判空或句柄判空；没有新保护性回退零参数；删除的重复行为为旧任务发布失败后同周期第二次 Control
未迁移 _smc CAN 任务里的控制业务，总线仍由现有 CAN 任务独占，ISR 不运行新算法，无新共享中断状态或临界区
人工核对 Allman、120 列、行尾空白、中文阶段注释、Doxygen、结构体集中字段说明及依赖方向通过

## EmberProbe 与真机边界

- feeder、trigger_remote 已添加到 EmberProbe 侧栏；旧观测项可在界面移除，新固件应使用 feeder.config.position_pid
- 当前 sampling.status 为 stopped；probe.list 设备为空，烧录预检 flashAuthorization.unavailable=true、PROBE_IDENTITY_UNVERIFIED
- 虽然本地 OpenOCD 版本和目标脚本兼容，实际连接身份尚不可验证，因此不申请或执行无效烧录计划
- 接回探针后须重新 preflight，展示确切 ELF 指纹、目标、探针身份、传输和 OpenOCD 路径，再确认烧录
- 新 ELF 未下板，侧栏新地址不能作为旧固件实时状态的证据
- 真实方向、逐齿响应、参考 10 A 上限在当前机构的适用性、RTOS 动态堆和栈余量、失联时延均待上板验证
