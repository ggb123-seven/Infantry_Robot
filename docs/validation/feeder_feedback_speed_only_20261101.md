# 纯速度拨弹限流与反馈失效诊断

日期：2026-11-01；基线提交：9035a14

## 已观察事实

- 只读采样保存于 feeder_limit_offline_diagnosis_20261101.json，timestamp=1793532343079
- EmberProbe 报告 firmwareConsistency=match，旧固件 ELF SHA-256=db67fca45ef88107f02ba90c008ca12fe3ecdfa8ffeeef2d82f67cb7553c3569
- feeder.config.speed_pid：k=1、p=0.16、i=d=0、out_limit=4；max_current_a=4、max_speed_rpm=300
- feeder.state=2，即 FEEDER_OFFLINE；initialized=config_valid=1，feedback_valid=0；右拨杆下位，遥控有效且发布成功
- 已核对 CAN2、0x207、M2006、gear=true 的驱动实例：online=1、angle_lost_count=1、angle_valid=0，驱动反馈时间继续更新
- feeder 保存的最后可信角度反馈时间停留在 32039361 us，驱动已更新至 243693333 us，说明新速度帧因角度标志被模块拒绝
- ID7 的 0x1FF 发送快照槽值为零；候选 output_current_a 残留值不等于闸门放行电流，发送成功状态也不等于电机已达到目标
- 现场采样时用户请求已经是零，不把此样本描述为非零目标运动测试

## 原因与边界

电流边界限制转子电流，目标速度另由 max_speed_rpm 限幅；源码没有根据实际转速超过 300 rpm 直接撤驱的分支
P=0.16、I=0 的控制器在存在维持转动所需电流时可保留稳态速度误差；是否达到电流饱和以及负载、供电的影响还需运行曲线，不能仅由本次停机采样确认

驱动在解码转子角度差绝对值达到 3686 编码器计数时递增 angle_lost_count，之后 angle_valid 持续为假
这表示角度连续跟踪失去可信性，不等于协议中的独立转速字段无效；高速下帧间角度跨度或漏采可能触发，但本轮未捕获触发瞬间，不给出固定实测超速门槛
原 Feeder_UpdateFeedback 把 angle_valid 当成整个速度反馈的有效性，因此角度失效后即使 CAN 持续在线仍进入 OFFLINE

## 修复范围

- feeder.c 接收驱动已校验完整性的速度反馈，速度有效性继续由是否收到帧与 10 ms 帧龄确定
- 保留反馈中的 angle_valid、angle_lost_count，角度观察值无效时为 NaN，不伪造可信角度
- feeder.h 同步字段语义；不新增结构体或字段，不改 config.c、设备协议、BSP、CubeMX、通用 PID 或控制增益
- 当前位置外环仍旁路；将来恢复位置环必须恢复角度可信性要求，不可直接复用本次纯速度条件

## 验证命令与结果

- python .auto-embedded/scripts/check.py：修改前及提交前 ARCH/HW/SPEC 全 PASS
- python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug：构建通过，产物 build/Debug/Infantry_Robot.elf
- 复用 compile_commands.json，对 feeder.c、motor_trigger.c 添加 -Wextra -Werror -fsyntax-only：均通过
- python build/validate_feeder_feedback_20261101.py：基线 6007 项、新版 6129 项主机断言通过；调用真实 feeder/config/PID/filter/user_math，仅替身 CAN 枚举隔离 HAL
- 基线精确复现 angle_valid=false 后 OFFLINE 与零输出；修复版在实测 500 rpm、目标 120 rpm 时保持 RUNNING 并输出 -4 A 制动，负 400 rpm 时同样保持运行
- 新旧正常反馈 1000 周期状态、目标、实测、输出及角度镜像逐位相同，轨迹 SHA-256 见同名 JSON
- 10 ms/10001 us 断帧边界、反馈恢复、低 32 位回绕、未来帧拒绝、遥控超时、周期异常、非法配置撤驱均通过
- 验证目标 400 rpm 被限制为 300 rpm，输出仍限制为 4 A；P 误差 10 rpm 时输出保持 1.6 A，测试中仅修改局部实例 I 参数后积分增长，证明参数路径有效，不将测试数值作为实机推荐增益
- 源码中文注释、Allman、120 列及行尾空白检查通过，git diff --check 通过

## 信任边界与代码质量自查

本次未新增判空、isfinite、范围检查或保护性 return；原 10 ms 帧龄、遥控、实测周期、配置和计算出口检查保留
移除纯速度模式对角度连续性条件的整体阻断；角度有效性只用于角度镜像，不在下游重复校验转速
新增 NaN 赋值仅用于角度观察字段，实际触发场景为本次已测 angle_lost_count=1，不参与速度控制
没有修改 main.c，没有新 RTOS/HAL 对象、ISR 共享状态或临界区；可写 requested 为 volatile，PID 参数仍引用实例内持久 cycle_config
DumpOutput 仍是唯一模块输出闸门，OFFLINE/RELAX/FAULT 均为零

## 尚待实测

本轮仅对旧固件执行只读诊断，没有写 RAM、复位、烧录或主动驱动电机
修复 ELF 重新构建后与板上旧固件不同，后续调试必须重新烧录并使用匹配 ELF；新固件高转速及负载速度精度未验收
临时测试目录自动清理，临时 Python 脚本于证据保存后删除
