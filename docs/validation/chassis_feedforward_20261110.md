# 底盘四轮速度前馈实现与架空整定

日期：2026-11-10；用户要求直接调试，前文已说明修改范围与四轮独立系数方案，用户随后要求继续执行

## 最终行为

源码只修改 User/module/chassis.c、chassis.h。四轮前馈入口为 `chassis_speed_feedforward_gain_a_per_rpm[0~3]`，依次对应左前、左后、右后、右前，单位 A/rpm。启动初值保存为 0.0137、0.0231、0.0097、0.0092；零值关闭对应轮前馈

单轮电流链为：换向保护后的目标 -> 目标斜坡 -> 速度前馈与 PID 电流相加 -> 现有电流滤波 -> 现有计算出口检查 -> 最终 10 A 限幅 -> DumpOutput 按模式与轮状态放行

在线前馈系数使用 volatile RAM 数组，每个周期在 Control 入口逐项读取到栈上快照，集中校验一次并合入 config_valid。任一系数非有限或为负时四轮待机，修正后按已有状态机预置恢复。下游只用本周期快照，不读取可调原值

保留所有既有结构体布局、RAM config 副本、持久 PID 参数指针和 DumpOutput 闸门。没有修改 module/config.c、设备层、BSP、CubeMX、任务源码、ISR、临界区或 RTOS 对象

注意：工作树原有 config.c、gimbal.c/h、motor_shoot.c 等未提交修改，构建固件包含这些原有修改；不能把整份 ELF 归因于本次底盘改动。调试前与本次最终源码中的底盘 P 均为 0.20、I=D=0，用户此前贴出的 P=0.18 不是本次实测运行值

## 软件验证

- `python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug` 配置和构建均成功
- 使用 build/Debug/compile_commands.json 中 chassis.c 的 ARM 编译参数，去除 -o、-c、-fstack-usage，添加 -Wextra -Werror -fsyntax-only，退出 0
- 真实 chassis、mixer、pid、filter、user_math 进行主机回归：PASS groups=28 assertions=26233 differential_cycles=2000
- 主机编译采用 GNU11、-fgnu89-inline、-O0、-Wall -Wextra -Werror；仅因主机为 64 位而屏蔽厂商头文件的指针与整数转换告警，未屏蔽业务代码其他告警
- 覆盖四轮独立系数、正反向前馈、PID 合成、目标斜坡、换向保护、零目标、正负最终限流、各轮 NaN/Inf/负系数、非法后恢复、在线系数修改、周期中系数和 config 改写、有限系数乘法溢出、单轮离线隔离与恢复、空命令、待机、非法模式、异常周期和初始化失败
- 与改动前 HEAD 中真实 chassis.c 在零前馈下比较 2000 个控制周期，模式、单轮状态、目标和电流一致，涵盖变速、正反转、待机和反馈离线
- 额外核对四轮编译启动初值；未建立新的结构体定义
- Allman、120 列、行尾空白、git diff --check 通过；保留完整 Doxygen 参数和返回值及中文阶段注释
- `python .auto-embedded/scripts/check.py` 的 ARCH、HW、SPEC 均通过；Windows 子进程执行策略通过当前进程环境变量 Bypass 处理，没有修改系统策略
- 本次 Chassis_Control 编译栈用量 152 B，完整任务栈高水位未测

尺寸工具输出：

```text
text	   data	    bss	    dec	    hex	filename
 129688	    200	  47024	 176912	  2b310	build/Debug/Infantry_Robot.elf
```

## 上板与权限证据

配置解析为 STM32F4、CMSIS-DAP，USB 序列号 332107146640，目标 UID 001D0035374E500820363355，OpenOCD 为 EmberProbe 配置的 0.12.0 开发版

首次板上读取为旧 ELF match，目标 30 rpm、P=0.20、I=0、限流 10 A。其后两次有限采样连接返回 OPENOCD_START_FAILED，未绕过 Bridge 或直接启动 OpenOCD/GDB。采样后来恢复；烧录遇到 PROBE_BUSY/liveWatch 后先查询状态，再停止共享采样释放探针

EmberProbe 返回 `authorization.authorized=true`、`confirmationRequired=false`、`mode=full` 的已有工作区完整授权，未由助手启用该授权。依据该授权完成两次 program/verify/reset：首次加入默认零前馈供调参，第二次保存最终系数。RAM 系数写入均 verified=true、halted=false，详情保存在 JSON 证据中

最终 ELF SHA-256：`56ab4b9d1796c2adc864a01dd5b7701d411fd08865eb7ab0336adcb1c7c06107`

最后一次烧录 code=0、verified=true，日志包含 Programming Finished、Verified OK、Resetting Target；后续采样 firmwareConsistency=match，且未再次写 RAM 系数即读回启动初值，验证的是烧录复位后自动加载，不是断电重启

## 实测整定

用户确认四轮架空。遥控持续给定独立模式纯旋转，四轮目标 30 rpm。未修改遥控命令、未注入目标值。先记录新固件零前馈基线，再以约一半基础驱动补偿试探，按实测响应调整为最终系数

采样记录依次为 stage0 100 点、stage1 100 点、stage2 150 点、confirmation 250 点、final 200 点，间隔请求 100 ms。每段统计均剔除前 20 点，最后一段为保存初值并重新烧录复位后的 180 个稳态采样点

| 车轮 | Kff A/rpm | 零前馈均值 rpm | 复位后均值 rpm | 复位后平均偏差 rpm | 最终采样电流峰值 A |
|---|---:|---:|---:|---:|---:|
| 左前 [0] | 0.0137 | 27.9908 | 29.9982 | +0.0018 | 0.5458 |
| 左后 [1] | 0.0231 | 26.6850 | 30.0786 | -0.0786 | 0.7756 |
| 右后 [2] | 0.0097 | 28.5258 | 29.8469 | +0.1531 | 0.5388 |
| 右前 [3] | 0.0092 | 28.5109 | 29.8784 | +0.1216 | 0.5051 |

confirmation 段各轮平均约为 29.8469、30.1383、29.8906、30.1299 rpm；final 段见上表。全流程采样电流指令峰值约 0.81 A，远低于 10 A。记录中没有实际电流反馈，电流指令不能表述为实测转子电流

改进针对平均稳态偏差，原始轮速仍有约 0.8~1.3 rpm 标准差；约 10 Hz 的有限采样不能证明不存在高频振荡。四轮独立系数只验证了架空正向 30 rpm，未做反向、多转速、落地负载、实车跟随角度、停机时延或断电重启验收

## 新增检查自查

| 新增条件 | 实际触发场景 | 信任边界 | 上游重复检查 |
|---|---|---|---|
| isfinite(feedforward_gain[i]) | EmberProbe 在线写入 NaN/Inf，或输入解析产生非有限数 | Control 入口在线调参快照 | 无，原配置不含该数组 |
| feedforward_gain[i] >= 0 | 在线写入负系数导致前馈方向错误 | Control 入口在线调参快照 | 无，原配置不含该数组 |

没有新增判空或保护性 return。前馈合成后的非有限计算复用原有出口检查，未新增下游重复校验，DumpOutput 未重检配置。删除项：无；测试源码内的断言不属于固件运行时防御检查

## 复核与产物

- 图表：chassis_feedforward_20261110.png
- 实测 CSV：chassis_feedforward_20261110_stage0.csv、stage1.csv、stage2.csv、confirmation.csv、final.csv
- 汇总与写回证据：chassis_feedforward_20261110_final_metrics.json
- 最终烧录证据：chassis_feedforward_20261110_flash.json
- 临时主机测试位于 User/task/.chassis_ff_host；自动审批两次拒绝清理命令，仅返回 blocked by policy，因此目录暂时保留且不纳入提交；其他用户已有临时目录保持原状
