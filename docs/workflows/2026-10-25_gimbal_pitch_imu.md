# 云台 pitch 参数同步与陀螺仪角速度接入

## 两步实施范围

1. 同步 pitch 与 yaw 的配置及控制参数，分别保留电机标识和机械标定
2. 将角速度反馈从电机编码器转速换算切换为 IMU 陀螺仪，并验证坐标、单位、时效和故障输出

## 第一步：参数同步已完成，输出接入待确认

- `User/module/config.c` 的 pitch 两环参数与本轮工作区中的 yaw 参数一致，分别保存独立副本
- 角度环：k=1、p=15、i=0.005、d=0、i_limit=1、out_limit=320×0.10471975512 rad/s、d_cutoff_freq=0、range=0
- 速度环：k=1、p=0.06、i=0.177、d=0、i_limit=0.5/0.177、out_limit=0.5 A、d_cutoff_freq=0、range=0
- 两轴限速约 33.51 rad/s，限流 0.5 A；同步参数不代表 pitch 已完成实际整定
- 保留 yaw ID5/0x209、pitch ID6/0x20A，均在 CAN2，电流控制帧为 0x2FE
- 保留 pitch 的 disabled 状态、零点 0° 和限位 [-20°,20°]；零点和限位当前为待实测占位值
- yaw 角度环 p=15、i=0.005 是任务开始前用户已有的未提交调参，本次不将该差异纳入提交
- 当前设备接口仍将 pitch 电流固定为零，接通输出依赖 `User/device/can_devices.c/.h` 和 `User/task/can_task.c`
- 输出接入需取得设备目录修改授权，并明确 pitch 标定与启用安排

## 第二步：待确认后实施

- 当前云台使用串级 PID；SMC 仅完成 C 语言迁移，尚未调用，需确认是否在本步替换控制器
- `INS_State_t.Gyro` 为安装修正后的机体系角速度，单位 rad/s；现有 PID 使用 rad/s，SMC 使用度每秒
- 需确认 IMU 固定位置、安装轴向及正方向；pitch 转动时机体系 Z 分量不能直接等同于 yaw 电机轴角速度
- 按实际机械轴对陀螺仪向量进行投影；保留编码器角度反馈及电机在线检测，角度来源迁移另行确定
- 跨任务传递一致的最新快照，附带有效性和时效检查，禁止无同步地直接读取正在更新的 INS 全局对象
- INS 未就绪、故障、数据过期或非有限时，进入既有故障状态并输出零电流；恢复按既有 RELAX 解锁流程处理
- 若需要新增或调整结构体，先按根目录规则与用户协商名称及字段，确认后再实施

## 本轮验证证据

- `python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --list-presets`：识别 Debug、Release
- `python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug -j 2`：配置、构建成功
- 固件产物：`build/Debug/Infantry_Robot.elf`，生成器为 Ninja
- `python .auto-embedded/scripts/check.py`：ARCH、HW、SPEC 全部通过
- `git diff --check`：通过
- Python 内联扫描确认 config.c 为 UTF-8，单行不超过 120 列、Allman 花括号、无行尾空白
- 人工复核确认仅修改只读配置和集中说明，未增加结构体、任务、ISR 共享数据或临界区，无硬件配置变化
- 未烧录，未进行 pitch 真实电机整定，未验证 IMU 安装方向或闭环效果；未执行 Keil 构建
- 本轮未创建临时测试源码、测试可执行文件或其他临时测试程序

## 后续：固化 pitch 水平零点

- 用户按水平位置的 `gimbal.pitch.encoder_deg` 提供实测读数 118.08016°
- 将 `.pitch.zero_encoder_deg` 从占位值 0° 更新为 118.08016°，并同步配置上方的集中说明
- 此零点沿用当前配置 `reversed=false` 的编码器坐标；后续改变方向设置或机械安装关系时需重新核对零点
- pitch 保持禁用，现有 [-20°,20°] 仍为占位限位，待实测上下行程
- 本轮仅固化用户提供的测量值，未修复 pitch 连续角首帧落入 [0°,360°) 的初始化问题，未接通 pitch 电流输出
- CMake Debug 配置和构建通过，产物为 `build/Debug/Infantry_Robot.elf`
- ARCH/HW/SPEC、UTF-8、120 列、Allman 花括号、行尾空白与 `git diff --check` 检查通过
- 人工复核确认没有新增结构体、分支、RTOS 对象、ISR 共享状态、临界区或硬件配置，修改限于零点常量和说明
- yaw 既有未提交调参继续单独保留；未烧录、未实际复核水平角或闭环效果，未创建临时测试程序

## 后续：pitch 目标角度上下各 60°

- 按用户指定，将 pitch 的目标角度范围从 [-20°,20°] 更新为 [-60°,60°]，参考水平零点 118.08016°
- 同步更新配置表上方的集中说明，此数值为用户指定的目标范围，不代表已完成机械行程实测
- pitch 继续禁用，连续角首帧正负区间处理与 pitch 电流输出接入仍待实施
- CMake Debug 配置、构建及 UTF-8、120 列、Allman、行尾空白、`git diff --check` 检查通过
- 人工复核确认只修改限位常量与集中说明，没有新增结构体、分支、并发访问或硬件配置
- 未烧录、未做机械行程或闭环实测，未创建临时测试程序；yaw 已有调参差异不纳入本次提交

## 后续实施一：pitch 有符号角度

- 用户已授权实施正负角初始化及两轴独立电流输出，涵盖云台模块、设备输出接口及 CAN 任务
- pitch 每帧按 `Wrap(方向修正后编码器角度 - 水平零点 + 180, 360) - 180` 映射到 [-180°,180°)
- 水平零点为 118.08016° 时，113.08016° 对应 -5°，123.08016° 对应 +5°；首帧及失联恢复直接确定正负角
- yaw 保留连续角展开；pitch 不使用展开器，但保留重复帧、时间倒退及帧间隔超时校验
- 未新增结构体或字段，复用现有单轴 RELAX/RUNNING/FAULT 状态机，默认命令仍为 RELAX
- 主机使用真实 gimbal、angle_unwrap、pid、filter、user_math 源码，临时 BSP 头仅隔离 HAL 依赖
- 编译命令：`gcc -std=c11 -Wall -Wextra -Werror -O0 -ffunction-sections -fdata-sections -Ibuild/gimbal_signed_check -IUser build/gimbal_signed_check/control.c User/module/gimbal.c User/component/angle_unwrap.c User/component/pid.c User/component/filter.c User/component/user_math.c '-Wl,--gc-sections' -lm -o build/gimbal_signed_check/control.exe`
- 执行 `build/gimbal_signed_check/control.exe`：401 项断言通过，覆盖 -60° 至 +60° 各首帧、零点两侧、编码器跨零、反装、电流方向、目标限幅、0.5 A 限流、90 ms 边界、单轴故障隔离、RELAX 解锁、重复及倒退帧、恢复、修改零点及非法反馈/周期
- `python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug -j 2`：成功，产物 `build/Debug/Infantry_Robot.elf`
- `python .auto-embedded/scripts/check.py`：ARCH/HW/SPEC 全通过；UTF-8、120 列、Allman、空白扫描和 `git diff --check` 通过
- 逐文件重读 AGENTS.md、agent.md 并人工复核注释、返回值、分层和安全输出；没有改变 RTOS 对象、ISR 共享状态或硬件配置
- 未烧录，机械正方向、真实行程及闭环效果待上板验证；临时回归程序在本任务最终记录后清理
