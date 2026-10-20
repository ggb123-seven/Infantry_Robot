# XRobot IMU 解算移植记录

2026-10-19，trace_id：`IMU-XR-003`、`IMU-XR-004`

## 来源与修改

姿态核心来自 XRobot 官方仓库 [xrobot-org/MadgwickAHRS](https://github.com/xrobot-org/MadgwickAHRS)，
固定版本为 `e98a223b84e44ca8718fe8e68b4b63577ec70499`，参考文件为 `MadgwickAHRS.hpp` 的 `Update()`。
上游采用 Apache-2.0 许可证，原文保存在 [许可证文件](licenses/xrobot_madgwick_apache_2.0.txt)。
上游文件未包含专属版权署名；许可证原文按原样保留。

本项目将计算核心转为纯 C，使用现有 FreeRTOS 任务与 BMI088 驱动，没有引入 LibXR、Eigen、Topic 或新线程。
算法改动如下：

- 梯度为零时跳过反馈归一化，修正上游在理想水平静止时产生 NaN 的问题
- 检查空指针、非有限输入、非法增益与采样周期，失败时不改写输入四元数
- 归一化采用最大分量缩放，防止平方和溢出或下溢，并在计算前归一化四元数副本
- 四元数成功计算后再提交；加速度全零时沿用上游的纯陀螺仪积分行为

正常输入的计算公式与上游对照通过，浮点舍入误差见验证表。

## 层级与接口

| 层级 | 文件 | 职责 |
|---|---|---|
| Component / L4 | `User/component/madgwick_ahrs.c/.h` | 无实例结构体的六轴姿态计算 |
| Module / L5 | `User/module/ins.c/.h` | 安装修正、姿态发布、累计 Yaw、运动加速度与温控 |
| Task / L6 | `User/task/ins_task.c` | 1000 Hz 调度，传入微秒时基计算的实际间隔 |

`MadgwickAHRS_Update(q, gyro, accel, beta, dt_s)` 接收 `[w,x,y,z]` 四元数、`rad/s` 角速度、
`m/s²` 或 `g` 加速度、有限非负增益及有限正数秒间隔，返回 `MadgwickAHRS_Status_t`。

用户选择方案 A：不新增结构体，直接复用 `INS_State_t.q`；`INS_t` 删除 EKF 实例与状态，
改为 `ahrs_beta`、`ahrs_status`。增益默认取上游 BMI088 示例的 `0.033`，不是构造器默认的 `0.05`。
主错误码 `-6` 的名称改为 `INS_STATUS_AHRS`。

旧 `quaternion_ekf.c/.h`、仅供它使用的 `kalman_filter.c/.h` 已删除，CMake/Keil 移除相关矩阵源文件登记，
加入 `madgwick_ahrs.c`。原来未编译的 `ahrs.c/.h` 保留为未启用组件，不参与新姿态链路。

## 兼容约定与能力边界

`INS_Init()`、`INS_Update()`、`INS_GetState()` 及公开状态字段保持原接口。
初始化四元数为 `[1,0,0,0]`，因此任意倾斜上电仍需等待重力方向反馈收敛。
按标准姿态定义，绕 X 轴横滚发布到 `Roll`，绕 Y 轴俯仰发布到 `Pitch`；
`Yaw` 仍为绕 Z 轴偏航，单位均为度。`asinf` 输入限制在 `[-1,1]`。
2026-10-19 根据用户反馈修正此前沿用的反向字段映射，验证结果见下方补充记录。

累计 Yaw 在 INS 中使用相邻主值的最短角差累加，消除 ±180° 边界跳变。
它要求相邻有效采样间旋转小于 180°，不消除零偏漂移；大累计值仍受单精度浮点分辨率限制。

BMI088 BSP/Device、静止校准、CubeMX 配置、加热 PID 与故障状态机沿用当前工程。
Madgwick 不包含旧 EKF 的在线零偏估计和卡方门控；设备层校准依然重要。
没有磁力计或外部航向观测，不能承诺长期绝对 Yaw，也未将姿态接入底盘控制。

原 [入门文档](imu_bmi088_ins_guide.md) 保留为旧实现说明；其中 EKF 字段、测试和单位问题描述不能作为当前结论。
当前设备读取路径为 `raw × BMI088_ACC_SENSITIVITY_MPS2_PER_LSB × BMI088_AccelScale`，
本轮未修改该设备代码，也没有测量实际原始样本。

## 软件验证

原始结果保存在 [验证 JSON](validation/imu_xrobot_20261019.json)。主机使用 GCC、实际 INS/Madgwick/PID/Filter 代码，
仅对 BMI088 采集和加热硬件接口提供桩。上游对照直接从固定版本 `Update()` 提取公式，未手工重写参考公式。

| 检查 | 结果 |
|---|---|
| 上游随机输入单步对照，固定随机种子 20261019，10,000 组 | 四元数分量最大差 `2.38419e-7` |
| 上游连续 20,000 步、变采样周期对照 | 四元数分量最大差 `5.42403e-6` |
| 水平静止、零梯度，10,000 步 | 保持单位四元数；同时复现上游 NaN |
| X/Y/Z 正反 90° 纯积分、0.5/1.5 ms 交替采样 | 四元数分量最大误差 `4.29335e-7` |
| X/Y 轴 30° 倾斜、每组 15,000 步 | 收敛误差小于 `0.02°` |
| 空指针、非法 dt/beta、NaN/Inf、零四元数、积分溢出，共 32 例 | 返回错误，保持原四元数字节不变 |
| INS 发布、温控二分频、静止和倾斜重力分离、原角度映射 | 历史验证通过；角度映射以下方修正记录为准 |
| 累计 Yaw 正反各四圈后反向回零 | 连续无跨周跳变，四圈累计误差最大 `0.04834°` |
| 设备失败、无效样本、加热错误、非法状态和数值错误，共 10 例 | 清公开状态、停热、故障锁定 |
| GCC C11 `-Wall -Wextra -Werror -pedantic` 核心语法检查 | 通过 |
| ARM Debug 全量构建，85 步 | 通过；RAM 42,176 B，Flash 102,288 B |
| ARCH/HW/SPEC、Keil XML 解析与源文件登记检查 | 通过 |

构建命令为 `cmake --build --preset Debug --clean-first --parallel 4`。
产物为 `build/Debug/Infantry_Robot.elf`；Keil 本轮仅检查登记与 XML，未执行 Keil 编译。
临时主机测试源码、参考公式副本、DLL 和脚本在保留结果后清理。

本轮没有烧录、复位或上板测量。仍需验证实际三轴方向、任务耗时与栈余量、静止噪声、
倾斜收敛时间、温控稳定性及重复启动后的长期 Yaw 漂移。

## Roll/Pitch 映射修正验证（2026-10-19）

用户确认将输出统一为 `Roll=绕 X 轴横滚`、`Pitch=绕 Y 轴俯仰`。
修改仅涉及 `ins.c` 的两处赋值及说明、`ins.h` 的集中字段说明，结构体字段和布局保持原样。
上方历史 JSON 中的 `ins_legacy_roll_pitch_mapping_and_rotated_gravity` 验证的是旧映射，不能证明本次修正。

主机测试编译实际 INS、Madgwick、PID、Filter 和 user_math 代码，只替代 BMI088 读取与加热硬件接口。
测试通过 `INS_Init()`、`INS_Update()`、`INS_GetState()`，使用已知四元数、零角速度和 `beta=0` 隔离发布映射。
共 13 组：零姿态、X 轴 ±30°/±90°、Y 轴 ±30°/±60°、Z 轴 ±90°、两组组合姿态。
全部通过，三轴最大角度误差 `0.000003815°`，断言阈值 `0.001°`。
GCC C11 使用 `-Wall -Wextra -Werror -pedantic` 编译通过，临时测试源码与可执行文件已清理。

`cmake --build --preset Debug --parallel 4` 实际编译并链接成功，RAM 42,176 B、Flash 102,288 B。
ARCH/HW/SPEC、Allman/120 列/行尾检查和 `git diff --check` 通过。
人工复核：发布函数保留原有有限性检查、偏航回绕和失败返回；本次未引入外设配置、ISR 共享状态或临界区变更。
未烧录、复位或上板实测；实际安装方向仍待硬件验证。

本次源码和构建产物 SHA256：

| 文件 | SHA256 |
|---|---|
| `User/module/ins.c` | `8797a489c513dfbb1f9a50d5f021d5de0ba5b89385f72088d6968178176e9f7e` |
| `User/module/ins.h` | `4246e94b2dac6aa9546bab78c07a156553e4b9cb3d1a6197e99687d91099e04d` |
| `build/Debug/Infantry_Robot.elf` | `75aee4f82261fdf3d550547906399130759571127e9b25d005be66b34e5792a2` |

## 后续实物测量（2026-10-19）

上述未上板说明属于软件移植和映射修正时的历史记录。
随后复用用户已有 EmberProbe 采样连接观测运行中的 C 板，没有烧录、主动停核或复位。
实测条件、原始数据、曲线及结论边界见 [INS 实物测量记录](validation/imu_live_20261019.md)。

`beta=0.033` 时，预热后 66.428 秒静止基线的 Roll/Pitch 标准差分别为 0.02468°、0.02548°，
Yaw 线性拟合斜率为 -0.29404°/分钟，温度范围为 39.875℃～40.125℃。
这些是本次工况下的观测值，不是绝对角度精度或最佳参数证明。
后续出现用户确认的突然断电和明显降温，相应数据单独记录，不用于正常温控验收或 beta 优选。
不同 beta 实物对照尚未完成；当前继续保留 0.033，不根据降温段擅自调整温控 PID 或校准常数。
