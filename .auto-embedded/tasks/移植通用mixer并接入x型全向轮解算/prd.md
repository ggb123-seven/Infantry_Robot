# 移植通用Mixer并接入X型全向轮解算

## 需求 / 验收标准
- 从 `F:\RM\未命名文件夹\User\component\mixer.c/.h` 移植通用 Mixer 接口到当前工程
- 保留多底盘模式选择能力，使 Chassis 不直接依赖某一套运动学公式
- 补全 `MIXER_OMNICROSS`，支持 `vx`、`vy` 平动和 `wz` 转动的任意组合
- 当前轮序按 ID1 左前、ID2 左后、ID3 右后、ID4 右前
- 四轮目标任一路超限时整体等比例缩放，保持平动和转动比例
- 空指针、非法长度、非法模式、NaN/Inf 或非法机械参数均返回错误并清零全部输出
- 零输入、纯前进、纯横移、纯逆时针旋转、组合输入和饱和输入的测试结果符合公式与轮序
- 不破坏现有四路速度 PID、CAN 邮箱、故障检测和默认零输出安全链

## 约束
- 遵守 `task -> module -> component/device -> bsp -> HAL` 依赖方向
- Mixer 位于 `User/component`，不得依赖 RTOS、HAL、Device、Module 或 Task
- 本次不修改 BSP、Device、CubeMX、`.ioc`、CAN ID 和硬件资源锁
- `MIXER_OMNIPLUS` 本次不启用，不使用未经验证的轮序和符号
- 临时有效轮半径使用 `0.076 m`，集中配置并标明待实测替换
- 旋转中心到轮心距离尚未确认，进入实现前必须确定采用物理参数还是归一化旋转尺度
- 新增或修改注释使用简体中文 UTF-8，末尾不使用句号，代码采用 Allman 风格和 120 列限制

## 方案决策

- 首版沿用参考工程的归一化 Mixer 语义：`vx`、`vy`、`wz` 都是无量纲控制分量
- `Mixer_Apply()` 先按所选底盘模式计算相对轮速，再按最大绝对值整体归一化，最后乘 `scale_rpm`
- 有效轮半径和轮心距不进入首版 Mixer 接口，因此当前不使用此前暂定的 `0.076 m`
- 真实 `m/s`、`rad/s` 到归一化控制量的换算属于上层命令或后续机械参数配置，不与运动学模式分发耦合

## 实施清单

- [x] `review:true` 新增 L4 Component 文件 `User/component/mixer.h`，移植并整理 `Mixer_Mode_t`、`Mixer_t`、`Mixer_Init()`、`Mixer_Apply()` 公共接口；验证公共头不依赖 Module、Task、Device、HAL 或 RTOS
- [x] `review:true` 新增 L4 Component 文件 `User/component/mixer.c`，移植已有底盘模式、修复缺失 `break`、补全 `MIXER_OMNICROSS`，所有失败路径整组清零；验证零输入、纯平动、纯转动、组合、饱和、空指针、错误长度和错误模式
- [x] `review:true` 修改 L5 Module 文件 `User/module/chassis.h/.c`，在底盘状态中持有 `Mixer_t`，将输入契约扩展为归一化 `MoveVector_t`，在四路速度 PID 前调用 Mixer 并检查返回值；验证 Mixer 失败、控制禁用或非法输入时四路电流保持为零
- [x] `review:true` 修改 L6 Task 文件 `User/task/ozone_debug.h/.c` 与必要的 `User/task/motor_chassis.c`，把 Ozone 调试入口从四路直接 rpm 调整为 `vx`、`vy`、`wz` 和 `scale_rpm`，保留总使能和 PID 在线参数；验证任务只负责编排，不嵌入运动学公式
- [x] `review:true` 修改 `CMakeLists.txt` 纳入 `User/component/mixer.c`，不修改 BSP、Device、CubeMX、`.ioc` 和 CAN 通信路径；验证 Debug CMake 完整构建
- [x] `review:false` 逐文件执行 `AGENTS.md`、`agent.md` 复读与格式检查，最终运行运动学主机测试、`python .auto-embedded/scripts/check.py`、`.auto-embedded/scripts/arch-check.ps1` 和 Debug 构建，记录验证证据
- [ ] `review:true` 上板前按 ID1 左前、ID2 左后、ID3 右后、ID4 右前做架空低速点动，依次验证纯 `+vx`、纯 `+vy`、纯 `+wz`；任何轮位或符号不符时只修正集中映射，不修改 PID 符号
