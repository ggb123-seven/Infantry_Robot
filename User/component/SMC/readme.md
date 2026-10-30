# 【2026-复旦大学-星云EGA】 云台yaw轴滑模控制器

## C 语言使用说明

实现文件为 `slidingmodec.c`，类型为 `SMC_t`，保留原 C++ 类的全部 19 个 `float` 字段及其名称。
原私有状态在 C 中可见，由控制器维护；每个实例使用前调用一次 `SMC_Init`，初始化时会清零全部历史状态。

在业务初始化函数中设置参数，参数顺序与原构造函数一致：

```c
#include "component/SMC/slidingmodec.h"

SMC_t YawSMC;

void YawControl_Init(void)
{
SMC_Init(&YawSMC, 5.0f, 10.0f, 0.0f, 1.0f, 21.0f, 27.0f, 0.5f, 0.001f, 0.5f);
}
```

每个控制周期先设置目标，再执行计算，从 `u` 读取输出：

```c
YawSMC.target = target_angle;
bool computed = SMC_Tick(&YawSMC, angle_now, angle_vel, target_speed, target_acceleration);
```

角度单位为度，角速度单位为度每秒，角加速度单位为度每平方秒，`YawSMC.u` 的单位为安培。
`target_speed` 与 `target_acceleration` 必须来自同一时刻的受限参考轨迹，由调用方按实际周期生成。
本分支通过 `SMC_AdvanceReference` 按实际周期生成有速度和加速度约束的参考，再由 `SMC_Tick` 计算电流。
`computed` 为假时必须进入故障并由模块输出闸门阻断输出，禁止使用上周期电流。
输出应交给工程现有的通信链路；接入时检查反馈与电流输出的符号，避免正反馈。

初始化和每周期在线调参边界先调用 `SMC_IsConfigValid`，计算函数信任已校验的参数与非空实例。
`error_eps` 现表示近零连续过渡尺度，必须大于零，不再表示角度误差死区。
过渡区使用三次多项式，与外侧幂函数同时匹配值和导数；零误差但存在速度误差时仍输出制动电流。
保留全部 19 个字段及顺序，`target_dot`、`target_ddot` 现保存有物理单位的本周期参考导数。
示例参数是本工程移植的未上板试验起点，不能视为原作者参数或已整定结果。
数学函数使用 `<math.h>` 中的 `fabsf`、`powf`，GCC 链接时需要数学库 `m`。
CMake 和 Keil 工程均已加入 `.c` 文件；yaw ANGLE 已接入，pitch 和 SPEED 保留 PID，验证见 `docs/workflows/2026-10-30_yaw_smc_implementation.md`。

原作者：李欣睿、任轩呈、梅峻炜
