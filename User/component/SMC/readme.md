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
    SMC_Init(&YawSMC, 20.0f, 120.0f, 0.0f, 0.001f, 21.0f, 27.0f, 25000.0f, 0.8f, 0.5f);
}
```

每个控制周期先设置目标，再执行计算，从 `u` 读取输出：

```c
YawSMC.target = target_angle;
SMC_Tick(&YawSMC, angle_now, angle_vel);
```

角度单位为度，角速度单位为度每秒，输出为 `YawSMC.u`，电流指令尺度沿用原算法。
输出应交给工程现有的通信链路；接入时检查反馈与电流输出的符号，避免正反馈。

`error_eps` 必须大于零，`p` 必须非零，`u_max` 必须非负；其余参数沿用原控制律整定值。
本次只做 C 语言迁移，保留原目标逐周期差分、死区提前返回及限幅逻辑；未新增采样周期参数。
数学函数使用 `<math.h>` 中的 `fabsf`、`powf`，GCC 链接时需要数学库 `m`。
CMake 和 Keil 工程均已加入 `.c` 文件，目前业务模块尚未调用此控制器。

原作者：李欣睿、任轩呈、梅峻炜
