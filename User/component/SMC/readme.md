# 【2026-复旦大学-星云EGA】 云台 yaw 角度滑模外环

## C 语言使用说明

实现文件为 `slidingmodec.c`，类型为 `SMC_t`，参数和运行观测沿用扁平结构
原私有状态在 C 中可见，由控制器维护；每个实例使用前调用一次 `SMC_Init`，初始化时会清零全部历史状态

在业务初始化函数中设置参数，先完成参数快照校验，再复制到运行实例：

```c
#include "component/SMC/slidingmodec.h"

SMC_t YawSMC;
SMC_t YawConfig =
{
    .C = 5.0f,
    .K = 10.0f,
    .error_eps = 1.0f,
    .q = 21.0f,
    .p = 27.0f,
    .speed_max_deg_s = 60.0f,
    .surface_eps_deg_s = 0.5f,
};

void YawControl_Init(void)
{
    if (SMC_IsConfigValid(&YawConfig))
    {
        SMC_Init(&YawSMC, &YawConfig, 0.0f);
    }
}
```

角度模式每个控制周期先推进受限参考，再执行滑模外环，从 `speed_target_deg_s` 读取速度目标：

```c
SMC_AdvanceReference(&YawSMC, target_angle, target_speed, 60.0f, 180.0f, 5.0f, period_s);
bool computed = SMC_Tick(&YawSMC, angle_now, angle_vel);
float speed_target_deg_s = YawSMC.speed_target_deg_s;
```

角度单位为 deg，角速度单位为 deg/s，角加速度单位为 deg/s²，速度目标上限和滑模面边界宽度也使用 deg/s
`target_speed` 为同一周期的角速度前馈，参考轨迹由 `SMC_AdvanceReference` 按实际周期限速和限加速度
`computed` 为假时必须进入故障并由模块输出闸门阻断输出，禁止使用上周期速度目标或电流
滑模输出应接入现有速度 PID；输出电流继续由速度 PID 的限幅和云台输出闸门处理

初始化和每周期在线调参边界先调用 `SMC_IsConfigValid`，计算函数信任已校验的参数与非空实例。
`error_eps` 表示角误差近零连续过渡尺度，必须大于零，不表示角度误差死区
过渡区使用三次多项式，与外侧幂函数同时匹配值和导数；零误差但存在速度误差时仍生成制动速度目标
`target_dot`、`target_ddot` 保存有物理单位的本周期参考导数，`speed_target_deg_s` 保存本周期速度目标
示例参数是本工程移植的未上板试验起点，不能视为原作者参数或已整定结果。
数学函数使用 `<math.h>` 中的 `fabsf`、`powf`，GCC 链接时需要数学库 `m`
CMake 和 Keil 工程均已加入 `.c` 文件；yaw ANGLE 已接入速度 PID，pitch 和 SPEED 保留原 PID

原作者：李欣睿、任轩呈、梅峻炜
