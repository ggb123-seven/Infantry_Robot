# yaw 直出电流线性滑模实现（2026-11-23）

## 背景与授权

- 用户选择方案 A：yaw ANGLE 由滑模直接输出电流，替代 2026-11-06 的速度域级联
- 用户明确授权修改 `User/component/SMC` 与 `User/module/gimbal.c/.h`，并确认 `SMC_t` 采用线性滑模电流版字段
- 架构对照与参数依据见 [直出电流与级联架构对照](2026-11-23_yaw_smc_direct_current_decision.md)

## SMC 组件改动

`SMC_t` 改为电流域扁平结构：

| 分类 | 字段 | 单位 | 说明 |
| --- | --- | --- | --- |
| 参数 | `C` | 1/s | 滑模面系数，必须大于零 |
| 参数 | `K` | 1/s | 线性趋近系数，不得为负 |
| 参数 | `epsilon_deg_s2` | deg/s² | 切换项幅值，不得为负 |
| 参数 | `J` | A/(deg/s²) | 等效惯量与电流换算增益，必须大于零 |
| 参数 | `surface_eps_deg_s` | deg/s | 饱和边界宽度，必须大于零 |
| 参考 | `target`、`target_dot`、`target_ddot` | deg、deg/s、deg/s² | 受限参考轨迹 |
| 观测 | `angle`、`ang_vel`、`error`、`error_last`、`target_last`、`s`、`u` | 见头文件 | 调试器观测 |

控制律：

```text
e     = angle - target
e_dot = ang_vel - target_dot
s     = e_dot + C * e
u     = J * [target_ddot - C * e_dot - K * s - epsilon * sat(s / surface_eps)]
```

- `SMC_IsConfigValid` 增加 J、epsilon 校验，保留 C、K、边界层校验
- `SMC_Tick` 在写入 `u` 前检查计算链出口，非有限时返回假且不覆盖上次电流，由模块进入故障
- `SMC_AdvanceReference` 未改动，继续输出限速、限加速度的参考轨迹
- 组件说明 `readme.md` 同步更新为先标定 J、再压制抖动、最后提高响应

## 云台模块改动

- yaw ANGLE：先推进受限参考，再由 `SMC_Tick` 生成候选电流，不再执行角度 PID 和速度 PID
- yaw 电流仍经 `max_current_a` 限幅、机械边界电流阻断和 `Gimbal_DumpOutput` 唯一输出闸门
- 机械边界的速度限幅只作用于 SPEED 模式与 pitch，滑模路径由目标角限幅加最终电流阻断处理
- pitch ANGLE 与 yaw SPEED 保持原串级 PID / 速度 PID，行为不变
- 在线调参只采纳 `C`、`K`、`epsilon_deg_s2`、`J`、`surface_eps_deg_s`，非法时 yaw 在 ANGLE 模式锁存故障
- 默认参数：C=10、K=10、epsilon=200 deg/s²、surface_eps=1 deg/s、J=7.0757e-4 A/(deg/s²)
- J 由假定 yaw 等效惯量 0.03 kg·m² 与 GM6020 力矩常数 0.74 N·m/A 推算，**上板必须用固定电流测角加速度重新标定**

## 验证

| 检查 | 方法 | 结果 |
| --- | --- | --- |
| Debug 固件 | `py .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug -j 2` | 通过，5.9 秒，ELF 1918252 字节 |
| ARM 严格语法 | 由 `compile_commands.json` 取 `gimbal.c`、`slidingmodec.c` 命令并加 `-Wall -Wextra -Werror -fsyntax-only` | 两个文件退出码 0 |
| SMC 单元测试 | 主机 GCC 链接实际 `slidingmodec.c`，覆盖参数校验、正负误差方向、前馈、制动、边界层、参考限速限加速度和非有限出口 | PASS，0 失败 |
| 云台集成测试 | 主机 GCC 链接实际 `gimbal.c`、SMC、PID、滤波和数学组件，HAL 仅用反馈类型替身隔离 | PASS，0 失败 |
| 集成覆盖 | SMC 电流路由与限幅、SPEED 模式不执行角度目标、pitch 机械目标夹紧、非法 SMC 参数只让 yaw 故障、待机清故障、轴电流上限、惯性失联后输出回零 | 全部断言通过 |
| 格式扫描 | 新增行显示宽度、行尾空白、同行花括号、`else` 同行、注释句号 | 新增行无违规 |
| 规范门禁 | `py .auto-embedded/scripts/check.py` | HW、SPEC 通过；ARCH 因本机执行策略无法加载子脚本 |
| 架构门禁 | `powershell -NoProfile -ExecutionPolicy Bypass -File .auto-embedded/scripts/arch-check.ps1` | 0 项违规 |

主机测试使用 `build/hoststubs/device/motor_rm.h` 把 `MOTOR_RM_Feedback_t` 映射为真实 `MOTOR_Feedback_t`，避免引入 HAL 头文件；测试完成后临时源码、替身和可执行文件已清理。

## 验证边界

- 未烧录、未连接探针、未运行真实电机
- J、摩擦、噪声均为合成参数，实测后才能确认响应和抖振
- 未执行 Keil 编译，本机无可用 UV4；工程文件清单未变化
- `User/component/pid.c` 在主机 `-Wall -Wextra` 下存在既有的 `-Wmaybe-uninitialized` 警告，与本次改动无关，未修改该文件

## 下一步

1. 烧录本轮 ELF，按 RELAX 零输出、反馈就绪、+5 度方向核对
2. 用固定小电流测角加速度，标定 J
3. 用角度调试入口做 5 度、10 度阶跃，记录超调、到位时间、电流峰值和静止抖动
4. 按实测结果调整 `surface_eps`、`epsilon`、`C`、`K`，并回写 `gimbal.c` 默认值
5. 稳定后放开 `speed_max` 参考限制，再做扰动与失联恢复
