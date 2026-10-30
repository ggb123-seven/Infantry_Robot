# yaw SMC 移植实施记录

## 基线与授权

- 实施分支：feature/gimbal-smc，起点 59c69d0，与 main 独立，不合并分支
- 用户确认修改 SMC、gimbal、gimbal_governor 及相关接口，优先复用 SMC_t 和原云台变量名
- 不修改 config.c，不修改设备协议、INS、CubeMX 或硬件资源
- 现有两轴 PID 配置仍从 config.c 复制到 gimbal.config，保留 Ozone 调参入口

## 调整后的方案

1. 复用 SMC_t，保持其字段布局，不新增结构体类型或字段
2. gimbal_yaw_smc_config 仅提供 SMC 参数，gimbal_yaw_smc 提供实际计算与参考轨迹观测，均复用 SMC_t
3. Control 入口对 gimbal.config 和 SMC 参数取本周期快照，集中校验与同步；反馈使用最近有效快照
4. yaw ANGLE 通过显式目标角速度、角加速度计算 SMC 电流；pitch 和 SPEED 继续 PID
5. 用连续可导的近零多项式衔接原终端滑模幂函数，避免零误差撤驱和一度边界电流跳变
6. 参考轨迹按实际 period_s 推进，限制参考速度和加速度；保护捕获及模式切换重置历史
7. governor 继续限制最终电流，SMC 只提交实际电流历史，不修改闲置 PID 积分
8. DumpOutput 按初始化、配置和轴状态统一阻断非运行输出，方向取已应用配置

## 参数与兼容性

- 保留原 SMC_t 19 个 float 字段和排列顺序，error_eps 改为近零连续过渡尺度，单位度
- SMC_Tick 显式接收参考角速度和角加速度，不再隐式使用无时间量纲的差分
- 初始试验参数 C=5、K=10、J=0.001、epsilon=0.5、q=21、p=27、error_eps=1、u_max=0.5 A
- 参数仅作为未上板的试验起点，不声明为本机已整定值；运行时有效限流取 SMC 与原轴限流的较小值
- 固定参考速度不超过 60 度每秒且服从原轴速度限制，参考加速度不超过 180 度每平方秒
- 保留固定单位边界层 sat(s)，不为可调 phi 扩展结构体
- SMC 参数在 Control 入口校验并实时同步，不恢复旧目标，不沿用旧目标导数
- Ozone 多字段写入不具备事务原子性，快照只保证同一周期不重复读取；联动调整宜在 RELAX 完成

## 实施清单

| 步骤 | 文件与接口 | 验证标准 |
| --- | --- | --- |
| SMC 数值适配 | slidingmodec.c/.h、readme.md：SMC_IsConfigValid、SMC_Tick | 正负误差、过零制动、连续导数、前馈、限流及异常输出测试 |
| 云台接入 | gimbal.c/.h、gimbal_governor.c/.h | ANGLE 使用 SMC，pitch/SPEED 回归，保护与故障零输出，配置快照 |
| 验收记录 | 本文、validation 记录、spec | Debug 构建、严格编译、ARCH/HW/SPEC、格式检查、主机集成测试 |

每个独立实现点完成后重读 AGENTS.md 与 agent.md、检查并记录证据，再提交本地 Git；不推送远端

## 验证边界

不烧录、不驱动真实电机；实机方向、负载稳定性、温升、计算耗时和 PID 对比录波需上板完成

## 第一步验证

- 主机 GCC 严格 C11 编译及实际 SMC 断言测试通过，扫描 40001 组误差，正负过渡边界电流跳变量为 1.14366412e-6 A
- 覆盖正负反馈符号、零位置误差时制动、等速参考零纠偏、解析加速度前馈、限流、非法参数及非有限计算出口
- `python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug -j 2` 通过，SMC 尚未接入时仅证明编译链接
- 新增校验仅为 `SMC_IsConfigValid` 的配置输入边界及 `SMC_Tick` 的浮点计算出口；移除原实例判空及角度死区保护性返回
