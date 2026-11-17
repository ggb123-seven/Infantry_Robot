# 云台业务层 gimbal.c 精简验证记录（2026-11-17）

## 背景与范围

- 目标：精简 `User/module/gimbal.c` 业务层代码、简化语法，控制行为保持等价（纯重构）
- 范围：只改 `User/module/gimbal.c`；接口、结构体字段、`User/module/config.c` 参数、任务层与底层均未改动
- 说明：工作树中加入的 pitch 重力前馈（`gimbal.h` 的 `pitch_gravity_*` 字段及对应实现）由另一会话提交前改动引入，本记录的精简工作在其之上进行，未修改该功能语义

## 行数变化

| 阶段 | 行数 | 说明 |
|---|---|---|
| HEAD（原始版本） | 969 | `git show HEAD:User/module/gimbal.c` |
| 第一轮精简 | 912 | 删除前置声明、两轴循环化、合并 PID 参数同步等 |
| 另一会话加入 pitch 重力前馈 | 983 | 增加前馈枚举、校验、计算与调用链 |
| 本轮第二轮精简 | **910** | 私有辅助函数 Doxygen 压缩、前馈选择改 if、局部命令单行初始化 |

## 第二轮精简项

1. 11 个私有辅助函数的 Doxygen 块压缩为单行或三行说明，保留 Doxygen 风格与关键契约（`WrapAngle`、`ResetAngleUnwrap`、`IsPidConfigValid`、`IsAxisConfigValid`、`FrictionMagnitude`、`IsFrictionConfigValid`、`CalcYawFriction`、`IsPitchGravityConfigValid`、`CalcPitchGravity`、`RunAxis`、`ControlAxis`）
2. `RunAxis` 前馈选择 `switch` 改为顺序等价的两个 `if`（前馈种类由轴身份唯一确定，原 `default` 为空，行为等价）
3. `UpdateImu` 首帧就绪由 `if` 改三目赋值
4. `Control` 的本地 relax 命令改单行初始化
5. 上一轮保留项（删除约 20 行前置声明、两轴指针数组循环、`Gimbal_SyncPid`/`Gimbal_IsGainValid` 抽取、switch→if 归一、`StopAxis` 布尔归一、`time_backward` 具名变量）未回退

## 等价性验证（主机差分回归）

- 装置：`build/host_gimbal_diff/`，真实 `pid.c`、`filter.c`、`user_math.c`，`device/motor_rm.h` 最小 shim，确定性 xorshift32，`-O1 -ffp-contract=off -fno-fast-math`
- 输入覆盖：两轴电机反馈（空帧/帧龄/丢帧/非法单圈角）、IMU 快照（空帧/帧龄/跳变/奇异点）、命令（RELAX/SPEED/ANGLE、INCREMENT/ABSOLUTE、NaN、非法模式）、底盘状态、非法周期、在线改配置（零点/方向/使能/PID/摩擦/限位）；pitch 重力前馈在驱动配置中启用（`enabled=true`、幅值 0.5 A、相位 30°、偏置 0.1 A）
- 规模：种子 1～3 各 20000 周期 + 边界序列种子 1000，共 61361 行输出
- 结果：基线（983 行）与精简版（910 行）输出**逐字节一致**，均为 19,110,802 字节，SHA256 前缀 `441fab8714d79763`；基线连续两次运行输出一致，装置确定性成立

## 构建与静态检查

- ARM 编译（`build/Debug` 中原编译命令）exit 0、stderr 空
- ARM `-Wall -Wextra -Werror -fsyntax-only` exit 0、stderr 长度 0
- 链接（等价 build.ninja 链接参数，`build/_manual_link.py`）exit 0：ELF 1,969,116 B，**RAM 50704 B（128 KB，38.68%）、FLASH 132848 B（1 MB，12.67%）、CCMRAM 0**
- 排版（Python 按 UTF-8 字节检查）：910 行，0 处超 120 列、0 尾随空白、0 制表符、无 `} else`/`} else if` 同行、块级左花括号独行

## 门禁

- `py .auto-embedded/scripts/check.py`：HW PASS、SPEC PASS；ARCH 阶段因 PowerShell 执行策略无法加载脚本
- `powershell -NoProfile -ExecutionPolicy Bypass -File .auto-embedded/scripts/arch-check.ps1`：`PASS: 0 violations`（仅库内 mega-header 提示与缺失硬件资源表提示）
- 上一版独立复核（gate-verifier，983 行版本）已确认：函数级 token 比对 15/28 完全一致、7 组指定语义点等价、未新增判空/isfinite/范围检查/保护性 return

## 已知差异与风险

- R1（低，不可达）：`Gimbal_UpdateImu` 不再有 `switch default` 分支；若 `imu_state` 被调试器写成非法枚举值，旧版会归一为 `GIMBAL_IMU_FAULT`，新版保留原值。模块内只写 0/2/3，且非法值下 `ready=false` 仍走零输出
- R2（极低/理论）：`frame_time_us` 无条件 `now_us - age_us`，仅当 `now_us==UINT64_MAX && age_us==0` 时与旧版哨兵碰撞，实际不可达
- 本轮未新增任何判空、`isfinite`、范围检查或保护性 `return`；删除项均为重复声明、中间变量、重复校验
- 未烧录、未实物验收；静态栈检查沿用 2026-11-11 云台任务栈记录

## 证据路径

- 差分基线输出 `build/host_gimbal_diff/logs/base_out.txt`、精简版输出 `build/host_gimbal_diff/logs/cand_out.txt`
- 基线源码副本 `build/host_gimbal_diff/base/gimbal.c`、精简版副本 `build/host_gimbal_diff/cand/gimbal.c`
- 差分驱动 `build/host_gimbal_diff/driver.c`（本记录新增重力前馈启用四行）
- 独立门禁复核 `build/gate_check/REPORT.md` 及同目录日志