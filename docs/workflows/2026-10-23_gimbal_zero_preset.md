# 云台固定零点与角度环使能修复

日期：2026-10-23

## 问题与授权

- 用户实测机械正前方对应编码器单圈角度 `97.0752°`
- 上次只在任务启动时赋值，模块仍在连续位置失效后将零点清为 `NAN`，后续人工使能被未标定分支立即撤销
- 本次用户明确授权修改 `User/module/gimbal.c`、`gimbal.h` 并同步调整 `User/task/motor_gimbal.c`
- 已确认代码存在上述路径；尚未采集实机故障时的反馈日志，不能确定实际触发的是哪种反馈异常

## 最终行为

- `gimbal.c` 的 `GIMBAL_YAW_DEFAULT_ZERO_DEG` 是默认零点唯一来源，值为 `97.0752f`
- `gimbal_yaw_zero_deg` 直接初始化为固定值，`Gimbal_Init()` 在初始化结束前采纳零点，避免首次有效反馈撤销初始化后的人工使能
- 任务不再写零点，只负责绑定邮箱和周期调度；相对仓库 HEAD，任务文件最终无内容差异
- 正常启动无需重新填写或采样零点，设置增益、目标后手动置 `gimbal_yaw_angle.enabled=true`
- 角度跟踪无效、反馈或采样间隙超过 90 ms、时间倒退、半圈歧义均零电流停机并撤销使能，清除两环历史及连续位置，但保留机械零点
- 反馈恢复后重新建立单圈相对坐标并保持禁用，用户确认机械圈数后手动重新使能，无需再次填写或采样零点
- 默认使能仍为 false；主动写入非法零点仍拒绝控制，主动改变零点仍停机并要求手动重新使能
- 可选人工采样入口保留，未新增或修改结构体布局，未写入 Flash
- 固定单圈零点不能恢复丢失圈数；恢复的新坐标从 `[0,360)` 开始，之后连续展开，0°与 360° 仍是不同目标

## 验证证据

- `python .auto-embedded/scripts/check.py`：ARCH、HW、SPEC 全部通过
- 使用 `aemb-build-cmake` 技能，`python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug` 配置和构建通过
- 产物：`build/Debug/Infantry_Robot.elf`
- ELF SHA256：`25A99198701E156AA8AFEC5B5A5AE37A7547767E78C88FFA4DB2701E426A5592`
- 主机测试链接实际 `gimbal.c`、`pid.c`、`filter.c`、`user_math.c`，采用仓库真实电机反馈类型和 CMSIS-RTOS2 声明，仅替代硬件时间和邮箱服务
- `gimbal.c` 使用主机 GCC `-std=gnu11 -Wall -Wextra -Werror` 独立编译通过，完整测试输出 `PASS assertions=15077`
- 覆盖默认零点、初始化后首次有效帧使能、500 Hz 连续十秒使能不撤销、角度有效标志失效、90 ms 精确边界和超界、采样间隙、倒序和未来时间、半圈歧义、跨编码器零点连续位置
- 覆盖异常后零点保留、反馈恢复不自动启动、无需标定即可手动使能、非法零点拒绝输出、主动改零点停机及重新初始化保留配置
- 真实 `Gimbal_Update()` 入口覆盖邮箱角度换算、缓存超时、零电流发布和恢复使能
- 同一测试对仓库 HEAD 的旧 `gimbal.c` 执行对照，在反馈角度失效后，断言 `fabsf(gimbal_yaw_zero_deg - 97.0752f) < 0.00001f` 失败，退出码 1，确认测试能够识别旧逻辑
- Allman 花括号、缺失控制块花括号、120 列、行尾空白、赋值机械拆行和 `git diff --check` 检查通过
- 人工检查中文目的注释、Doxygen 参数与返回值、集中参数说明、已有枚举状态转换、异常零输出和分层依赖；未增加 ISR 共享状态、临界区或 RTOS 对象
- 固定零点与连续位置分离的约定已通过 `task.py promote conventions decision` 沉淀，架构 spec 中的旧重标定要求同步修正
- 临时测试目录 `User/task/.gimbal_zero_recovery_verify` 在证据记录后清理

本次仅完成软件构建和主机回归，未烧录和上板验证；需使用新 ELF 对应固件复测角度环持续使能及故障恢复
