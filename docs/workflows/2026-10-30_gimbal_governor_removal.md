# 云台私有辅助合并与 governor 删除

## 变更目标

- 连续角展开不再作为独立 component，由 gimbal.h 中的模块私有 static inline 实现
- 遥控输入整形不再作为独立 component，由命令模块私有实现
- 删除 gimbal_governor 及其悬空残差检测、制动、限流和恢复状态机
- 删除仅服务 governor 的底盘旋转观测类型、导出接口和任务邮箱

## 保留行为

- yaw 角度模式继续使用 SMC，pitch 和速度模式继续使用 PID
- SMC 候选电流继续受云台配置的 max_current_a 限制
- 机械限位继续限制目标、速度和指向行程外侧的最终电流
- 电机反馈超时、IMU 故障、非法配置和非法控制结果继续进入状态机安全输出
- Gimbal_DumpOutput 继续作为唯一输出闸门

## 验证证据

- CMake Debug 清理后全量构建通过，产物为 build/Debug/Infantry_Robot.elf
- ARM GCC 对 gimbal.c、cmd.c、chassis.c、init.c、motor_chassis.c、motor_gimbal.c 执行 -Wall -Wextra -Werror -fsyntax-only 全部通过
- ARCH、HW、SPEC 机械门禁通过
- CMake 与 Keil 工程清单不存在 angle_unwrap.c、input_shaping.c 和 gimbal_governor.c
- ELF 符号表不存在 GimbalGovernor、InputShaping、AngleUnwrap_、Chassis_DumpGimbalFeedback 和 Cmd_ParseChassisMotion

## 验证边界

- 本次未烧录、未进行真实电机和机械台架测试
- 删除 governor 后不再提供悬空疯转检测、主动制动、低电流保持和自动恢复
- 仍保留的机械限位和故障零输出不用于识别悬空状态
