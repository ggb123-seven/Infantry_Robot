#pragma once

#include "module/gimbal.h"

// 仅供云台模块内部的反馈与控制实现共享，不作为任务层接口

// 完整一圈为 360 度，用于机械零点校验及编码器环绕
#define GIMBAL_FULL_TURN_DEG (360.0f)

/**
 * @brief 撤销机械反馈并断开连续角参考
 *
 * @param[out] axis 单轴运行数据
 * @return 无返回值
 */
void Gimbal_ClearFeedback(Gimbal_Axis_t *axis);

/**
 * @brief 清空两环历史及输出，进入指定停止状态
 *
 * @param[in,out] axis 单轴运行数据
 * @param[in] next_state RELAX、OFFLINE 或 FAULT，其他值按 FAULT 处理
 * @return 无返回值
 */
void Gimbal_StopAxis(Gimbal_Axis_t *axis, Gimbal_AxisState_t next_state);
