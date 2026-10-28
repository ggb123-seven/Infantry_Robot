#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "module/gimbal.h"
#include "module/chassis.h"

/* 整机底盘与云台只读默认配置，参数说明见定义处，模块初始化时复制到各自实例 */
extern const Chassis_Config_t chassis_config;
extern const Gimbal_Config_t gimbal_config;

// 平均输出轴轮速到车体转速的标定系数，单位为每 rpm 对应的 rad/s，零表示尚未标定
extern const float chassis_yaw_rad_s_per_rpm;

#ifdef __cplusplus
}
#endif
