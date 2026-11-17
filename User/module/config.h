/*
 * 整机配置模块对外接口
 *
 * 汇总各业务模块的只读默认配置与标定系数，模块初始化时复制到各自实例
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ----------------------------------------------------------------- */
#include "module/gimbal.h"
#include "module/chassis.h"
#include "module/shoot.h"
#include "module/feeder.h"

/* Exported constants ------------------------------------------------------- */
/* 各业务模块的只读默认配置，参数说明见定义处，模块初始化时复制到实例 */
extern const Chassis_Config_t chassis_config; /* 底盘默认配置 */
extern const Gimbal_Config_t gimbal_config;   /* 云台默认配置 */
extern const Shoot_Config_t shoot_config;     /* 摩擦轮默认配置 */
extern const Feeder_Config_t feeder_config;   /* 拨弹默认配置 */

/* 平均输出轴轮速到车体转速的标定系数，零表示尚未标定 */
extern const float chassis_yaw_rad_s_per_rpm; /* 每 rpm 对应的车体转速，单位：rad/s */

#ifdef __cplusplus
}
#endif
