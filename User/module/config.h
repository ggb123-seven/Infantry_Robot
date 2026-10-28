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

#ifdef __cplusplus
}
#endif
