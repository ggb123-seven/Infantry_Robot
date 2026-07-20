#pragma once

#include "bsp/can.h"

#include <stdbool.h>
#include <stdint.h>

#define DEVICE_OK (0)
#define DEVICE_ERR (-1)
#define DEVICE_ERR_NULL (-2)
#define DEVICE_ERR_INITED (-3)
#define DEVICE_ERR_NO_DEV (-4)

typedef enum
{
    MOTOR_M2006,
    MOTOR_M3508,
    MOTOR_GM6020,
} MOTOR_RM_Module_t;

/*
 * 主机测试所需的最小电机参数：
 * - can：电机所在 CAN 总线。
 * - id：电机反馈标准帧 ID。
 * - module：RM 电机型号。
 * - reverse：安装方向反向标记。
 * - gear：减速箱换算启用标记。
 */
typedef struct
{
    BSP_CAN_t can;
    uint16_t id;
    MOTOR_RM_Module_t module;
    bool reverse;
    bool gear;
} MOTOR_RM_Param_t;

/*
 * 主机测试所需的最小 RM 电机实例：
 * - motor.header.online：设备层维护的反馈在线状态。
 * - feedback.rotor_speed：输出轴转速，单位 rpm。
 * - feedback.temp：反馈温度，单位摄氏度。
 */
typedef struct
{
    struct
    {
        struct
        {
            bool online;
        } header;
    } motor;
    struct
    {
        float rotor_speed;
        float temp;
    } feedback;
} MOTOR_RM_t;

int8_t MOTOR_RM_Register(MOTOR_RM_Param_t *param);
MOTOR_RM_t *MOTOR_RM_GetMotor(MOTOR_RM_Param_t *param);
int8_t MOTOR_RM_Update(MOTOR_RM_Param_t *param);
int8_t MOTOR_RM_SetTorqueCurrent(MOTOR_RM_Param_t *param, float current_a);
int8_t MOTOR_RM_FlushGroup(MOTOR_RM_Param_t *param);
