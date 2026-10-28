#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * 底盘旋转意图：无主动旋转、小陀螺、预留底盘跟随、来源不可用
 * 旋转意图不证明接地，也不直接解除云台异常保护
 */
typedef enum
{
    GIMBAL_CHASSIS_MOTION_NONE = 0,
    GIMBAL_CHASSIS_MOTION_SPIN,
    GIMBAL_CHASSIS_MOTION_FOLLOW,
    GIMBAL_CHASSIS_MOTION_UNKNOWN,
} Gimbal_ChassisMotion_t;

/*
 * 底盘任务发布、云台任务消费的完整观测：
 * - mode：本周期有效命令的旋转意图，跟随仅预留接口
 * - measured_speed_rad_s：四轮实测运动学估计的底盘转速，俯视逆时针为正，单位 rad/s
 * - requested_speed_rad_s：限幅与斜坡后的四轮目标对应的底盘转速，单位 rad/s
 * - oldest_sample_time_us、newest_sample_time_us：本组四轮源反馈时间范围，单位微秒
 * - valid：配置、四轮反馈和源时间均有效时为真，不表示已确认接地
 */
typedef struct
{
    Gimbal_ChassisMotion_t mode;
    float measured_speed_rad_s;
    float requested_speed_rad_s;
    uint64_t oldest_sample_time_us;
    uint64_t newest_sample_time_us;
    bool valid;
} Gimbal_ChassisFeedback_t;
