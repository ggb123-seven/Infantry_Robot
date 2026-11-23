/*
 * 底盘功率限制模块对外接口
 *
 * 按可配置功率上限预测四轮消耗功率，超限时重新分配功率并反解各轮电流
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ----------------------------------------------------------------- */
#include <stdbool.h>
#include <stdint.h>

/* Exported constants ------------------------------------------------------- */
#define POWER_WHEEL_COUNT (4U) /* 功率环覆盖的底盘轮数，须与 CHASSIS_WHEEL_COUNT 一致 */

/* Exported types ----------------------------------------------------------- */
/* 功率限制配置，初始化时复制到实例，运行中可由调试器在线修改 */
typedef struct
{
    bool enabled;           /* 是否启用功率限制 */
    float limit_w;          /* 底盘功率上限，单位：W */
    float torque_const;     /* 输出轴力矩常数，单位：N*m/A */
    float k_abs;            /* 转速损耗系数，单位：W/(rad/s) */
    float k_sq;             /* 转速平方损耗系数，单位：W/(rad/s)^2 */
    float k_cur;            /* 电流平方损耗系数，单位：W/A^2 */
    float k0;               /* 整机静态损耗，单位：W */
    float error_low_rad_s;  /* 功率分配置信度下限，单位：rad/s */
    float error_high_rad_s; /* 功率分配置信度上限，单位：rad/s */
} Power_Config_t;

/* 单轮功率环输入，由底盘模块按本周期结果填写 */
typedef struct
{
    float current_a;            /* 限幅前电流指令，单位：A */
    float measured_speed_rad_s; /* 实测输出轴角速度，单位：rad/s */
    float target_speed_rad_s;   /* 目标输出轴角速度，单位：rad/s */
    float max_current_a;        /* 该轮电流上限，单位：A */
    bool online;                /* 该轮反馈是否有效 */
} Power_WheelInput_t;

/* 功率限制实例，由底盘模块单一持有 */
typedef struct
{
    Power_Config_t config;   /* 默认配置的可调副本 */
    bool initialized;        /* 初始化成功 */
    bool config_valid;       /* 本周期配置校验结果 */
    bool limited;            /* 本周期是否发生限幅 */
    float limit_w;           /* 本周期采用的功率上限，单位：W */
    float predicted_power_w; /* 限幅前预测功率，单位：W */
    float applied_power_w;   /* 限幅后预测功率，单位：W */
} Power_t;

/* Exported functions ------------------------------------------------------- */
/**
 * @brief 复制配置并校验功率模型参数
 *
 * @param power 待初始化的功率限制实例
 * @param config 默认配置
 * @return int8_t 成功返回 0，配置非法返回 -1
 */
int8_t Power_Init(Power_t *power, const Power_Config_t *config);

/**
 * @brief 按功率上限重新分配四轮电流，原地写回
 *
 * @param power 功率限制实例
 * @param wheels 四轮功率环输入
 * @param current_a 四轮电流指令，单位：A，原地写回
 * @return 无返回值
 */
void Power_Limit(Power_t *power, const Power_WheelInput_t *wheels, float *current_a);

#ifdef __cplusplus
}
#endif
