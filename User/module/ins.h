/*
 * 惯性导航模块对外接口
 *
 * BMI088 采样经 Madgwick 滤波输出姿态四元数、欧拉角和运动加速度，并向下游发布统一状态
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ----------------------------------------------------------------- */
#include <stdbool.h>
#include <stdint.h>

#include "component/madgwick_ahrs.h"
#include "component/pid.h"
#include "device/bmi088.h"

/* Exported types ----------------------------------------------------------- */
/* INS 状态码，负值为错误 */
typedef enum
{
    INS_STATUS_OK = 0,                  /* 成功 */
    INS_STATUS_NULL = -1,               /* 实例为空 */
    INS_STATUS_NOT_INITIALIZED = -2,    /* 尚未初始化 */
    INS_STATUS_NOT_READY = -3,          /* 尚无有效更新 */
    INS_STATUS_INVALID_ARGUMENT = -4,   /* 参数非法 */
    INS_STATUS_BMI088 = -5,             /* BMI088 操作失败 */
    INS_STATUS_AHRS = -6,               /* 姿态滤波失败 */
    INS_STATUS_PID = -7,                /* 温控 PID 操作失败 */
    INS_STATUS_HEATER = -8,             /* 加热器操作失败 */
    INS_STATUS_ILLEGAL_STATE = -9,      /* 非法状态 */
    INS_STATUS_NUMERIC = -10,           /* 数值非法 */
} INS_Status_t;

/* INS 模块生命周期状态 */
typedef enum
{
    INS_MODULE_STATE_UNINITIALIZED = 0, /* 未初始化 */
    INS_MODULE_STATE_INITIALIZING,      /* 初始化中 */
    INS_MODULE_STATE_RUNNING,           /* 正常运行 */
    INS_MODULE_STATE_FAULT,             /* 故障 */
} INS_ModuleState_t;

/* 对外发布的 INS 状态，沿用参考工程 INS_t 的字段组织 */
typedef struct
{
    bool valid;                        /* 本次完整更新成功 */
    float q[4];                        /* 姿态四元数 [w,x,y,z] */
    float Gyro[3];                     /* 机体系三轴角速度，单位：rad/s */
    float Accel[3];                    /* 机体系三轴加速度，单位：m/s² */
    float MotionAccel_b[3];            /* 机体系运动加速度，单位：m/s² */
    float MotionAccel_n[3];            /* 导航系运动加速度，单位：m/s² */
    float xn[3];                       /* 机体系 X 轴在导航系的单位方向向量 */
    float yn[3];                       /* 机体系 Y 轴在导航系的单位方向向量 */
    float zn[3];                       /* 机体系 Z 轴在导航系的单位方向向量 */
    float atanxz;                      /* 重力在 XZ 平面的倾角，单位：度 */
    float atanyz;                      /* 重力在 YZ 平面的倾角，单位：度 */
    float Roll;                        /* 横滚角，绕 X 轴，单位：度 */
    float Pitch;                       /* 俯仰角，绕 Y 轴，单位：度 */
    float Yaw;                         /* 偏航角，绕 Z 轴，单位：度 */
    float YawTotalAngle;               /* 按相邻偏航最短差展开的累计偏航角，单位：度 */
    float Temperature;                 /* BMI088 温度，单位：摄氏度 */
} INS_State_t;

/* INS 实例，保存设备状态、温控、安装修正和对外数据 */
typedef struct
{
    INS_ModuleState_t module_state;    /* 模块生命周期状态 */
    INS_Status_t status;               /* 最近一次操作结果 */
    BMI088_Status_t bmi088_status;     /* BMI088 驱动状态 */
    MadgwickAHRS_Status_t ahrs_status; /* 姿态滤波器状态 */
    int8_t pid_status;                 /* 温控 PID 状态 */
    int8_t heat_status;                /* 加热器状态 */

    float ahrs_beta;                   /* 姿态校正增益 */
    KPID_t temperature_controller;     /* 温控 PID 控制器状态 */
    INS_State_t state;                 /* 对外发布的状态数据 */

    float gyro_scale[3];               /* 三轴陀螺仪比例系数，无量纲 */
    float correction_yaw_deg;          /* 安装误差偏航角，单位：度 */
    float correction_pitch_deg;        /* 安装误差俯仰角，单位：度 */
    float correction_roll_deg;         /* 安装误差横滚角，单位：度 */
    float correction_matrix[9];        /* 行优先排列的 3×3 安装修正矩阵 */
    float correction_angle_last[3];    /* 上次生成修正矩阵的角度，单位：度 */
    float MotionAccelLPF;              /* 运动加速度低通时间常数，单位：s */
    float temperature_elapsed_s;       /* 温控周期累计时间，单位：s */

    uint32_t UpdateCount;              /* 成功更新的累计次数 */
    uint8_t temperature_control_divider; /* 温控更新分频，将温控降至 500 Hz */
} INS_t;

/* Exported functions ------------------------------------------------------- */
/**
 * @brief 初始化 BMI088、姿态滤波器和加热温控
 *
 * @param ins INS 模块实例
 * @return INS_Status_t 成功返回 INS_STATUS_OK，初始化失败时模块进入故障状态
 */
INS_Status_t INS_Init(INS_t *ins);

/**
 * @brief 读取 IMU 数据并更新姿态、坐标变换、运动加速度和温控
 *
 * @param ins 已初始化的 INS 模块实例
 * @param dt_s 本次采集间隔，单位：s，须为有限正数
 * @return INS_Status_t 成功返回 INS_STATUS_OK，失败时关闭加热器并进入故障状态
 */
INS_Status_t INS_Update(INS_t *ins, float dt_s);

/**
 * @brief 获取最近一次有效的 INS 对外状态
 *
 * @param ins 已初始化的 INS 模块实例
 * @param state 状态输出地址
 * @return INS_Status_t 状态有效时返回 INS_STATUS_OK，尚无有效更新返回 INS_STATUS_NOT_READY，故障时返回最近故障状态
 */
INS_Status_t INS_GetState(const INS_t *ins, INS_State_t *state);

#ifdef __cplusplus
}
#endif
