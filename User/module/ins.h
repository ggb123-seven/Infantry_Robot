#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

#include "component/pid.h"
#include "component/quaternion_ekf.h"
#include "device/bmi088.h"

typedef enum
{
    INS_STATUS_OK = 0,
    INS_STATUS_NULL = -1,
    INS_STATUS_NOT_INITIALIZED = -2,
    INS_STATUS_NOT_READY = -3,
    INS_STATUS_INVALID_ARGUMENT = -4,
    INS_STATUS_BMI088 = -5,
    INS_STATUS_QUATERNION_EKF = -6,
    INS_STATUS_PID = -7,
    INS_STATUS_HEATER = -8,
    INS_STATUS_ILLEGAL_STATE = -9,
    INS_STATUS_NUMERIC = -10,
} INS_Status_t;

typedef enum
{
    INS_MODULE_STATE_UNINITIALIZED = 0,
    INS_MODULE_STATE_INITIALIZING,
    INS_MODULE_STATE_RUNNING,
    INS_MODULE_STATE_FAULT,
} INS_ModuleState_t;

/*
 * INS 对外状态沿用参考工程 INS_t 的字段组织
 * valid 表示本次完整更新已成功；q 为姿态四元数 [w,x,y,z]
 * Gyro 为修正后的三轴角速度，单位 rad/s；Accel 为修正后的三轴加速度，单位 m/s²
 * MotionAccel_b 与 MotionAccel_n 分别为机体系和导航系运动加速度，单位 m/s²
 * xn、yn、zn 为机体系三轴在导航系中的单位方向向量
 * atanxz 与 atanyz 为重力加速度在 XZ、YZ 平面的倾角，单位度
 * Roll、Pitch、Yaw 为姿态角，YawTotalAngle 为累计偏航角，单位度
 * Temperature 为 BMI088 温度，单位摄氏度
 */
typedef struct
{
    bool valid;
    float q[4];
    float Gyro[3];
    float Accel[3];
    float MotionAccel_b[3];
    float MotionAccel_n[3];
    float xn[3];
    float yn[3];
    float zn[3];
    float atanxz;
    float atanyz;
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
    float Temperature;
} INS_State_t;

/*
 * INS_t 保存设备滤波组合、温控状态、安装修正参数和诊断结果
 * module_state 为模块生命周期状态；status 为最近一次操作结果
 * bmi088_status、quaternion_ekf_status、pid_status 与 heat_status 保存对应底层操作状态
 * quaternion_ekf 与 temperature_controller 分别保存姿态滤波实例和温控 PID 状态
 * state 保存对外发布数据；gyro_scale 为三轴无量纲陀螺仪比例系数
 * correction_yaw_deg、correction_pitch_deg 与 correction_roll_deg 为安装误差角，单位度
 * correction_matrix 为行优先排列的 3×3 安装修正矩阵；correction_angle_last 保存上次生成矩阵的角度
 * MotionAccelLPF 为运动加速度低通时间常数，temperature_elapsed_s 为温控周期累计时间，单位秒
 * UpdateCount 统计成功更新次数；temperature_control_divider 用于将温控更新降至 500 Hz
 */
typedef struct
{
    INS_ModuleState_t module_state;
    INS_Status_t status;
    BMI088_Status_t bmi088_status;
    QuaternionEKF_Status_t quaternion_ekf_status;
    int8_t pid_status;
    int8_t heat_status;

    QuaternionEKF_t quaternion_ekf;
    KPID_t temperature_controller;
    INS_State_t state;

    float gyro_scale[3];
    float correction_yaw_deg;
    float correction_pitch_deg;
    float correction_roll_deg;
    float correction_matrix[9];
    float correction_angle_last[3];
    float MotionAccelLPF;
    float temperature_elapsed_s;

    uint32_t UpdateCount;
    uint8_t temperature_control_divider;
} INS_t;

/**
 * @brief 初始化 BMI088、姿态滤波器和加热温控
 *
 * @param[in,out] ins INS 模块实例，不能为空
 * @return 初始化成功返回 INS_STATUS_OK，设备、滤波器、PID 或加热器初始化失败时进入故障状态
 */
INS_Status_t INS_Init(INS_t *ins);

/**
 * @brief 读取 IMU 数据并更新姿态、坐标变换、运动加速度和温控
 *
 * @param[in,out] ins 已初始化的 INS 模块实例
 * @param[in] dt_s 本次采集间隔，单位 s，必须为有限正数
 * @return 更新成功返回 INS_STATUS_OK，设备或算法失败时关闭加热器并进入故障状态
 */
INS_Status_t INS_Update(INS_t *ins, float dt_s);

/**
 * @brief 获取最近一次有效的 INS 对外状态
 *
 * @param[in] ins 已初始化的 INS 模块实例
 * @param[out] state 状态输出地址，不能为空
 * @return 状态有效时返回 INS_STATUS_OK，尚无有效更新时返回 INS_STATUS_NOT_READY，故障时返回最近故障状态
 */
INS_Status_t INS_GetState(const INS_t *ins, INS_State_t *state);

#ifdef __cplusplus
}
#endif
