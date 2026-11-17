/*
 * 六轴 Madgwick 姿态解算接口
 *
 * 输入陀螺角速度与加速度，输出单位四元数，供惯性导航模块发布欧拉角与运动加速度
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

/* Exported types ----------------------------------------------------------- */
/* 姿态更新状态码，负值为错误 */
typedef enum
{
    MADGWICK_AHRS_OK = 0,                /* 成功 */
    MADGWICK_AHRS_NOT_INITIALIZED = -1,  /* 尚未初始化 */
    MADGWICK_AHRS_NULL = -2,             /* 空指针 */
    MADGWICK_AHRS_INVALID_ARGUMENT = -3, /* 参数非法 */
    MADGWICK_AHRS_NUMERIC = -4,          /* 数值异常 */
} MadgwickAHRS_Status_t;

/* Exported functions ------------------------------------------------------- */
/**
 * @brief 使用 XRobot 六轴 Madgwick 算法更新姿态，失败时保持原四元数
 *
 * @param q 单位四元数 [w,x,y,z]，初值通常为 [1,0,0,0]
 * @param gyro 三轴角速度，单位：rad/s，与加速度同坐标系
 * @param accel 三轴加速度，单位：m/s²，全零时仅积分角速度
 * @param beta 反馈增益，XRobot 的 BMI088 示例取 0.033
 * @param dt_s 本次更新间隔，单位：s
 * @return MadgwickAHRS_Status_t 成功返回 MADGWICK_AHRS_OK，失败返回对应错误码
 */
MadgwickAHRS_Status_t MadgwickAHRS_Update(float q[4], const float gyro[3], const float accel[3],
                                       float beta, float dt_s);

#ifdef __cplusplus
}
#endif
