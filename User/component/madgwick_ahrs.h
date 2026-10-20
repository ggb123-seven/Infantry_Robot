#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
    MADGWICK_AHRS_OK = 0,
    MADGWICK_AHRS_NOT_INITIALIZED = -1,
    MADGWICK_AHRS_NULL = -2,
    MADGWICK_AHRS_INVALID_ARGUMENT = -3,
    MADGWICK_AHRS_NUMERIC = -4,
} MadgwickAHRS_Status_t;

/**
 * @brief 使用 XRobot 六轴 Madgwick 算法更新姿态，失败时保持原四元数
 *
 * @param[in,out] q 单位四元数 [w,x,y,z]，初值通常为 [1,0,0,0]
 * @param[in] gyro 三轴角速度，单位 rad/s，坐标轴须与加速度一致
 * @param[in] accel 三轴加速度，可使用 m/s² 或 g，全零时仅积分角速度
 * @param[in] beta 有限非负反馈增益，XRobot 的 BMI088 示例取 0.033
 * @param[in] dt_s 本次更新间隔，单位秒，须为有限正数
 * @return 成功返回 MADGWICK_AHRS_OK，空指针、非法参数或数值异常返回对应错误码
 */
MadgwickAHRS_Status_t MadgwickAHRS_Update(float q[4], const float gyro[3], const float accel[3],
                                       float beta, float dt_s);

#ifdef __cplusplus
}
#endif
