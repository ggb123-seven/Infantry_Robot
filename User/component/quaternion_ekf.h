#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#include "component/kalman_filter.h"

/* 六维状态由四元数与 X/Y 陀螺仪零偏组成，三维量测为归一化重力方向，沿用参考 Quaternion EKF 维数 */
#define QUATERNION_EKF_STATE_SIZE (6U)
#define QUATERNION_EKF_MEASUREMENT_SIZE (3U)
#define QUATERNION_EKF_COVARIANCE_ELEMENTS (36U)
#define QUATERNION_EKF_GAIN_ELEMENTS (18U)

typedef enum
{
    QUATERNION_EKF_STATUS_OK = 0,
    QUATERNION_EKF_STATUS_NULL = -1,
    QUATERNION_EKF_STATUS_NOT_INITIALIZED = -2,
    QUATERNION_EKF_STATUS_INVALID_ARGUMENT = -3,
    QUATERNION_EKF_STATUS_INVALID_ACCELERATION = -4,
    QUATERNION_EKF_STATUS_KALMAN = -5,
    QUATERNION_EKF_STATUS_NUMERIC = -6,
    QUATERNION_EKF_STATUS_ILLEGAL_STATE = -7,
} QuaternionEKF_Status_t;

typedef enum
{
    QUATERNION_EKF_CONVERGENCE_TRACKING = 0,
    QUATERNION_EKF_CONVERGENCE_LOCKED = 1,
} QuaternionEKF_ConvergenceState_t;

/* 配置字段沿用参考 Quaternion EKF 的噪声、渐消和滤波参数；过程噪声与量测噪声必须有限且非负，量测噪声和卡方阈值必须大于零 */
typedef struct
{
    float process_noise_quaternion;
    float process_noise_gyro_bias;
    float measurement_noise_accel;
    float fading_factor;
    float accel_lpf_time_constant_s;
    float chi_square_test_threshold;
} QuaternionEKF_Config_t;

/*
 * QuaternionEKF_t 按参考 QEKF_INS_t 的字段组织保存一个姿态滤波实例
 * Initialized 表示初始化完成；IMU_QuaternionEKF 保存六维 Kalman 内部矩阵和工作区
 * IMU_QuaternionEKF_P、IMU_QuaternionEKF_K 与 IMU_QuaternionEKF_H 保存协方差、增益和量测矩阵的诊断快照
 * ConvergeFlag 是跨周期收敛状态；StableFlag 表示当前角速度与加速度满足静止判据
 * ErrorCount 与 UpdateCount 分别统计连续异常次数和成功更新次数
 * q 为四元数 [w,x,y,z]；GyroBias 为 X/Y/Z 轴零偏，单位 rad/s，Z 轴零偏固定为零
 * Gyro 为扣除零偏后的角速度，单位 rad/s；Accel 为低通后的加速度，单位 m/s²
 * OrientationCosine 保存预测重力方向与各轴的夹角，单位 rad
 * accLPFcoef 为加速度低通时间常数，单位 s；gyro_norm 与 accl_norm 分别为角速度模长和加速度模长
 * AdaptiveGainScale 为卡方检验产生的自适应增益系数；Roll、Pitch、Yaw 与 YawTotalAngle 单位为度
 * Q1、Q2、R、lambda、dt 与 ChiSquareTestThreshold 分别保存四元数过程噪声、零偏过程噪声、量测噪声、渐消因子、更新周期和卡方阈值
 * ChiSquare 与 ChiSquare_Data 保存卡方检验矩阵和结果；YawRoundCount 与 YawAngleLast 用于累计 Yaw 跨周
 * Status 保存最近一次算法操作结果
 */
typedef struct
{
    uint8_t Initialized;
    KalmanFilter_t IMU_QuaternionEKF;
    float IMU_QuaternionEKF_P[QUATERNION_EKF_COVARIANCE_ELEMENTS];
    float IMU_QuaternionEKF_K[QUATERNION_EKF_GAIN_ELEMENTS];
    float IMU_QuaternionEKF_H[QUATERNION_EKF_GAIN_ELEMENTS];
    QuaternionEKF_ConvergenceState_t ConvergeFlag;
    uint8_t StableFlag;
    uint64_t ErrorCount;
    uint64_t UpdateCount;

    float q[4];
    float GyroBias[3];
    float Gyro[3];
    float Accel[3];
    float OrientationCosine[3];

    float accLPFcoef;
    float gyro_norm;
    float accl_norm;
    float AdaptiveGainScale;

    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;

    float Q1;
    float Q2;
    float R;
    float dt;
    mat ChiSquare;
    float ChiSquare_Data[1];
    float ChiSquareTestThreshold;
    float lambda;

    int16_t YawRoundCount;
    float YawAngleLast;
    QuaternionEKF_Status_t Status;
} QuaternionEKF_t;

/**
 * @brief 初始化四元数扩展 Kalman 姿态滤波器
 *
 * @param[in,out] ekf 姿态滤波实例，不能为空
 * @param[in] config 滤波参数，过程噪声非负、量测噪声和卡方阈值大于零、渐消因子范围为 (0,1]
 * @return 初始化状态，成功返回 QUATERNION_EKF_STATUS_OK
 */
QuaternionEKF_Status_t QuaternionEKF_Init(QuaternionEKF_t *ekf, const QuaternionEKF_Config_t *config);

/**
 * @brief 使用陀螺仪和加速度计数据更新四元数姿态估计
 *
 * @param[in,out] ekf 已初始化的姿态滤波实例
 * @param[in] gx X 轴角速度，单位 rad/s
 * @param[in] gy Y 轴角速度，单位 rad/s
 * @param[in] gz Z 轴角速度，单位 rad/s
 * @param[in] ax X 轴加速度，单位 m/s²
 * @param[in] ay Y 轴加速度，单位 m/s²
 * @param[in] az Z 轴加速度，单位 m/s²
 * @param[in] dt 本次更新间隔，单位 s，必须为有限正数
 * @return 更新状态，成功返回 QUATERNION_EKF_STATUS_OK
 */
QuaternionEKF_Status_t QuaternionEKF_Update(QuaternionEKF_t *ekf, float gx, float gy, float gz, float ax, float ay,
                                            float az, float dt);

#ifdef __cplusplus
}
#endif
