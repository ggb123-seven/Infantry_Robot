#include "component/quaternion_ekf.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

/* 状态矩阵维数与卡方、自适应、稳定和角度换算边界沿用参考工程 QuaternionEKF.c */
#define QUATERNION_EKF_CONVERGENCE_RATIO (0.5F)
#define QUATERNION_EKF_ADAPTIVE_START_RATIO (0.1F)
#define QUATERNION_EKF_ADAPTIVE_RANGE_RATIO (0.9F)
#define QUATERNION_EKF_MAX_RESIDUAL_ERRORS (50U)
#define QUATERNION_EKF_MAX_BIAS_VARIANCE (10000.0F)
#define QUATERNION_EKF_STABLE_GYRO_LIMIT_RAD_S (0.3F)
#define QUATERNION_EKF_STABLE_ACCEL_CENTER_MPS2 (9.8F)
#define QUATERNION_EKF_STABLE_ACCEL_TOLERANCE_MPS2 (0.5F)
#define QUATERNION_EKF_MAX_BIAS_STEP_RAD_S (1.0e-2F)
#define QUATERNION_EKF_HALF_PI_RAD (1.5707963F)
#define QUATERNION_EKF_RAD_TO_DEG (57.295779513F)
#define QUATERNION_EKF_YAW_HALF_TURN_DEG (180.0F)

/* 参考工程使用单位阵初始化 F，P 的四元数对角为 100000、零偏对角为 100、其余元素为 0.1 */
static const float quaternion_ekf_initial_f[QUATERNION_EKF_COVARIANCE_ELEMENTS] =
{
    1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F,
};

static const float quaternion_ekf_initial_p[QUATERNION_EKF_COVARIANCE_ELEMENTS] =
{
    100000.0F, 0.1F, 0.1F, 0.1F, 0.1F, 0.1F,
    0.1F, 100000.0F, 0.1F, 0.1F, 0.1F, 0.1F,
    0.1F, 0.1F, 100000.0F, 0.1F, 0.1F, 0.1F,
    0.1F, 0.1F, 0.1F, 100000.0F, 0.1F, 0.1F,
    0.1F, 0.1F, 0.1F, 0.1F, 100.0F, 0.1F,
    0.1F, 0.1F, 0.1F, 0.1F, 0.1F, 100.0F,
};

static bool QuaternionEKF_ConfigIsValid(const QuaternionEKF_Config_t *config);
static QuaternionEKF_t *QuaternionEKF_FromKalman(KalmanFilter_t *kf);
static QuaternionEKF_Status_t QuaternionEKF_SetStatus(QuaternionEKF_t *ekf, QuaternionEKF_Status_t status);
static bool QuaternionEKF_RecordMatrixStatus(QuaternionEKF_t *ekf, KalmanFilter_t *kf, arm_status status);
static bool QuaternionEKF_Normalize(float *vector, uint8_t length);
static bool QuaternionEKF_UpdateConvergence(QuaternionEKF_t *ekf, KalmanFilter_t *kf, bool *apply_measurement);
static void QuaternionEKF_Observe(KalmanFilter_t *kf);
static void QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf);
static void QuaternionEKF_SetH(KalmanFilter_t *kf);
static void QuaternionEKF_xhatUpdate(KalmanFilter_t *kf);

/**
 * @brief 检查四元数 EKF 配置参数是否在数值安全范围内
 *
 * @param[in] config 待检查的滤波配置
 * @return 所有配置有效时返回 true
 */
static bool QuaternionEKF_ConfigIsValid(const QuaternionEKF_Config_t *config)
{
    if (config == NULL || !isfinite(config->process_noise_quaternion) || !isfinite(config->process_noise_gyro_bias) ||
        !isfinite(config->measurement_noise_accel) || !isfinite(config->fading_factor) ||
        !isfinite(config->accel_lpf_time_constant_s) || !isfinite(config->chi_square_test_threshold))
    {
        return false;
    }

    return config->process_noise_quaternion >= 0.0F && config->process_noise_gyro_bias >= 0.0F &&
           config->measurement_noise_accel > 0.0F && config->fading_factor > 0.0F && config->fading_factor <= 1.0F &&
           config->accel_lpf_time_constant_s >= 0.0F && config->chi_square_test_threshold > 0.0F;
}

/**
 * @brief 根据 Kalman 成员地址获取所属四元数 EKF 实例
 *
 * @param[in] kf 回调收到的 Kalman 滤波器对象
 * @return 所属姿态滤波实例
 */
static QuaternionEKF_t *QuaternionEKF_FromKalman(KalmanFilter_t *kf)
{
    return (QuaternionEKF_t *)((uint8_t *)kf - offsetof(QuaternionEKF_t, IMU_QuaternionEKF));
}

/**
 * @brief 保存四元数 EKF 当前状态
 *
 * @param[in,out] ekf 姿态滤波实例
 * @param[in] status 需要保存的算法状态
 * @return 写入的算法状态
 */
static QuaternionEKF_Status_t QuaternionEKF_SetStatus(QuaternionEKF_t *ekf, QuaternionEKF_Status_t status)
{
    if (ekf != NULL)
    {
        ekf->Status = status;
    }
    return status;
}

/**
 * @brief 保存矩阵运算状态并同步 Kalman 与 EKF 错误状态
 *
 * @param[in,out] ekf 姿态滤波实例
 * @param[in,out] kf Kalman 滤波器对象
 * @param[in] status CMSIS-DSP 矩阵运算状态
 * @return 矩阵运算成功时返回 true
 */
static bool QuaternionEKF_RecordMatrixStatus(QuaternionEKF_t *ekf, KalmanFilter_t *kf, arm_status status)
{
    kf->MatStatus = (int8_t)status;
    if (status == ARM_MATH_SUCCESS)
    {
        kf->Status = KALMAN_FILTER_STATUS_OK;
        return true;
    }

    if (status == ARM_MATH_SINGULAR)
    {
        kf->Status = KALMAN_FILTER_STATUS_SINGULAR;
    }
    else if (status == ARM_MATH_SIZE_MISMATCH)
    {
        kf->Status = KALMAN_FILTER_STATUS_DIMENSION;
    }
    else
    {
        kf->Status = KALMAN_FILTER_STATUS_MATRIX;
    }

    QuaternionEKF_SetStatus(ekf, QUATERNION_EKF_STATUS_KALMAN);
    return false;
}

/**
 * @brief 归一化向量并拒绝非有限或模长过小的数据
 *
 * @param[in,out] vector 待归一化向量
 * @param[in] length 向量元素数量
 * @return 向量有效且归一化成功时返回 true
 */
static bool QuaternionEKF_Normalize(float *vector, uint8_t length)
{
    float norm_squared = 0.0F;
    for (uint8_t index = 0U; index < length; index++)
    {
        if (!isfinite(vector[index]))
        {
            return false;
        }
        norm_squared += vector[index] * vector[index];
    }

    if (!isfinite(norm_squared) || norm_squared <= FLT_MIN)
    {
        return false;
    }

    const float inverse_norm = 1.0F / sqrtf(norm_squared);
    if (!isfinite(inverse_norm))
    {
        return false;
    }
    for (uint8_t index = 0U; index < length; index++)
    {
        vector[index] *= inverse_norm;
    }
    return true;
}

/**
 * @brief 按参考算法状态转移规则决定加速度量测是否参与本周期更新
 *
 * @param[in,out] ekf 姿态滤波实例
 * @param[in,out] kf Kalman 滤波器对象
 * @param[out] apply_measurement 是否应用加速度量测
 * @return 状态合法且转换完成时返回 true
 */
static bool QuaternionEKF_UpdateConvergence(QuaternionEKF_t *ekf, KalmanFilter_t *kf, bool *apply_measurement)
{
    const float chi_square = ekf->ChiSquare_Data[0];
    if (!isfinite(chi_square) || chi_square < 0.0F)
    {
        QuaternionEKF_SetStatus(ekf, QUATERNION_EKF_STATUS_NUMERIC);
        kf->Status = KALMAN_FILTER_STATUS_NUMERIC;
        return false;
    }

    if (chi_square < QUATERNION_EKF_CONVERGENCE_RATIO * ekf->ChiSquareTestThreshold)
    {
        ekf->ConvergeFlag = QUATERNION_EKF_CONVERGENCE_LOCKED;
    }

    switch (ekf->ConvergeFlag)
    {
        case QUATERNION_EKF_CONVERGENCE_TRACKING:
            ekf->ErrorCount = 0U;
            ekf->AdaptiveGainScale = 1.0F;
            kf->SkipEq5 = 0U;
            *apply_measurement = true;
            return true;

        case QUATERNION_EKF_CONVERGENCE_LOCKED:
            if (chi_square > ekf->ChiSquareTestThreshold)
            {
                if (ekf->StableFlag != 0U)
                {
                    ekf->ErrorCount++;
                }
                else
                {
                    ekf->ErrorCount = 0U;
                }

                if (ekf->ErrorCount > QUATERNION_EKF_MAX_RESIDUAL_ERRORS)
                {
                    ekf->ConvergeFlag = QUATERNION_EKF_CONVERGENCE_TRACKING;
                    ekf->ErrorCount = 0U;
                    ekf->AdaptiveGainScale = 1.0F;
                    kf->SkipEq5 = 0U;
                    *apply_measurement = true;
                    return true;
                }

                kf->SkipEq5 = 1U;
                *apply_measurement = false;
                return true;
            }

            ekf->ErrorCount = 0U;
            if (chi_square > QUATERNION_EKF_ADAPTIVE_START_RATIO * ekf->ChiSquareTestThreshold)
            {
                ekf->AdaptiveGainScale = (ekf->ChiSquareTestThreshold - chi_square) /
                                         (QUATERNION_EKF_ADAPTIVE_RANGE_RATIO * ekf->ChiSquareTestThreshold);
            }
            else
            {
                ekf->AdaptiveGainScale = 1.0F;
            }
            kf->SkipEq5 = 0U;
            *apply_measurement = true;
            return true;

        default:
            QuaternionEKF_SetStatus(ekf, QUATERNION_EKF_STATUS_ILLEGAL_STATE);
            kf->Status = KALMAN_FILTER_STATUS_INVALID_ARGUMENT;
            return false;
    }
}

/**
 * @brief 保存当前 Kalman 矩阵快照供调试观察
 *
 * @param[in,out] kf Kalman 滤波器对象
 * @return 无
 */
static void QuaternionEKF_Observe(KalmanFilter_t *kf)
{
    QuaternionEKF_t *ekf = QuaternionEKF_FromKalman(kf);
    memcpy(ekf->IMU_QuaternionEKF_P, kf->P_data, sizeof(ekf->IMU_QuaternionEKF_P));
    memcpy(ekf->IMU_QuaternionEKF_K, kf->K_data, sizeof(ekf->IMU_QuaternionEKF_K));
    memcpy(ekf->IMU_QuaternionEKF_H, kf->H_data, sizeof(ekf->IMU_QuaternionEKF_H));
}

/**
 * @brief 线性化状态转移矩阵并限制陀螺仪零偏协方差
 *
 * @param[in,out] kf Kalman 滤波器对象
 * @return 无，错误状态通过滤波器与 EKF 对象保存
 */
static void QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf)
{
    QuaternionEKF_t *ekf = QuaternionEKF_FromKalman(kf);
    float *quaternion = kf->xhatminus_data;
    if (!QuaternionEKF_Normalize(quaternion, 4U))
    {
        QuaternionEKF_SetStatus(ekf, QUATERNION_EKF_STATUS_NUMERIC);
        kf->Status = KALMAN_FILTER_STATUS_NUMERIC;
        return;
    }

    const float q0 = quaternion[0];
    const float q1 = quaternion[1];
    const float q2 = quaternion[2];
    const float q3 = quaternion[3];

    // 使用本周期角速度和实测 dt 更新四元数到零偏的线性化项
    kf->F_data[4] = q1 * ekf->dt * 0.5F;
    kf->F_data[5] = q2 * ekf->dt * 0.5F;
    kf->F_data[10] = -q0 * ekf->dt * 0.5F;
    kf->F_data[11] = q3 * ekf->dt * 0.5F;
    kf->F_data[16] = -q3 * ekf->dt * 0.5F;
    kf->F_data[17] = -q0 * ekf->dt * 0.5F;
    kf->F_data[22] = q2 * ekf->dt * 0.5F;
    kf->F_data[23] = -q1 * ekf->dt * 0.5F;

    // 按渐消因子放宽 X/Y 零偏估计方差并限制其最大值
    kf->P_data[28] /= ekf->lambda;
    kf->P_data[35] /= ekf->lambda;
    if (!isfinite(kf->P_data[28]) || !isfinite(kf->P_data[35]))
    {
        QuaternionEKF_SetStatus(ekf, QUATERNION_EKF_STATUS_NUMERIC);
        kf->Status = KALMAN_FILTER_STATUS_NUMERIC;
        return;
    }
    if (kf->P_data[28] > QUATERNION_EKF_MAX_BIAS_VARIANCE)
    {
        kf->P_data[28] = QUATERNION_EKF_MAX_BIAS_VARIANCE;
    }
    if (kf->P_data[35] > QUATERNION_EKF_MAX_BIAS_VARIANCE)
    {
        kf->P_data[35] = QUATERNION_EKF_MAX_BIAS_VARIANCE;
    }
}

/**
 * @brief 根据先验四元数生成归一化重力方向量测矩阵
 *
 * @param[in,out] kf Kalman 滤波器对象
 * @return 无，错误状态通过滤波器与 EKF 对象保存
 */
static void QuaternionEKF_SetH(KalmanFilter_t *kf)
{
    QuaternionEKF_t *ekf = QuaternionEKF_FromKalman(kf);
    if (kf->xhatSize != QUATERNION_EKF_STATE_SIZE || kf->zSize != QUATERNION_EKF_MEASUREMENT_SIZE)
    {
        QuaternionEKF_SetStatus(ekf, QUATERNION_EKF_STATUS_KALMAN);
        kf->Status = KALMAN_FILTER_STATUS_DIMENSION;
        return;
    }

    const float q0 = 2.0F * kf->xhatminus_data[0];
    const float q1 = 2.0F * kf->xhatminus_data[1];
    const float q2 = 2.0F * kf->xhatminus_data[2];
    const float q3 = 2.0F * kf->xhatminus_data[3];
    memset(kf->H_data, 0, sizeof(kf->H_data));
    kf->H_data[0] = -q2;
    kf->H_data[1] = q3;
    kf->H_data[2] = -q0;
    kf->H_data[3] = q1;
    kf->H_data[6] = q1;
    kf->H_data[7] = q0;
    kf->H_data[8] = q3;
    kf->H_data[9] = q2;
    kf->H_data[12] = q0;
    kf->H_data[13] = -q1;
    kf->H_data[14] = -q2;
    kf->H_data[15] = q3;
}

/**
 * @brief 依据卡方门限、自适应增益和参考观测函数执行 EKF 后验更新
 *
 * @param[in,out] kf Kalman 滤波器对象
 * @return 无，错误状态通过滤波器与 EKF 对象保存
 */
static void QuaternionEKF_xhatUpdate(KalmanFilter_t *kf)
{
    QuaternionEKF_t *ekf = QuaternionEKF_FromKalman(kf);
    bool apply_measurement = false;

    // 构造创新协方差并求逆，奇异矩阵时停止本周期融合
    kf->temp_matrix.numRows = kf->H.numRows;
    kf->temp_matrix.numCols = kf->Pminus.numCols;
    if (!QuaternionEKF_RecordMatrixStatus(ekf, kf, Matrix_Transpose(&kf->H, &kf->HT)) ||
        !QuaternionEKF_RecordMatrixStatus(ekf, kf, Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix)))
    {
        return;
    }
    kf->temp_matrix1.numRows = kf->H.numRows;
    kf->temp_matrix1.numCols = kf->H.numRows;
    kf->S.numRows = kf->H.numRows;
    kf->S.numCols = kf->H.numRows;
    if (!QuaternionEKF_RecordMatrixStatus(ekf, kf, Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1)) ||
        !QuaternionEKF_RecordMatrixStatus(ekf, kf, Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S)) ||
        !QuaternionEKF_RecordMatrixStatus(ekf, kf, Matrix_Inverse(&kf->S, &kf->temp_matrix1)))
    {
        return;
    }

    // 按先验姿态计算重力方向与加速度创新
    const float q0 = kf->xhatminus_data[0];
    const float q1 = kf->xhatminus_data[1];
    const float q2 = kf->xhatminus_data[2];
    const float q3 = kf->xhatminus_data[3];
    kf->temp_vector.numRows = kf->H.numRows;
    kf->temp_vector.numCols = 1U;
    kf->temp_vector_data[0] = 2.0F * (q1 * q3 - q0 * q2);
    kf->temp_vector_data[1] = 2.0F * (q0 * q1 + q2 * q3);
    kf->temp_vector_data[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    for (uint8_t axis = 0U; axis < QUATERNION_EKF_MEASUREMENT_SIZE; axis++)
    {
        const float direction_cosine = fminf(1.0F, fabsf(kf->temp_vector_data[axis]));
        ekf->OrientationCosine[axis] = acosf(direction_cosine);
    }

    kf->temp_vector1.numRows = kf->z.numRows;
    kf->temp_vector1.numCols = 1U;
    if (!QuaternionEKF_RecordMatrixStatus(ekf, kf, Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1)))
    {
        return;
    }

    // 计算卡方残差并由集中状态转换逻辑决定量测更新或仅预测
    kf->temp_matrix.numRows = kf->temp_vector1.numRows;
    kf->temp_matrix.numCols = 1U;
    kf->temp_vector.numRows = 1U;
    kf->temp_vector.numCols = kf->temp_vector1.numRows;
    if (!QuaternionEKF_RecordMatrixStatus(
            ekf, kf, Matrix_Multiply(&kf->temp_matrix1, &kf->temp_vector1, &kf->temp_matrix)) ||
        !QuaternionEKF_RecordMatrixStatus(ekf, kf, Matrix_Transpose(&kf->temp_vector1, &kf->temp_vector)) ||
        !QuaternionEKF_RecordMatrixStatus(
            ekf, kf, Matrix_Multiply(&kf->temp_vector, &kf->temp_matrix, &ekf->ChiSquare)))
    {
        return;
    }
    if (!QuaternionEKF_UpdateConvergence(ekf, kf, &apply_measurement))
    {
        return;
    }
    if (!apply_measurement)
    {
        memcpy(kf->xhat_data, kf->xhatminus_data, sizeof(float) * QUATERNION_EKF_STATE_SIZE);
        memcpy(kf->P_data, kf->Pminus_data, sizeof(float) * QUATERNION_EKF_COVARIANCE_ELEMENTS);
        return;
    }

    // 计算自适应 Kalman 增益并限制已收敛状态的零偏单步修正
    kf->temp_matrix.numRows = kf->Pminus.numRows;
    kf->temp_matrix.numCols = kf->HT.numCols;
    if (!QuaternionEKF_RecordMatrixStatus(ekf, kf, Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix)) ||
        !QuaternionEKF_RecordMatrixStatus(ekf, kf, Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K)))
    {
        return;
    }
    const uint16_t gain_elements = (uint16_t)kf->K.numRows * kf->K.numCols;
    for (uint16_t index = 0U; index < gain_elements; index++)
    {
        kf->K_data[index] *= ekf->AdaptiveGainScale;
    }
    for (uint8_t bias_axis = 0U; bias_axis < 2U; bias_axis++)
    {
        for (uint8_t measurement_axis = 0U; measurement_axis < QUATERNION_EKF_MEASUREMENT_SIZE; measurement_axis++)
        {
            const uint8_t index = (uint8_t)((bias_axis + 4U) * QUATERNION_EKF_MEASUREMENT_SIZE + measurement_axis);
            kf->K_data[index] *= ekf->OrientationCosine[bias_axis] / QUATERNION_EKF_HALF_PI_RAD;
        }
    }

    kf->temp_vector.numRows = kf->K.numRows;
    kf->temp_vector.numCols = 1U;
    if (!QuaternionEKF_RecordMatrixStatus(ekf, kf, Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector)))
    {
        return;
    }
    if (ekf->ConvergeFlag == QUATERNION_EKF_CONVERGENCE_LOCKED)
    {
        for (uint8_t bias_index = 4U; bias_index < QUATERNION_EKF_STATE_SIZE; bias_index++)
        {
            const float max_bias_correction = QUATERNION_EKF_MAX_BIAS_STEP_RAD_S * ekf->dt;
            if (kf->temp_vector_data[bias_index] > max_bias_correction)
            {
                kf->temp_vector_data[bias_index] = max_bias_correction;
            }
            if (kf->temp_vector_data[bias_index] < -max_bias_correction)
            {
                kf->temp_vector_data[bias_index] = -max_bias_correction;
            }
        }
    }
    kf->temp_vector_data[3] = 0.0F;
    QuaternionEKF_RecordMatrixStatus(ekf, kf, Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat));
}

/**
 * @brief 初始化四元数扩展 Kalman 姿态滤波器
 *
 * @param[in,out] ekf 姿态滤波实例，不能为空
 * @param[in] config 滤波参数，过程噪声非负、量测噪声和卡方阈值大于零、渐消因子范围为 (0,1]
 * @return 初始化状态，成功返回 QUATERNION_EKF_STATUS_OK
 */
QuaternionEKF_Status_t QuaternionEKF_Init(QuaternionEKF_t *ekf, const QuaternionEKF_Config_t *config)
{
    if (ekf == NULL)
    {
        return QUATERNION_EKF_STATUS_NULL;
    }

    // 清理实例并先记录未初始化状态，避免失败时遗留旧姿态
    memset(ekf, 0, sizeof(*ekf));
    ekf->Status = QUATERNION_EKF_STATUS_NOT_INITIALIZED;
    if (!QuaternionEKF_ConfigIsValid(config))
    {
        return QuaternionEKF_SetStatus(ekf, QUATERNION_EKF_STATUS_INVALID_ARGUMENT);
    }

    // 创建参考算法使用的六维状态、三维重力量测和对象内固定矩阵
    Kalman_Filter_Init(&ekf->IMU_QuaternionEKF, QUATERNION_EKF_STATE_SIZE, 0U, QUATERNION_EKF_MEASUREMENT_SIZE);
    if (Kalman_Filter_GetStatus(&ekf->IMU_QuaternionEKF) != KALMAN_FILTER_STATUS_OK)
    {
        return QuaternionEKF_SetStatus(ekf, QUATERNION_EKF_STATUS_KALMAN);
    }
    memcpy(ekf->IMU_QuaternionEKF.F_data, quaternion_ekf_initial_f, sizeof(quaternion_ekf_initial_f));
    memcpy(ekf->IMU_QuaternionEKF.P_data, quaternion_ekf_initial_p, sizeof(quaternion_ekf_initial_p));
    Matrix_Init(&ekf->ChiSquare, 1U, 1U, ekf->ChiSquare_Data);
    if (ekf->ChiSquare.pData != ekf->ChiSquare_Data || ekf->ChiSquare.numRows != 1U || ekf->ChiSquare.numCols != 1U)
    {
        return QuaternionEKF_SetStatus(ekf, QUATERNION_EKF_STATUS_KALMAN);
    }

    // 按参考 QEKF_INS_t 初始化参数、单位四元数和阶段回调
    ekf->Q1 = config->process_noise_quaternion;
    ekf->Q2 = config->process_noise_gyro_bias;
    ekf->R = config->measurement_noise_accel;
    ekf->lambda = config->fading_factor;
    ekf->accLPFcoef = config->accel_lpf_time_constant_s;
    ekf->ChiSquareTestThreshold = config->chi_square_test_threshold;
    ekf->ConvergeFlag = QUATERNION_EKF_CONVERGENCE_TRACKING;
    ekf->AdaptiveGainScale = 1.0F;
    ekf->q[0] = 1.0F;
    ekf->IMU_QuaternionEKF.xhat_data[0] = 1.0F;
    ekf->IMU_QuaternionEKF.User_Func0_f = QuaternionEKF_Observe;
    ekf->IMU_QuaternionEKF.User_Func1_f = QuaternionEKF_F_Linearization_P_Fading;
    ekf->IMU_QuaternionEKF.User_Func2_f = QuaternionEKF_SetH;
    ekf->IMU_QuaternionEKF.User_Func3_f = QuaternionEKF_xhatUpdate;
    ekf->IMU_QuaternionEKF.SkipEq3 = 1U;
    ekf->IMU_QuaternionEKF.SkipEq4 = 1U;
    ekf->Initialized = 1U;
    return QuaternionEKF_SetStatus(ekf, QUATERNION_EKF_STATUS_OK);
}

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
                                            float az, float dt)
{
    if (ekf == NULL)
    {
        return QUATERNION_EKF_STATUS_NULL;
    }
    if (ekf->Initialized == 0U)
    {
        return QuaternionEKF_SetStatus(ekf, QUATERNION_EKF_STATUS_NOT_INITIALIZED);
    }

    // 检查状态机当前值与传感器输入，拒绝非法周期和全零加速度
    switch (ekf->ConvergeFlag)
    {
        case QUATERNION_EKF_CONVERGENCE_TRACKING:
        case QUATERNION_EKF_CONVERGENCE_LOCKED:
            break;
        default:
            return QuaternionEKF_SetStatus(ekf, QUATERNION_EKF_STATUS_ILLEGAL_STATE);
    }
    if (!isfinite(gx) || !isfinite(gy) || !isfinite(gz) || !isfinite(ax) || !isfinite(ay) || !isfinite(az) ||
        !isfinite(dt) || dt <= 0.0F)
    {
        return QuaternionEKF_SetStatus(ekf, QUATERNION_EKF_STATUS_INVALID_ARGUMENT);
    }
    const float accel_norm_squared = ax * ax + ay * ay + az * az;
    if (!isfinite(accel_norm_squared))
    {
        return QuaternionEKF_SetStatus(ekf, QUATERNION_EKF_STATUS_NUMERIC);
    }
    if (accel_norm_squared <= FLT_MIN)
    {
        return QuaternionEKF_SetStatus(ekf, QUATERNION_EKF_STATUS_INVALID_ACCELERATION);
    }

    // 校验滤波时间常数后按实测 dt 修正角速度并平滑加速度输入
    const float lpf_denominator = dt + ekf->accLPFcoef;
    if (!isfinite(lpf_denominator) || lpf_denominator <= 0.0F)
    {
        return QuaternionEKF_SetStatus(ekf, QUATERNION_EKF_STATUS_INVALID_ARGUMENT);
    }
    ekf->dt = dt;
    ekf->Gyro[0] = gx - ekf->GyroBias[0];
    ekf->Gyro[1] = gy - ekf->GyroBias[1];
    ekf->Gyro[2] = gz - ekf->GyroBias[2];
    const float gyro_norm_squared = ekf->Gyro[0] * ekf->Gyro[0] + ekf->Gyro[1] * ekf->Gyro[1] +
                                    ekf->Gyro[2] * ekf->Gyro[2];
    if (!isfinite(gyro_norm_squared))
    {
        return QuaternionEKF_SetStatus(ekf, QUATERNION_EKF_STATUS_NUMERIC);
    }
    if (ekf->UpdateCount == 0U)
    {
        ekf->Accel[0] = ax;
        ekf->Accel[1] = ay;
        ekf->Accel[2] = az;
    }
    const float previous_weight = ekf->accLPFcoef / lpf_denominator;
    const float input_weight = dt / lpf_denominator;
    for (uint8_t axis = 0U; axis < QUATERNION_EKF_MEASUREMENT_SIZE; axis++)
    {
        const float input = axis == 0U ? ax : (axis == 1U ? ay : az);
        ekf->Accel[axis] = ekf->Accel[axis] * previous_weight + input * input_weight;
    }

    // 根据去偏角速度更新四元数转移矩阵并生成归一化重力方向量测
    memcpy(ekf->IMU_QuaternionEKF.F_data, quaternion_ekf_initial_f, sizeof(quaternion_ekf_initial_f));
    const float halfgxdt = 0.5F * ekf->Gyro[0] * dt;
    const float halfgydt = 0.5F * ekf->Gyro[1] * dt;
    const float halfgzdt = 0.5F * ekf->Gyro[2] * dt;
    ekf->IMU_QuaternionEKF.F_data[1] = -halfgxdt;
    ekf->IMU_QuaternionEKF.F_data[2] = -halfgydt;
    ekf->IMU_QuaternionEKF.F_data[3] = -halfgzdt;
    ekf->IMU_QuaternionEKF.F_data[6] = halfgxdt;
    ekf->IMU_QuaternionEKF.F_data[8] = halfgzdt;
    ekf->IMU_QuaternionEKF.F_data[9] = -halfgydt;
    ekf->IMU_QuaternionEKF.F_data[12] = halfgydt;
    ekf->IMU_QuaternionEKF.F_data[13] = -halfgzdt;
    ekf->IMU_QuaternionEKF.F_data[15] = halfgxdt;
    ekf->IMU_QuaternionEKF.F_data[18] = halfgzdt;
    ekf->IMU_QuaternionEKF.F_data[19] = halfgydt;
    ekf->IMU_QuaternionEKF.F_data[20] = -halfgxdt;
    float normalized_accel[QUATERNION_EKF_MEASUREMENT_SIZE] = {ekf->Accel[0], ekf->Accel[1], ekf->Accel[2]};
    const float filtered_accel_norm_squared = normalized_accel[0] * normalized_accel[0] +
                                              normalized_accel[1] * normalized_accel[1] +
                                              normalized_accel[2] * normalized_accel[2];
    if (!isfinite(filtered_accel_norm_squared) || filtered_accel_norm_squared <= FLT_MIN ||
        !QuaternionEKF_Normalize(normalized_accel, QUATERNION_EKF_MEASUREMENT_SIZE))
    {
        return QuaternionEKF_SetStatus(ekf, QUATERNION_EKF_STATUS_INVALID_ACCELERATION);
    }
    ekf->IMU_QuaternionEKF.MeasuredVector[0] = normalized_accel[0];
    ekf->IMU_QuaternionEKF.MeasuredVector[1] = normalized_accel[1];
    ekf->IMU_QuaternionEKF.MeasuredVector[2] = normalized_accel[2];

    // 计算稳定判据与过程噪声、加速度量测协方差
    ekf->gyro_norm = sqrtf(gyro_norm_squared);
    ekf->accl_norm = sqrtf(filtered_accel_norm_squared);
    ekf->StableFlag = (ekf->gyro_norm < QUATERNION_EKF_STABLE_GYRO_LIMIT_RAD_S &&
                       ekf->accl_norm > QUATERNION_EKF_STABLE_ACCEL_CENTER_MPS2 -
                                             QUATERNION_EKF_STABLE_ACCEL_TOLERANCE_MPS2 &&
                       ekf->accl_norm < QUATERNION_EKF_STABLE_ACCEL_CENTER_MPS2 +
                                             QUATERNION_EKF_STABLE_ACCEL_TOLERANCE_MPS2) ?
                          1U : 0U;
    memset(ekf->IMU_QuaternionEKF.Q_data, 0, sizeof(ekf->IMU_QuaternionEKF.Q_data));
    memset(ekf->IMU_QuaternionEKF.R_data, 0, sizeof(ekf->IMU_QuaternionEKF.R_data));
    ekf->IMU_QuaternionEKF.Q_data[0] = ekf->Q1 * dt;
    ekf->IMU_QuaternionEKF.Q_data[7] = ekf->Q1 * dt;
    ekf->IMU_QuaternionEKF.Q_data[14] = ekf->Q1 * dt;
    ekf->IMU_QuaternionEKF.Q_data[21] = ekf->Q1 * dt;
    ekf->IMU_QuaternionEKF.Q_data[28] = ekf->Q2 * dt;
    ekf->IMU_QuaternionEKF.Q_data[35] = ekf->Q2 * dt;
    ekf->IMU_QuaternionEKF.R_data[0] = ekf->R;
    ekf->IMU_QuaternionEKF.R_data[4] = ekf->R;
    ekf->IMU_QuaternionEKF.R_data[8] = ekf->R;

    // 执行参考 Kalman 阶段链并将矩阵失败保留为组件错误状态
    ekf->Status = QUATERNION_EKF_STATUS_OK;
    float *filtered_state = Kalman_Filter_Update(&ekf->IMU_QuaternionEKF);
    if (filtered_state == NULL || Kalman_Filter_GetStatus(&ekf->IMU_QuaternionEKF) != KALMAN_FILTER_STATUS_OK)
    {
        return QuaternionEKF_SetStatus(ekf, QUATERNION_EKF_STATUS_KALMAN);
    }

    // 归一化融合四元数并发布零偏与欧拉角估计
    if (!QuaternionEKF_Normalize(&ekf->IMU_QuaternionEKF.xhat_data[0], 4U))
    {
        return QuaternionEKF_SetStatus(ekf, QUATERNION_EKF_STATUS_NUMERIC);
    }
    memcpy(filtered_state, ekf->IMU_QuaternionEKF.xhat_data, sizeof(float) * QUATERNION_EKF_STATE_SIZE);
    memcpy(ekf->q, filtered_state, sizeof(ekf->q));
    ekf->GyroBias[0] = filtered_state[4];
    ekf->GyroBias[1] = filtered_state[5];
    ekf->GyroBias[2] = 0.0F;
    const float roll_argument = fmaxf(-1.0F, fminf(1.0F, -2.0F * (ekf->q[1] * ekf->q[3] - ekf->q[0] * ekf->q[2])));
    ekf->Yaw = atan2f(2.0F * (ekf->q[0] * ekf->q[3] + ekf->q[1] * ekf->q[2]),
                      2.0F * (ekf->q[0] * ekf->q[0] + ekf->q[1] * ekf->q[1]) - 1.0F) * QUATERNION_EKF_RAD_TO_DEG;
    ekf->Pitch = atan2f(2.0F * (ekf->q[0] * ekf->q[1] + ekf->q[2] * ekf->q[3]),
                        2.0F * (ekf->q[0] * ekf->q[0] + ekf->q[3] * ekf->q[3]) - 1.0F) * QUATERNION_EKF_RAD_TO_DEG;
    ekf->Roll = asinf(roll_argument) * QUATERNION_EKF_RAD_TO_DEG;

    // 根据相邻 Yaw 角变化累计旋转圈数并记录本次成功更新
    const float yaw_delta = ekf->Yaw - ekf->YawAngleLast;
    if (yaw_delta > QUATERNION_EKF_YAW_HALF_TURN_DEG)
    {
        if (ekf->YawRoundCount == INT16_MIN)
        {
            return QuaternionEKF_SetStatus(ekf, QUATERNION_EKF_STATUS_NUMERIC);
        }
        ekf->YawRoundCount--;
    }
    else if (yaw_delta < -QUATERNION_EKF_YAW_HALF_TURN_DEG)
    {
        if (ekf->YawRoundCount == INT16_MAX)
        {
            return QuaternionEKF_SetStatus(ekf, QUATERNION_EKF_STATUS_NUMERIC);
        }
        ekf->YawRoundCount++;
    }
    ekf->YawTotalAngle = 360.0F * ekf->YawRoundCount + ekf->Yaw;
    ekf->YawAngleLast = ekf->Yaw;
    ekf->UpdateCount++;
    return QuaternionEKF_SetStatus(ekf, QUATERNION_EKF_STATUS_OK);
}
