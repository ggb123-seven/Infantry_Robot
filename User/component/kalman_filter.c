#include "component/kalman_filter.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

uint16_t sizeof_float;
uint16_t sizeof_double;

static KalmanFilter_Status_t Kalman_Filter_H_K_R_Adjustment(KalmanFilter_t *kf);
static KalmanFilter_Status_t Kalman_Filter_Validate(KalmanFilter_t *kf);
static KalmanFilter_Status_t Kalman_Filter_SetStatus(KalmanFilter_t *kf, KalmanFilter_Status_t status);
static bool Kalman_Filter_RecordMatrixStatus(KalmanFilter_t *kf, arm_status status);
static bool Kalman_Filter_InvokeCallback(KalmanFilter_t *kf, KalmanFilterStageCallback_t callback);
static bool Kalman_Filter_MatrixIsValid(KalmanFilter_t *kf, const mat *matrix);
static bool Kalman_Filter_MatrixAdd(KalmanFilter_t *kf, const mat *left, const mat *right, mat *output);
static bool Kalman_Filter_MatrixSubtract(KalmanFilter_t *kf, const mat *left, const mat *right, mat *output);
static bool Kalman_Filter_MatrixMultiply(KalmanFilter_t *kf, const mat *left, const mat *right, mat *output);
static bool Kalman_Filter_MatrixTranspose(KalmanFilter_t *kf, const mat *input, mat *output);
static bool Kalman_Filter_MatrixInverse(KalmanFilter_t *kf, const mat *input, mat *output);
static bool Kalman_Filter_VectorIsFinite(const float *vector, uint8_t length);

/**
 * @brief 写入滤波器当前组件状态
 *
 * @param[in,out] kf 滤波器对象
 * @param[in] status 需要保存的组件状态
 * @return 写入的组件状态
 */
static KalmanFilter_Status_t Kalman_Filter_SetStatus(KalmanFilter_t *kf, KalmanFilter_Status_t status)
{
    if (kf != NULL)
    {
        kf->Status = status;
    }
    return status;
}

/**
 * @brief 检查滤波器对象是否已初始化
 *
 * @param[in,out] kf 滤波器对象
 * @return 当前对象可用时返回成功状态，否则返回对应错误
 */
static KalmanFilter_Status_t Kalman_Filter_Validate(KalmanFilter_t *kf)
{
    if (kf == NULL)
    {
        return KALMAN_FILTER_STATUS_NULL;
    }
    if (!kf->initialized)
    {
        return Kalman_Filter_SetStatus(kf, KALMAN_FILTER_STATUS_NOT_INITIALIZED);
    }
    return KALMAN_FILTER_STATUS_OK;
}

/**
 * @brief 保存 CMSIS 矩阵运算结果并转换组件状态
 *
 * @param[in,out] kf 滤波器对象
 * @param[in] status CMSIS 矩阵运算状态
 * @return 矩阵运算成功时返回 true，否则返回 false
 */
static bool Kalman_Filter_RecordMatrixStatus(KalmanFilter_t *kf, arm_status status)
{
    if (kf == NULL)
    {
        return false;
    }

    kf->MatStatus = (int8_t)status;
    if (status == ARM_MATH_SUCCESS)
    {
        kf->Status = KALMAN_FILTER_STATUS_OK;
        return true;
    }
    if (status == ARM_MATH_SINGULAR)
    {
        kf->Status = KALMAN_FILTER_STATUS_SINGULAR;
        return false;
    }
    if (status == ARM_MATH_SIZE_MISMATCH)
    {
        kf->Status = KALMAN_FILTER_STATUS_DIMENSION;
        return false;
    }

    kf->Status = KALMAN_FILTER_STATUS_MATRIX;
    return false;
}

/**
 * @brief 检查矩阵视图的数据和维度是否合法
 *
 * @param[in,out] kf 滤波器对象
 * @param[in] matrix 待检查的矩阵视图
 * @return 矩阵视图有效时返回 true，否则返回 false
 */
static bool Kalman_Filter_MatrixIsValid(KalmanFilter_t *kf, const mat *matrix)
{
    if (kf == NULL || matrix == NULL || matrix->pData == NULL || matrix->numRows == 0U || matrix->numCols == 0U)
    {
        Kalman_Filter_RecordMatrixStatus(kf, ARM_MATH_SIZE_MISMATCH);
        return false;
    }

    uint32_t max_elements = 0U;
    if (matrix == &kf->xhat || matrix == &kf->xhatminus)
    {
        max_elements = kf->xhatSize;
    }
    else if (matrix == &kf->u)
    {
        max_elements = kf->uSize;
    }
    else if (matrix == &kf->z)
    {
        max_elements = kf->zSize;
    }
    else if (matrix == &kf->P || matrix == &kf->Pminus || matrix == &kf->F || matrix == &kf->FT || matrix == &kf->Q)
    {
        max_elements = (uint32_t)kf->xhatSize * kf->xhatSize;
    }
    else if (matrix == &kf->B)
    {
        max_elements = (uint32_t)kf->xhatSize * kf->uSize;
    }
    else if (matrix == &kf->H || matrix == &kf->HT || matrix == &kf->K)
    {
        max_elements = (uint32_t)kf->xhatSize * kf->zSize;
    }
    else if (matrix == &kf->R || matrix == &kf->S)
    {
        max_elements = (uint32_t)kf->zSize * kf->zSize;
    }
    else if (matrix == &kf->temp_matrix || matrix == &kf->temp_matrix1)
    {
        max_elements = KALMAN_FILTER_MAX_MATRIX_ELEMENTS;
    }
    else if (matrix == &kf->temp_vector || matrix == &kf->temp_vector1)
    {
        max_elements = KALMAN_FILTER_MAX_MATRIX_DIMENSION;
    }
    if (max_elements == 0U || (uint32_t)matrix->numRows * matrix->numCols > max_elements)
    {
        Kalman_Filter_RecordMatrixStatus(kf, ARM_MATH_SIZE_MISMATCH);
        return false;
    }
    return true;
}

/**
 * @brief 检查两个矩阵的维度并执行加法
 *
 * @param[in,out] kf 滤波器对象
 * @param[in] left 左侧矩阵
 * @param[in] right 右侧矩阵
 * @param[out] output 结果矩阵
 * @return 运算成功时返回 true，否则返回 false
 */
static bool Kalman_Filter_MatrixAdd(KalmanFilter_t *kf, const mat *left, const mat *right, mat *output)
{
    if (!Kalman_Filter_MatrixIsValid(kf, left) || !Kalman_Filter_MatrixIsValid(kf, right) ||
        !Kalman_Filter_MatrixIsValid(kf, output))
    {
        return false;
    }
    if (left->numRows != right->numRows || left->numCols != right->numCols ||
        output->numRows != left->numRows || output->numCols != left->numCols)
    {
        Kalman_Filter_RecordMatrixStatus(kf, ARM_MATH_SIZE_MISMATCH);
        return false;
    }
    return Kalman_Filter_RecordMatrixStatus(kf, Matrix_Add(left, right, output));
}

/**
 * @brief 检查两个矩阵的维度并执行减法
 *
 * @param[in,out] kf 滤波器对象
 * @param[in] left 左侧矩阵
 * @param[in] right 右侧矩阵
 * @param[out] output 结果矩阵
 * @return 运算成功时返回 true，否则返回 false
 */
static bool Kalman_Filter_MatrixSubtract(KalmanFilter_t *kf, const mat *left, const mat *right, mat *output)
{
    if (!Kalman_Filter_MatrixIsValid(kf, left) || !Kalman_Filter_MatrixIsValid(kf, right) ||
        !Kalman_Filter_MatrixIsValid(kf, output))
    {
        return false;
    }
    if (left->numRows != right->numRows || left->numCols != right->numCols ||
        output->numRows != left->numRows || output->numCols != left->numCols)
    {
        Kalman_Filter_RecordMatrixStatus(kf, ARM_MATH_SIZE_MISMATCH);
        return false;
    }
    return Kalman_Filter_RecordMatrixStatus(kf, Matrix_Subtract(left, right, output));
}

/**
 * @brief 检查矩阵乘法维度并执行乘法
 *
 * @param[in,out] kf 滤波器对象
 * @param[in] left 左侧矩阵
 * @param[in] right 右侧矩阵
 * @param[out] output 结果矩阵
 * @return 运算成功时返回 true，否则返回 false
 */
static bool Kalman_Filter_MatrixMultiply(KalmanFilter_t *kf, const mat *left, const mat *right, mat *output)
{
    if (!Kalman_Filter_MatrixIsValid(kf, left) || !Kalman_Filter_MatrixIsValid(kf, right) ||
        !Kalman_Filter_MatrixIsValid(kf, output))
    {
        return false;
    }
    if (left->numCols != right->numRows || output->numRows != left->numRows || output->numCols != right->numCols)
    {
        Kalman_Filter_RecordMatrixStatus(kf, ARM_MATH_SIZE_MISMATCH);
        return false;
    }
    return Kalman_Filter_RecordMatrixStatus(kf, Matrix_Multiply(left, right, output));
}

/**
 * @brief 检查矩阵转置维度并执行转置
 *
 * @param[in,out] kf 滤波器对象
 * @param[in] input 输入矩阵
 * @param[out] output 转置结果矩阵
 * @return 运算成功时返回 true，否则返回 false
 */
static bool Kalman_Filter_MatrixTranspose(KalmanFilter_t *kf, const mat *input, mat *output)
{
    if (!Kalman_Filter_MatrixIsValid(kf, input) || !Kalman_Filter_MatrixIsValid(kf, output))
    {
        return false;
    }
    if (output->numRows != input->numCols || output->numCols != input->numRows)
    {
        Kalman_Filter_RecordMatrixStatus(kf, ARM_MATH_SIZE_MISMATCH);
        return false;
    }
    return Kalman_Filter_RecordMatrixStatus(kf, Matrix_Transpose(input, output));
}

/**
 * @brief 检查方阵维度并执行求逆
 *
 * @param[in,out] kf 滤波器对象
 * @param[in] input 待求逆矩阵
 * @param[out] output 逆矩阵结果
 * @return 运算成功时返回 true，否则返回 false
 */
static bool Kalman_Filter_MatrixInverse(KalmanFilter_t *kf, const mat *input, mat *output)
{
    if (!Kalman_Filter_MatrixIsValid(kf, input) || !Kalman_Filter_MatrixIsValid(kf, output))
    {
        return false;
    }
    if (input->numRows != input->numCols || output->numRows != input->numRows || output->numCols != input->numCols)
    {
        Kalman_Filter_RecordMatrixStatus(kf, ARM_MATH_SIZE_MISMATCH);
        return false;
    }
    return Kalman_Filter_RecordMatrixStatus(kf, Matrix_Inverse(input, output));
}

/**
 * @brief 检查向量中的数值是否均为有限值
 *
 * @param[in] vector 待检查向量
 * @param[in] length 向量元素数量
 * @return 全部元素有效时返回 true，否则返回 false
 */
static bool Kalman_Filter_VectorIsFinite(const float *vector, uint8_t length)
{
    if (vector == NULL)
    {
        return false;
    }
    for (uint8_t index = 0U; index < length; index++)
    {
        if (!isfinite(vector[index]))
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief 在量测处理阶段调用自定义回调并核对其矩阵结果
 *
 * @param[in,out] kf 滤波器对象
 * @param[in] callback 当前阶段回调，可为空
 * @return 回调及矩阵运算成功时返回 true，否则返回 false
 */
static bool Kalman_Filter_InvokeCallback(KalmanFilter_t *kf, KalmanFilterStageCallback_t callback)
{
    if (callback == NULL)
    {
        return kf->Status == KALMAN_FILTER_STATUS_OK;
    }

    kf->MatStatus = (int8_t)ARM_MATH_SUCCESS;
    callback(kf);
    if (kf->MatStatus != (int8_t)ARM_MATH_SUCCESS)
    {
        return Kalman_Filter_RecordMatrixStatus(kf, (arm_status)kf->MatStatus);
    }
    return kf->Status == KALMAN_FILTER_STATUS_OK;
}

/**
 * @brief 按量测有效性重排量测向量及 H、R、K 矩阵
 *
 * @param[in,out] kf 已初始化的滤波器对象
 * @return 调整成功时返回成功状态，否则返回参数错误
 */
static KalmanFilter_Status_t Kalman_Filter_H_K_R_Adjustment(KalmanFilter_t *kf)
{
    kf->MeasurementValidNum = 0U;
    memcpy(kf->z_data, kf->MeasuredVector, sizeof(float) * kf->zSize);
    memset(kf->MeasuredVector, 0, sizeof(float) * kf->zSize);
    memset(kf->R_data, 0, sizeof(kf->R_data));
    memset(kf->H_data, 0, sizeof(kf->H_data));

    // 筛选有效量测并核对其状态映射和噪声参数
    for (uint8_t index = 0U; index < kf->zSize; index++)
    {
        if (!isfinite(kf->z_data[index]))
        {
            return Kalman_Filter_SetStatus(kf, KALMAN_FILTER_STATUS_INVALID_ARGUMENT);
        }
        if (kf->z_data[index] != 0.0F)
        {
            const uint8_t state_index = kf->MeasurementMap[index];
            const float degree = kf->MeasurementDegree[index];
            const float variance = kf->MatR_DiagonalElements[index];
            if (state_index == 0U || state_index > kf->xhatSize || !isfinite(degree) || !isfinite(variance) ||
                variance < 0.0F)
            {
                return Kalman_Filter_SetStatus(kf, KALMAN_FILTER_STATUS_INVALID_ARGUMENT);
            }

            const uint8_t valid_index = kf->MeasurementValidNum;
            kf->z_data[valid_index] = kf->z_data[index];
            kf->temp[valid_index] = index;
            kf->H_data[kf->xhatSize * valid_index + state_index - 1U] = degree;
            kf->MeasurementValidNum++;
        }
    }

    // 根据有效量测数量重建对角量测噪声矩阵
    for (uint8_t index = 0U; index < kf->MeasurementValidNum; index++)
    {
        const uint8_t source_index = kf->temp[index];
        kf->R_data[index * kf->MeasurementValidNum + index] = kf->MatR_DiagonalElements[source_index];
    }

    // 同步动态量测维数，避免矩阵运算读取无效行列
    kf->H.numRows = kf->MeasurementValidNum;
    kf->H.numCols = kf->xhatSize;
    kf->HT.numRows = kf->xhatSize;
    kf->HT.numCols = kf->MeasurementValidNum;
    kf->R.numRows = kf->MeasurementValidNum;
    kf->R.numCols = kf->MeasurementValidNum;
    kf->K.numRows = kf->xhatSize;
    kf->K.numCols = kf->MeasurementValidNum;
    kf->z.numRows = kf->MeasurementValidNum;
    return Kalman_Filter_SetStatus(kf, KALMAN_FILTER_STATUS_OK);
}

/**
 * @brief 初始化滤波器维数、矩阵视图和静态工作区
 *
 * @param[in,out] kf 滤波器对象，不能为空
 * @param[in] xhatSize 状态维数，范围为 1 至配置上限
 * @param[in] uSize 控制维数，范围为 0 至配置上限
 * @param[in] zSize 量测维数，范围为 1 至配置上限
 * @return 无，初始化结果通过 Kalman_Filter_GetStatus 查询
 */
void Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize)
{
    if (kf == NULL)
    {
        return;
    }

    // 清空对象内静态存储并建立可诊断的初始状态
    memset(kf, 0, sizeof(*kf));
    kf->Status = KALMAN_FILTER_STATUS_NOT_INITIALIZED;
    kf->MatStatus = (int8_t)ARM_MATH_SUCCESS;
    sizeof_float = (uint16_t)sizeof(float);
    sizeof_double = (uint16_t)sizeof(double);

    // 拒绝零维或超出静态容量的滤波配置
    if (xhatSize == 0U || xhatSize > KALMAN_FILTER_MAX_STATE_SIZE ||
        uSize > KALMAN_FILTER_MAX_CONTROL_SIZE || zSize == 0U || zSize > KALMAN_FILTER_MAX_MEASUREMENT_SIZE)
    {
        kf->Status = KALMAN_FILTER_STATUS_DIMENSION;
        return;
    }

    kf->xhatSize = xhatSize;
    kf->uSize = uSize;
    kf->zSize = zSize;

    // 将状态和量测向量映射到滤波器对象内的固定数组
    Matrix_Init(&kf->xhat, xhatSize, 1U, kf->xhat_data);
    Matrix_Init(&kf->xhatminus, xhatSize, 1U, kf->xhatminus_data);
    Matrix_Init(&kf->z, zSize, 1U, kf->z_data);
    if (uSize > 0U)
    {
        Matrix_Init(&kf->u, uSize, 1U, kf->u_data);
        Matrix_Init(&kf->B, xhatSize, uSize, kf->B_data);
    }

    // 建立协方差、转移、量测和增益矩阵视图
    Matrix_Init(&kf->P, xhatSize, xhatSize, kf->P_data);
    Matrix_Init(&kf->Pminus, xhatSize, xhatSize, kf->Pminus_data);
    Matrix_Init(&kf->F, xhatSize, xhatSize, kf->F_data);
    Matrix_Init(&kf->FT, xhatSize, xhatSize, kf->FT_data);
    Matrix_Init(&kf->H, zSize, xhatSize, kf->H_data);
    Matrix_Init(&kf->HT, xhatSize, zSize, kf->HT_data);
    Matrix_Init(&kf->Q, xhatSize, xhatSize, kf->Q_data);
    Matrix_Init(&kf->R, zSize, zSize, kf->R_data);
    Matrix_Init(&kf->K, xhatSize, zSize, kf->K_data);
    Matrix_Init(&kf->S, zSize, zSize, kf->S_data);

    // 将临时矩阵和向量映射到对象内共享工作区
    Matrix_Init(&kf->temp_matrix, xhatSize, xhatSize, kf->temp_matrix_data);
    Matrix_Init(&kf->temp_matrix1, xhatSize, xhatSize, kf->temp_matrix_data1);
    Matrix_Init(&kf->temp_vector, xhatSize, 1U, kf->temp_vector_data);
    Matrix_Init(&kf->temp_vector1, xhatSize, 1U, kf->temp_vector_data1);
    kf->initialized = true;
    kf->Status = KALMAN_FILTER_STATUS_OK;
}

/**
 * @brief 获取滤波器最近一次初始化或更新状态
 *
 * @param[in] kf 滤波器对象，不能为空
 * @return 最近一次组件状态，空指针时返回 KALMAN_FILTER_STATUS_NULL
 */
KalmanFilter_Status_t Kalman_Filter_GetStatus(const KalmanFilter_t *kf)
{
    if (kf == NULL)
    {
        return KALMAN_FILTER_STATUS_NULL;
    }
    return kf->Status;
}

/**
 * @brief 准备量测和控制数据并生成量测矩阵
 *
 * @param[in,out] kf 已初始化的滤波器对象
 * @return 无，处理结果通过 Kalman_Filter_GetStatus 查询
 */
void Kalman_Filter_Measure(KalmanFilter_t *kf)
{
    if (Kalman_Filter_Validate(kf) != KALMAN_FILTER_STATUS_OK)
    {
        return;
    }

    // 根据量测有效标志压缩量测向量和对应矩阵
    if (kf->UseAutoAdjustment != 0U)
    {
        if (Kalman_Filter_H_K_R_Adjustment(kf) != KALMAN_FILTER_STATUS_OK)
        {
            return;
        }
    }
    else
    {
        if (!Kalman_Filter_VectorIsFinite(kf->MeasuredVector, kf->zSize))
        {
            Kalman_Filter_SetStatus(kf, KALMAN_FILTER_STATUS_INVALID_ARGUMENT);
            return;
        }
        memcpy(kf->z_data, kf->MeasuredVector, sizeof(float) * kf->zSize);
        memset(kf->MeasuredVector, 0, sizeof(float) * kf->zSize);
        kf->MeasurementValidNum = kf->zSize;
        kf->H.numRows = kf->zSize;
        kf->H.numCols = kf->xhatSize;
        kf->HT.numRows = kf->xhatSize;
        kf->HT.numCols = kf->zSize;
        kf->R.numRows = kf->zSize;
        kf->R.numCols = kf->zSize;
        kf->K.numRows = kf->xhatSize;
        kf->K.numCols = kf->zSize;
        kf->z.numRows = kf->zSize;
    }

    // 仅在配置控制输入时复制控制向量
    if (kf->uSize > 0U)
    {
        if (!Kalman_Filter_VectorIsFinite(kf->ControlVector, kf->uSize))
        {
            Kalman_Filter_SetStatus(kf, KALMAN_FILTER_STATUS_INVALID_ARGUMENT);
            return;
        }
        memcpy(kf->u_data, kf->ControlVector, sizeof(float) * kf->uSize);
    }
    kf->MatStatus = (int8_t)ARM_MATH_SUCCESS;
    Kalman_Filter_SetStatus(kf, KALMAN_FILTER_STATUS_OK);
}

/**
 * @brief 计算状态向量的先验估计
 *
 * @param[in,out] kf 已初始化的滤波器对象
 * @return 无，处理结果通过 Kalman_Filter_GetStatus 查询
 */
void Kalman_Filter_xhatMinusUpdate(KalmanFilter_t *kf)
{
    if (Kalman_Filter_Validate(kf) != KALMAN_FILTER_STATUS_OK || kf->SkipEq1 != 0U)
    {
        return;
    }

    // 组合状态转移和可选控制输入得到先验状态
    if (kf->uSize > 0U)
    {
        kf->temp_vector.numRows = kf->xhatSize;
        kf->temp_vector.numCols = 1U;
        kf->temp_vector1.numRows = kf->xhatSize;
        kf->temp_vector1.numCols = 1U;
        if (!Kalman_Filter_MatrixMultiply(kf, &kf->F, &kf->xhat, &kf->temp_vector) ||
            !Kalman_Filter_MatrixMultiply(kf, &kf->B, &kf->u, &kf->temp_vector1) ||
            !Kalman_Filter_MatrixAdd(kf, &kf->temp_vector, &kf->temp_vector1, &kf->xhatminus))
        {
            return;
        }
    }
    else if (!Kalman_Filter_MatrixMultiply(kf, &kf->F, &kf->xhat, &kf->xhatminus))
    {
        return;
    }
    Kalman_Filter_SetStatus(kf, KALMAN_FILTER_STATUS_OK);
}

/**
 * @brief 计算状态协方差的先验估计
 *
 * @param[in,out] kf 已初始化的滤波器对象
 * @return 无，处理结果通过 Kalman_Filter_GetStatus 查询
 */
void Kalman_Filter_PminusUpdate(KalmanFilter_t *kf)
{
    if (Kalman_Filter_Validate(kf) != KALMAN_FILTER_STATUS_OK || kf->SkipEq2 != 0U)
    {
        return;
    }

    // 按 F P F 转置加 Q 计算先验协方差
    kf->temp_matrix.numRows = kf->xhatSize;
    kf->temp_matrix.numCols = kf->xhatSize;
    kf->temp_matrix1.numRows = kf->xhatSize;
    kf->temp_matrix1.numCols = kf->xhatSize;
    if (!Kalman_Filter_MatrixTranspose(kf, &kf->F, &kf->FT) ||
        !Kalman_Filter_MatrixMultiply(kf, &kf->F, &kf->P, &kf->temp_matrix) ||
        !Kalman_Filter_MatrixMultiply(kf, &kf->temp_matrix, &kf->FT, &kf->temp_matrix1) ||
        !Kalman_Filter_MatrixAdd(kf, &kf->temp_matrix1, &kf->Q, &kf->Pminus))
    {
        return;
    }
    Kalman_Filter_SetStatus(kf, KALMAN_FILTER_STATUS_OK);
}

/**
 * @brief 根据当前量测和先验协方差计算 Kalman 增益
 *
 * @param[in,out] kf 已初始化且包含有效量测的滤波器对象
 * @return 无，处理结果通过 Kalman_Filter_GetStatus 查询
 */
void Kalman_Filter_SetK(KalmanFilter_t *kf)
{
    if (Kalman_Filter_Validate(kf) != KALMAN_FILTER_STATUS_OK || kf->SkipEq3 != 0U)
    {
        return;
    }
    if (kf->MeasurementValidNum == 0U)
    {
        Kalman_Filter_SetStatus(kf, KALMAN_FILTER_STATUS_INVALID_ARGUMENT);
        return;
    }

    // 计算残差协方差及其逆矩阵
    kf->temp_matrix.numRows = kf->H.numRows;
    kf->temp_matrix.numCols = kf->Pminus.numCols;
    kf->temp_matrix1.numRows = kf->H.numRows;
    kf->temp_matrix1.numCols = kf->H.numRows;
    kf->S.numRows = kf->R.numRows;
    kf->S.numCols = kf->R.numCols;
    if (!Kalman_Filter_MatrixTranspose(kf, &kf->H, &kf->HT) ||
        !Kalman_Filter_MatrixMultiply(kf, &kf->H, &kf->Pminus, &kf->temp_matrix) ||
        !Kalman_Filter_MatrixMultiply(kf, &kf->temp_matrix, &kf->HT, &kf->temp_matrix1) ||
        !Kalman_Filter_MatrixAdd(kf, &kf->temp_matrix1, &kf->R, &kf->S))
    {
        return;
    }
    kf->temp_matrix1.numRows = kf->S.numRows;
    kf->temp_matrix1.numCols = kf->S.numCols;
    if (!Kalman_Filter_MatrixInverse(kf, &kf->S, &kf->temp_matrix1))
    {
        return;
    }

    // 将先验协方差与残差协方差逆矩阵相乘得到增益
    kf->temp_matrix.numRows = kf->Pminus.numRows;
    kf->temp_matrix.numCols = kf->HT.numCols;
    if (!Kalman_Filter_MatrixMultiply(kf, &kf->Pminus, &kf->HT, &kf->temp_matrix) ||
        !Kalman_Filter_MatrixMultiply(kf, &kf->temp_matrix, &kf->temp_matrix1, &kf->K))
    {
        return;
    }
    Kalman_Filter_SetStatus(kf, KALMAN_FILTER_STATUS_OK);
}

/**
 * @brief 根据量测残差和 Kalman 增益更新状态估计
 *
 * @param[in,out] kf 已初始化且包含有效量测的滤波器对象
 * @return 无，处理结果通过 Kalman_Filter_GetStatus 查询
 */
void Kalman_Filter_xhatUpdate(KalmanFilter_t *kf)
{
    if (Kalman_Filter_Validate(kf) != KALMAN_FILTER_STATUS_OK || kf->SkipEq4 != 0U)
    {
        return;
    }

    // 计算量测残差并修正先验状态
    kf->temp_vector.numRows = kf->H.numRows;
    kf->temp_vector.numCols = 1U;
    kf->temp_vector1.numRows = kf->z.numRows;
    kf->temp_vector1.numCols = 1U;
    if (!Kalman_Filter_MatrixMultiply(kf, &kf->H, &kf->xhatminus, &kf->temp_vector) ||
        !Kalman_Filter_MatrixSubtract(kf, &kf->z, &kf->temp_vector, &kf->temp_vector1))
    {
        return;
    }
    kf->temp_vector.numRows = kf->K.numRows;
    kf->temp_vector.numCols = 1U;
    if (!Kalman_Filter_MatrixMultiply(kf, &kf->K, &kf->temp_vector1, &kf->temp_vector) ||
        !Kalman_Filter_MatrixAdd(kf, &kf->xhatminus, &kf->temp_vector, &kf->xhat))
    {
        return;
    }
    Kalman_Filter_SetStatus(kf, KALMAN_FILTER_STATUS_OK);
}

/**
 * @brief 根据 Kalman 增益更新状态后验协方差
 *
 * @param[in,out] kf 已初始化且包含有效量测的滤波器对象
 * @return 无，处理结果通过 Kalman_Filter_GetStatus 查询
 */
void Kalman_Filter_P_Update(KalmanFilter_t *kf)
{
    if (Kalman_Filter_Validate(kf) != KALMAN_FILTER_STATUS_OK || kf->SkipEq5 != 0U)
    {
        return;
    }

    // 按 P 先验减去 K H P 先验得到后验协方差
    kf->temp_matrix.numRows = kf->K.numRows;
    kf->temp_matrix.numCols = kf->H.numCols;
    kf->temp_matrix1.numRows = kf->K.numRows;
    kf->temp_matrix1.numCols = kf->Pminus.numCols;
    if (!Kalman_Filter_MatrixMultiply(kf, &kf->K, &kf->H, &kf->temp_matrix) ||
        !Kalman_Filter_MatrixMultiply(kf, &kf->temp_matrix, &kf->Pminus, &kf->temp_matrix1) ||
        !Kalman_Filter_MatrixSubtract(kf, &kf->Pminus, &kf->temp_matrix1, &kf->P))
    {
        return;
    }
    Kalman_Filter_SetStatus(kf, KALMAN_FILTER_STATUS_OK);
}

/**
 * @brief 执行量测、预测、增益和后验更新阶段
 *
 * @param[in,out] kf 已初始化的滤波器对象
 * @return 成功时返回滤波状态向量，失败时返回空指针
 */
float *Kalman_Filter_Update(KalmanFilter_t *kf)
{
    if (Kalman_Filter_Validate(kf) != KALMAN_FILTER_STATUS_OK)
    {
        return NULL;
    }

    // 准备本周期量测数据并调用量测阶段扩展
    Kalman_Filter_Measure(kf);
    if (kf->Status != KALMAN_FILTER_STATUS_OK || !Kalman_Filter_InvokeCallback(kf, kf->User_Func0_f))
    {
        return NULL;
    }

    // 计算先验状态并调用状态预测扩展
    Kalman_Filter_xhatMinusUpdate(kf);
    if (kf->Status != KALMAN_FILTER_STATUS_OK || !Kalman_Filter_InvokeCallback(kf, kf->User_Func1_f))
    {
        return NULL;
    }

    // 计算先验协方差并调用协方差预测扩展
    Kalman_Filter_PminusUpdate(kf);
    if (kf->Status != KALMAN_FILTER_STATUS_OK || !Kalman_Filter_InvokeCallback(kf, kf->User_Func2_f))
    {
        return NULL;
    }

    // 有有效量测时依次计算增益、后验状态和后验协方差
    if (kf->MeasurementValidNum != 0U || kf->UseAutoAdjustment == 0U)
    {
        Kalman_Filter_SetK(kf);
        if (kf->Status != KALMAN_FILTER_STATUS_OK || !Kalman_Filter_InvokeCallback(kf, kf->User_Func3_f))
        {
            return NULL;
        }

        Kalman_Filter_xhatUpdate(kf);
        if (kf->Status != KALMAN_FILTER_STATUS_OK || !Kalman_Filter_InvokeCallback(kf, kf->User_Func4_f))
        {
            return NULL;
        }

        Kalman_Filter_P_Update(kf);
        if (kf->Status != KALMAN_FILTER_STATUS_OK)
        {
            return NULL;
        }
    }
    else
    {
        // 无有效量测时仅发布先验估计
        memcpy(kf->xhat_data, kf->xhatminus_data, sizeof(float) * kf->xhatSize);
        memcpy(kf->P_data, kf->Pminus_data, sizeof(float) * kf->xhatSize * kf->xhatSize);
    }

    // 执行后处理回调并限制协方差对角线的最小值
    if (!Kalman_Filter_InvokeCallback(kf, kf->User_Func5_f))
    {
        return NULL;
    }
    for (uint8_t index = 0U; index < kf->xhatSize; index++)
    {
        const uint16_t diagonal_index = (uint16_t)index * kf->xhatSize + index;
        const float minimum_variance = kf->StateMinVariance[index];
        if (!isfinite(minimum_variance) || minimum_variance < 0.0F || !isfinite(kf->P_data[diagonal_index]))
        {
            Kalman_Filter_SetStatus(kf, KALMAN_FILTER_STATUS_NUMERIC);
            return NULL;
        }
        if (kf->P_data[diagonal_index] < minimum_variance)
        {
            kf->P_data[diagonal_index] = minimum_variance;
        }
    }
    if (!Kalman_Filter_VectorIsFinite(kf->xhat_data, kf->xhatSize))
    {
        Kalman_Filter_SetStatus(kf, KALMAN_FILTER_STATUS_NUMERIC);
        return NULL;
    }

    // 发布滤波结果并执行最终观测回调
    memcpy(kf->FilteredValue, kf->xhat_data, sizeof(float) * kf->xhatSize);
    if (!Kalman_Filter_InvokeCallback(kf, kf->User_Func6_f))
    {
        return NULL;
    }
    if (!Kalman_Filter_VectorIsFinite(kf->FilteredValue, kf->xhatSize))
    {
        Kalman_Filter_SetStatus(kf, KALMAN_FILTER_STATUS_NUMERIC);
        return NULL;
    }
    return kf->FilteredValue;
}
