#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

#include "arm_math.h"

/* 默认支持 6 维状态、控制和量测；构建时可统一覆盖容量以适配其他滤波器 */
#ifndef KALMAN_FILTER_MAX_STATE_SIZE
#define KALMAN_FILTER_MAX_STATE_SIZE (6U)
#endif

#ifndef KALMAN_FILTER_MAX_CONTROL_SIZE
#define KALMAN_FILTER_MAX_CONTROL_SIZE (6U)
#endif

#ifndef KALMAN_FILTER_MAX_MEASUREMENT_SIZE
#define KALMAN_FILTER_MAX_MEASUREMENT_SIZE (6U)
#endif

#define KALMAN_FILTER_MAX_MATRIX_DIMENSION \
    ((KALMAN_FILTER_MAX_STATE_SIZE > KALMAN_FILTER_MAX_MEASUREMENT_SIZE) ? \
         KALMAN_FILTER_MAX_STATE_SIZE : KALMAN_FILTER_MAX_MEASUREMENT_SIZE)
#define KALMAN_FILTER_MAX_MATRIX_ELEMENTS \
    (KALMAN_FILTER_MAX_MATRIX_DIMENSION * KALMAN_FILTER_MAX_MATRIX_DIMENSION)

#define mat arm_matrix_instance_f32
#define Matrix_Init arm_mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32

/**
 * @brief 通用 Kalman 滤波器状态
 */
typedef enum
{
    KALMAN_FILTER_STATUS_OK = 0,
    KALMAN_FILTER_STATUS_NULL = -1,
    KALMAN_FILTER_STATUS_DIMENSION = -2,
    KALMAN_FILTER_STATUS_NOT_INITIALIZED = -3,
    KALMAN_FILTER_STATUS_MATRIX = -4,
    KALMAN_FILTER_STATUS_SINGULAR = -5,
    KALMAN_FILTER_STATUS_INVALID_ARGUMENT = -6,
    KALMAN_FILTER_STATUS_NUMERIC = -7,
} KalmanFilter_Status_t;

typedef struct kf_t KalmanFilter_t;
typedef void (*KalmanFilterStageCallback_t)(KalmanFilter_t *kf);

/*
 * KalmanFilter_t 保存滤波输入输出、量测配置、矩阵视图、回调和固定容量工作区
 * FilteredValue 是状态估计输出，MeasuredVector 和 ControlVector 是待处理输入向量
 * xhatSize、uSize、zSize 和 MeasurementValidNum 分别表示状态、控制、量测及有效量测维数
 * UseAutoAdjustment 控制是否按有效量测重排 H、R、K；MeasurementMap 是量测到状态的 1 起始索引映射
 * MeasurementDegree 是 H 矩阵对应系数，MatR_DiagonalElements 是 R 对角量测方差，StateMinVariance 是 P 对角最小值
 * SkipEq1 至 SkipEq5 分别跳过五个标准方程；User_Func0_f 至 User_Func6_f 是量测、预测、更新和后处理回调
 * xhat 和 xhatminus 是后验及先验状态，u 和 z 是矩阵形式的控制及量测向量
 * P 和 Pminus 是后验及先验协方差，F 和 FT 是状态转移矩阵及其转置
 * B 是控制矩阵，H 和 HT 是量测矩阵及其转置，Q 和 R 是过程噪声及量测噪声协方差
 * K 是 Kalman 增益，S 是量测残差协方差，temp_matrix 与 temp_vector 是计算暂存视图
 * *_data 数组提供对应矩阵或向量的对象内存储，容量受文件顶部宏限制
 * MatStatus 保存最近一次 CMSIS 矩阵状态，Status 保存组件状态，initialized 表示初始化是否成功
 */
struct kf_t
{
    float FilteredValue[KALMAN_FILTER_MAX_STATE_SIZE];
    float MeasuredVector[KALMAN_FILTER_MAX_MEASUREMENT_SIZE];
    float ControlVector[KALMAN_FILTER_MAX_CONTROL_SIZE];

    uint8_t xhatSize;
    uint8_t uSize;
    uint8_t zSize;
    uint8_t UseAutoAdjustment;
    uint8_t MeasurementValidNum;

    uint8_t MeasurementMap[KALMAN_FILTER_MAX_MEASUREMENT_SIZE];
    float MeasurementDegree[KALMAN_FILTER_MAX_MEASUREMENT_SIZE];
    float MatR_DiagonalElements[KALMAN_FILTER_MAX_MEASUREMENT_SIZE];
    float StateMinVariance[KALMAN_FILTER_MAX_STATE_SIZE];
    uint8_t temp[KALMAN_FILTER_MAX_MEASUREMENT_SIZE];

    uint8_t SkipEq1;
    uint8_t SkipEq2;
    uint8_t SkipEq3;
    uint8_t SkipEq4;
    uint8_t SkipEq5;

    mat xhat;
    mat xhatminus;
    mat u;
    mat z;
    mat P;
    mat Pminus;
    mat F;
    mat FT;
    mat B;
    mat H;
    mat HT;
    mat Q;
    mat R;
    mat K;
    mat S;
    mat temp_matrix;
    mat temp_matrix1;
    mat temp_vector;
    mat temp_vector1;

    int8_t MatStatus;
    KalmanFilter_Status_t Status;
    bool initialized;

    KalmanFilterStageCallback_t User_Func0_f;
    KalmanFilterStageCallback_t User_Func1_f;
    KalmanFilterStageCallback_t User_Func2_f;
    KalmanFilterStageCallback_t User_Func3_f;
    KalmanFilterStageCallback_t User_Func4_f;
    KalmanFilterStageCallback_t User_Func5_f;
    KalmanFilterStageCallback_t User_Func6_f;

    float xhat_data[KALMAN_FILTER_MAX_STATE_SIZE];
    float xhatminus_data[KALMAN_FILTER_MAX_STATE_SIZE];
    float u_data[KALMAN_FILTER_MAX_CONTROL_SIZE];
    float z_data[KALMAN_FILTER_MAX_MEASUREMENT_SIZE];
    float P_data[KALMAN_FILTER_MAX_STATE_SIZE * KALMAN_FILTER_MAX_STATE_SIZE];
    float Pminus_data[KALMAN_FILTER_MAX_STATE_SIZE * KALMAN_FILTER_MAX_STATE_SIZE];
    float F_data[KALMAN_FILTER_MAX_STATE_SIZE * KALMAN_FILTER_MAX_STATE_SIZE];
    float FT_data[KALMAN_FILTER_MAX_STATE_SIZE * KALMAN_FILTER_MAX_STATE_SIZE];
    float B_data[KALMAN_FILTER_MAX_STATE_SIZE * KALMAN_FILTER_MAX_CONTROL_SIZE];
    float H_data[KALMAN_FILTER_MAX_MEASUREMENT_SIZE * KALMAN_FILTER_MAX_STATE_SIZE];
    float HT_data[KALMAN_FILTER_MAX_STATE_SIZE * KALMAN_FILTER_MAX_MEASUREMENT_SIZE];
    float Q_data[KALMAN_FILTER_MAX_STATE_SIZE * KALMAN_FILTER_MAX_STATE_SIZE];
    float R_data[KALMAN_FILTER_MAX_MEASUREMENT_SIZE * KALMAN_FILTER_MAX_MEASUREMENT_SIZE];
    float K_data[KALMAN_FILTER_MAX_STATE_SIZE * KALMAN_FILTER_MAX_MEASUREMENT_SIZE];
    float S_data[KALMAN_FILTER_MAX_MEASUREMENT_SIZE * KALMAN_FILTER_MAX_MEASUREMENT_SIZE];
    float temp_matrix_data[KALMAN_FILTER_MAX_MATRIX_ELEMENTS];
    float temp_matrix_data1[KALMAN_FILTER_MAX_MATRIX_ELEMENTS];
    float temp_vector_data[KALMAN_FILTER_MAX_MATRIX_DIMENSION];
    float temp_vector_data1[KALMAN_FILTER_MAX_MATRIX_DIMENSION];
};

extern uint16_t sizeof_float;
extern uint16_t sizeof_double;

/**
 * @brief 初始化 Kalman 滤波器及其静态矩阵视图
 *
 * @param[in,out] kf 滤波器对象，不能为空
 * @param[in] xhatSize 状态维数，范围为 1 至 KALMAN_FILTER_MAX_STATE_SIZE
 * @param[in] uSize 控制维数，范围为 0 至 KALMAN_FILTER_MAX_CONTROL_SIZE
 * @param[in] zSize 量测维数，范围为 1 至 KALMAN_FILTER_MAX_MEASUREMENT_SIZE
 * @return 无，初始化结果通过 Kalman_Filter_GetStatus 查询
 */
void Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize);

/**
 * @brief 执行一次 Kalman 滤波更新并调用已配置的阶段回调
 *
 * @param[in,out] kf 已初始化的滤波器对象
 * @return 成功时返回滤波状态向量，失败时返回空指针
 */
float *Kalman_Filter_Update(KalmanFilter_t *kf);

/**
 * @brief 获取滤波器最近一次初始化或更新状态
 *
 * @param[in] kf 滤波器对象，不能为空
 * @return 最近一次组件状态，空指针时返回 KALMAN_FILTER_STATUS_NULL
 */
KalmanFilter_Status_t Kalman_Filter_GetStatus(const KalmanFilter_t *kf);

/**
 * @brief 准备量测与控制向量并按需压缩有效量测
 *
 * @param[in,out] kf 已初始化的滤波器对象
 * @return 无，处理状态通过 Kalman_Filter_GetStatus 查询
 */
void Kalman_Filter_Measure(KalmanFilter_t *kf);

/**
 * @brief 根据状态转移矩阵计算先验状态
 *
 * @param[in,out] kf 已初始化的滤波器对象
 * @return 无，处理状态通过 Kalman_Filter_GetStatus 查询
 */
void Kalman_Filter_xhatMinusUpdate(KalmanFilter_t *kf);

/**
 * @brief 根据状态转移矩阵和过程噪声计算先验协方差
 *
 * @param[in,out] kf 已初始化的滤波器对象
 * @return 无，处理状态通过 Kalman_Filter_GetStatus 查询
 */
void Kalman_Filter_PminusUpdate(KalmanFilter_t *kf);

/**
 * @brief 根据量测和协方差矩阵计算 Kalman 增益
 *
 * @param[in,out] kf 已初始化且包含有效量测的滤波器对象
 * @return 无，处理状态通过 Kalman_Filter_GetStatus 查询
 */
void Kalman_Filter_SetK(KalmanFilter_t *kf);

/**
 * @brief 根据量测残差和 Kalman 增益更新状态估计
 *
 * @param[in,out] kf 已初始化且包含有效量测的滤波器对象
 * @return 无，处理状态通过 Kalman_Filter_GetStatus 查询
 */
void Kalman_Filter_xhatUpdate(KalmanFilter_t *kf);

/**
 * @brief 根据 Kalman 增益更新后验协方差
 *
 * @param[in,out] kf 已初始化且包含有效量测的滤波器对象
 * @return 无，处理状态通过 Kalman_Filter_GetStatus 查询
 */
void Kalman_Filter_P_Update(KalmanFilter_t *kf);

#ifdef __cplusplus
}
#endif
