#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "component/user_math.h"

#include <stdint.h>

#define MIXER_MAX_OUTPUT_COUNT (4U)

/**
 * @brief 混合器运行状态
 */
typedef enum
{
    MIXER_OK = 0,
    MIXER_ERROR = -1,
    MIXER_NULL_ERROR = -2,
    MIXER_LENGTH_ERROR = -3,
    MIXER_MODE_ERROR = -4,
    MIXER_INVALID_VALUE = -5,
    MIXER_UNSUPPORTED = -6,
} Mixer_Status_t;

/**
 * @brief 混合器支持的底盘机械布局
 */
typedef enum
{
    MIXER_MECANUM = 0,
    MIXER_PARLFIX4,
    MIXER_PARLFIX2,
    MIXER_OMNICROSS,
    MIXER_OMNIPLUS,
    MIXER_SINGLE,
    MIXER_MODE_COUNT,
} Mixer_Mode_t;

/*
 * 混合器运行上下文：
 * - mode：当前采用的底盘机械布局
 */
typedef struct
{
    Mixer_Mode_t mode;
} Mixer_t;

/**
 * @brief 初始化底盘运动学混合器
 *
 * @param[out] mixer 待初始化的混合器上下文
 * @param[in] mode 底盘机械布局
 * @return 成功返回 MIXER_OK，失败返回对应状态码
 */
int8_t Mixer_Init(Mixer_t *mixer, Mixer_Mode_t mode);

/**
 * @brief 将归一化底盘运动向量转换为各轮输出轴目标转速
 *
 * 输入采用前向为正的 vx、左向为正的 vy 和俯视逆时针为正的 wz
 * 输出超过单位范围时按最大绝对值整体缩放，再乘以目标转速尺度
 *
 * @param[in] mixer 已初始化的混合器上下文
 * @param[in] move_vector 归一化底盘运动向量，各分量必须为有限值
 * @param[out] output_rpm 各轮输出轴目标转速数组，单位 rpm
 * @param[in] output_count 输出数组元素数量，必须与当前布局的轮数一致
 * @param[in] scale_rpm 归一化输出对应的转速尺度，单位 rpm，必须为有限非负值
 * @return 成功返回 MIXER_OK，失败返回对应状态码并清零可访问输出
 */
int8_t Mixer_Apply(const Mixer_t *mixer, const MoveVector_t *move_vector, float *output_rpm,
                   uint32_t output_count, float scale_rpm);

#ifdef __cplusplus
}
#endif
