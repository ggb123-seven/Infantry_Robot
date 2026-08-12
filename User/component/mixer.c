#include "component/mixer.h"

#include <math.h>
#include <stddef.h>

static void Mixer_ResetOutput(float *output, uint32_t output_count);
static bool Mixer_IsModeValid(Mixer_Mode_t mode);
static uint32_t Mixer_GetOutputCount(Mixer_Mode_t mode);

/**
 * @brief 初始化底盘运动学混合器
 *
 * @param[out] mixer 待初始化的混合器上下文
 * @param[in] mode 底盘机械布局
 * @return 成功返回 MIXER_OK，失败返回对应状态码
 */
int8_t Mixer_Init(Mixer_t *mixer, Mixer_Mode_t mode)
{
    if (mixer == NULL)
    {
        return MIXER_NULL_ERROR;
    }
    if (!Mixer_IsModeValid(mode))
    {
        return MIXER_MODE_ERROR;
    }

    mixer->mode = mode;
    return MIXER_OK;
}

/**
 * @brief 将归一化底盘运动向量转换为各轮输出轴目标转速
 *
 * @param[in] mixer 已初始化的混合器上下文
 * @param[in] move_vector 归一化底盘运动向量，各分量必须为有限值
 * @param[out] output_rpm 各轮输出轴目标转速数组，单位 rpm
 * @param[in] output_count 输出数组元素数量，必须与当前布局的轮数一致
 * @param[in] scale_rpm 归一化输出对应的转速尺度，单位 rpm，必须为有限非负值
 * @return 成功返回 MIXER_OK，失败返回对应状态码并清零可访问输出
 */
int8_t Mixer_Apply(const Mixer_t *mixer, const MoveVector_t *move_vector, float *output_rpm,
                   uint32_t output_count, float scale_rpm)
{
    if (output_rpm == NULL)
    {
        return MIXER_NULL_ERROR;
    }

    // 先建立整组零输出，保证后续任一校验或计算失败时保持安全状态
    Mixer_ResetOutput(output_rpm, output_count);
    if (mixer == NULL || move_vector == NULL)
    {
        return MIXER_NULL_ERROR;
    }
    if (!Mixer_IsModeValid(mixer->mode))
    {
        return MIXER_MODE_ERROR;
    }
    if (output_count != Mixer_GetOutputCount(mixer->mode))
    {
        return MIXER_LENGTH_ERROR;
    }
    if (!isfinite(move_vector->vx) || !isfinite(move_vector->vy) || !isfinite(move_vector->wz) ||
        !isfinite(scale_rpm) || scale_rpm < 0.0F)
    {
        return MIXER_INVALID_VALUE;
    }

    // 按底盘机械布局计算相对轮速，数组顺序与对应布局约定保持一致
    switch (mixer->mode)
    {
        case MIXER_MECANUM:
            output_rpm[0] = move_vector->vx - move_vector->vy + move_vector->wz;
            output_rpm[1] = move_vector->vx + move_vector->vy + move_vector->wz;
            output_rpm[2] = -move_vector->vx + move_vector->vy + move_vector->wz;
            output_rpm[3] = -move_vector->vx - move_vector->vy + move_vector->wz;
            break;

        case MIXER_PARLFIX4:
            output_rpm[0] = -move_vector->vx;
            output_rpm[1] = move_vector->vx;
            output_rpm[2] = move_vector->vx;
            output_rpm[3] = -move_vector->vx;
            break;

        case MIXER_PARLFIX2:
            output_rpm[0] = -move_vector->vx;
            output_rpm[1] = move_vector->vx;
            break;

        case MIXER_OMNICROSS:
            // 轮序依次为左前、左后、右后、右前，正转动为俯视逆时针
            output_rpm[0] = -move_vector->vx + move_vector->vy + move_vector->wz;
            output_rpm[1] = -move_vector->vx - move_vector->vy + move_vector->wz;
            output_rpm[2] = move_vector->vx - move_vector->vy + move_vector->wz;
            output_rpm[3] = move_vector->vx + move_vector->vy + move_vector->wz;
            break;

        case MIXER_OMNIPLUS:
            return MIXER_UNSUPPORTED;

        case MIXER_SINGLE:
            output_rpm[0] = move_vector->vx;
            break;

        default:
            return MIXER_MODE_ERROR;
    }

    // 拒绝溢出结果并确定四轮共同使用的缩放比例
    float maximum_absolute_output = 0.0F;
    for (uint32_t output_index = 0U; output_index < output_count; output_index++)
    {
        if (!isfinite(output_rpm[output_index]))
        {
            Mixer_ResetOutput(output_rpm, output_count);
            return MIXER_INVALID_VALUE;
        }
        const float absolute_output = fabsf(output_rpm[output_index]);
        if (absolute_output > maximum_absolute_output)
        {
            maximum_absolute_output = absolute_output;
        }
    }

    // 超过单位范围时整体等比例缩放，避免改变平动与转动的合成方向
    const float normalization = maximum_absolute_output > 1.0F ? maximum_absolute_output : 1.0F;
    for (uint32_t output_index = 0U; output_index < output_count; output_index++)
    {
        output_rpm[output_index] = output_rpm[output_index] / normalization * scale_rpm;
        if (!isfinite(output_rpm[output_index]))
        {
            Mixer_ResetOutput(output_rpm, output_count);
            return MIXER_INVALID_VALUE;
        }
    }
    return MIXER_OK;
}

/**
 * @brief 清零调用方声明可访问的混合器输出
 *
 * @param[out] output 待清零的输出数组
 * @param[in] output_count 调用方声明的数组元素数量
 * @return 无返回值
 */
static void Mixer_ResetOutput(float *output, uint32_t output_count)
{
    if (output == NULL)
    {
        return;
    }

    const uint32_t reset_count = output_count < MIXER_MAX_OUTPUT_COUNT ? output_count : MIXER_MAX_OUTPUT_COUNT;
    for (uint32_t output_index = 0U; output_index < reset_count; output_index++)
    {
        output[output_index] = 0.0F;
    }
}

/**
 * @brief 检查混合器模式是否属于公开枚举范围
 *
 * @param[in] mode 待检查的混合器模式
 * @return 模式合法时返回 true，否则返回 false
 */
static bool Mixer_IsModeValid(Mixer_Mode_t mode)
{
    return mode >= MIXER_MECANUM && mode < MIXER_MODE_COUNT;
}

/**
 * @brief 获取指定底盘布局的输出数量
 *
 * @param[in] mode 混合器模式
 * @return 对应布局的输出数量，模式非法时返回 0
 */
static uint32_t Mixer_GetOutputCount(Mixer_Mode_t mode)
{
    switch (mode)
    {
        case MIXER_PARLFIX2:
            return 2U;

        case MIXER_SINGLE:
            return 1U;

        case MIXER_MECANUM:
        case MIXER_PARLFIX4:
        case MIXER_OMNICROSS:
        case MIXER_OMNIPLUS:
            return 4U;

        default:
            return 0U;
    }
}
