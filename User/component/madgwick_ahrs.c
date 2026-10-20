/*
 * 计算核心移植自 xrobot-org/MadgwickAHRS，许可证为 Apache-2.0
 * 上游版本：e98a223b84e44ca8718fe8e68b4b63577ec70499
 * 本项目改为纯 C 接口，并增加零梯度保护、有限性检查和缩放归一化
 * 来源及许可证见 docs/imu_xrobot_port.md 与 docs/licenses/xrobot_madgwick_apache_2.0.txt
 */
#include "component/madgwick_ahrs.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/**
 * @brief 将有限非零向量归一化，使用最大分量缩放避免平方和溢出
 *
 * @param[in,out] vector 待归一化向量
 * @param[in] length 向量元素数量
 * @return 归一化成功返回 true，零向量或数值异常返回 false
 */
static bool MadgwickAHRS_Normalize(float *vector, size_t length)
{
    float scale = 0.0F;
    for (size_t index = 0U; index < length; index++)
    {
        if (!isfinite(vector[index]))
        {
            return false;
        }

        scale = fmaxf(scale, fabsf(vector[index]));
    }

    if (scale <= 0.0F)
    {
        return false;
    }

    float norm_squared = 0.0F;
    for (size_t index = 0U; index < length; index++)
    {
        const float scaled = vector[index] / scale;
        norm_squared += scaled * scaled;
    }
    const float norm = sqrtf(norm_squared);
    if (!isfinite(norm) || norm <= 0.0F)
    {
        return false;
    }

    for (size_t index = 0U; index < length; index++)
    {
        vector[index] = (vector[index] / scale) / norm;
    }

    return true;
}

/**
 * @brief 更新六轴 Madgwick 姿态并在所有计算成功后提交四元数
 *
 * @param[in,out] q 单位四元数 [w,x,y,z]，初值通常为 [1,0,0,0]
 * @param[in] gyro 三轴角速度，单位 rad/s，坐标轴须与加速度一致
 * @param[in] accel 三轴加速度，可使用 m/s² 或 g，全零时仅积分角速度
 * @param[in] beta 有限非负反馈增益，XRobot 的 BMI088 示例取 0.033
 * @param[in] dt_s 本次更新间隔，单位秒，须为有限正数
 * @return 成功返回 MADGWICK_AHRS_OK，空指针、非法参数或数值异常返回对应错误码
 */
MadgwickAHRS_Status_t MadgwickAHRS_Update(float q[4], const float gyro[3], const float accel[3],
                                       float beta, float dt_s)
{
    if (q == NULL || gyro == NULL || accel == NULL)
    {
        return MADGWICK_AHRS_NULL;
    }

    if (!isfinite(beta) || beta < 0.0F || !isfinite(dt_s) || dt_s <= 0.0F)
    {
        return MADGWICK_AHRS_INVALID_ARGUMENT;
    }

    for (size_t axis = 0U; axis < 3U; axis++)
    {
        if (!isfinite(gyro[axis]) || !isfinite(accel[axis]))
        {
            return MADGWICK_AHRS_INVALID_ARGUMENT;
        }
    }

    // 仅在局部副本中处理姿态，确保失败时不改写调用方状态
    float q_next[4];
    memcpy(q_next, q, sizeof(q_next));
    if (!MadgwickAHRS_Normalize(q_next, 4U))
    {
        return MADGWICK_AHRS_NUMERIC;
    }

    // 根据角速度计算四元数变化率，输入角速度使用弧度每秒
    float q_dot[4];
    q_dot[0] = 0.5F * (-q_next[1] * gyro[0] - q_next[2] * gyro[1] - q_next[3] * gyro[2]);
    q_dot[1] = 0.5F * (q_next[0] * gyro[0] + q_next[2] * gyro[2] - q_next[3] * gyro[1]);
    q_dot[2] = 0.5F * (q_next[0] * gyro[1] - q_next[1] * gyro[2] + q_next[3] * gyro[0]);
    q_dot[3] = 0.5F * (q_next[0] * gyro[2] + q_next[1] * gyro[1] - q_next[2] * gyro[0]);

    // 仅在加速度有效且梯度非零时应用重力方向反馈
    if (accel[0] != 0.0F || accel[1] != 0.0F || accel[2] != 0.0F)
    {
        float normalized_accel[3] =
        {
            accel[0], accel[1], accel[2]
        };
        if (!MadgwickAHRS_Normalize(normalized_accel, 3U))
        {
            return MADGWICK_AHRS_NUMERIC;
        }

        const float ax = normalized_accel[0];
        const float ay = normalized_accel[1];
        const float az = normalized_accel[2];
        const float q_2q0 = 2.0F * q_next[0];
        const float q_2q1 = 2.0F * q_next[1];
        const float q_2q2 = 2.0F * q_next[2];
        const float q_2q3 = 2.0F * q_next[3];
        const float q_4q0 = 4.0F * q_next[0];
        const float q_4q1 = 4.0F * q_next[1];
        const float q_4q2 = 4.0F * q_next[2];
        const float q_8q1 = 8.0F * q_next[1];
        const float q_8q2 = 8.0F * q_next[2];
        const float q0q0 = q_next[0] * q_next[0];
        const float q1q1 = q_next[1] * q_next[1];
        const float q2q2 = q_next[2] * q_next[2];
        const float q3q3 = q_next[3] * q_next[3];

        // 按 XRobot MadgwickAHRS 固定版本的六轴梯度公式计算重力方向误差
        float gradient[4];
        gradient[0] = q_4q0 * q2q2 + q_2q2 * ax + q_4q0 * q1q1 - q_2q1 * ay;
        gradient[1] = q_4q1 * q3q3 - q_2q3 * ax + 4.0F * q0q0 * q_next[1] - q_2q0 * ay - q_4q1 +
                      q_8q1 * q1q1 + q_8q1 * q2q2 + q_4q1 * az;
        gradient[2] = 4.0F * q0q0 * q_next[2] + q_2q0 * ax + q_4q2 * q3q3 - q_2q3 * ay - q_4q2 +
                      q_8q2 * q1q1 + q_8q2 * q2q2 + q_4q2 * az;
        gradient[3] = 4.0F * q1q1 * q_next[3] - q_2q1 * ax + 4.0F * q2q2 * q_next[3] - q_2q2 * ay;

        // 水平静止时梯度为零，跳过归一化以避免上游实现的除零
        float gradient_scale = 0.0F;
        for (size_t index = 0U; index < 4U; index++)
        {
            if (!isfinite(gradient[index]))
            {
                return MADGWICK_AHRS_NUMERIC;
            }

            gradient_scale = fmaxf(gradient_scale, fabsf(gradient[index]));
        }

        if (gradient_scale > 0.0F)
        {
            if (!MadgwickAHRS_Normalize(gradient, 4U))
            {
                return MADGWICK_AHRS_NUMERIC;
            }

            for (size_t index = 0U; index < 4U; index++)
            {
                q_dot[index] -= beta * gradient[index];
            }
        }
    }

    // 按实测采样间隔积分并归一化，完成有限性检查后才发布新姿态
    for (size_t index = 0U; index < 4U; index++)
    {
        q_next[index] += q_dot[index] * dt_s;
    }
    if (!MadgwickAHRS_Normalize(q_next, 4U))
    {
        return MADGWICK_AHRS_NUMERIC;
    }

    memcpy(q, q_next, sizeof(q_next));
    return MADGWICK_AHRS_OK;
}
