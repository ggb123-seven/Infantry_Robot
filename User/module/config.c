#include "module/config.h"

/*
 * 云台默认配置：
 * - yaw 零点 97.0752° 取自用户实测，安装关系改变后须重新标定
 * - yaw 目标角范围 [0°, 360°]，沿用原角度环线性目标范围
 * - 速度上限依据 GM6020 使用说明 20231013 的 320 rpm 空载转速换算，约 33.51 rad/s
 * - 电流上限 0.5 A，低于手册额定电流 1.62 A，为当前调试阶段的软件保护
 * - 速度环 Kp=0.06、Ki=0.177 为已整定值，积分限幅取电流上限除以 Ki，约 2.825 rad
 * - 角度环增益初值为零待整定，积分限幅暂取 1.0 rad·s，增大 Ki 前须同步评估
 * - pitch 尚未接入控制，enabled 为 false，零点和角度范围为占位值，启用前须实测标定
 * - 反馈超时 90 ms：320 rpm 时 90 ms 内转动小于半圈，保证编码器角度可唯一展开
 */
static const Gimbal_Config_t gimbal_config =
{
    .yaw =
    {
        .enabled = true,
        .reversed = false,
        .zero_encoder_deg = 97.0752f,
        .min_angle_deg = 0.0f,
        .max_angle_deg = 360.0f,
        .max_speed_rad_s = 320.0f * 0.10471975512f,
        .max_current_a = 0.5f,
        .angle_pid =
        {
            .k = 1.0f,
            .p = 0.0f,
            .i = 0.0f,
            .d = 0.0f,
            .i_limit = 1.0f,
            .out_limit = 320.0f * 0.10471975512f,
            .d_cutoff_freq = 0.0f,
            .range = 0.0f,
        },
        .speed_pid =
        {
            .k = 1.0f,
            .p = 0.06f,
            .i = 0.177f,
            .d = 0.0f,
            .i_limit = 0.5f / 0.177f,
            .out_limit = 0.5f,
            .d_cutoff_freq = 0.0f,
            .range = 0.0f,
        },
    },
    .pitch =
    {
        .enabled = false,
        .reversed = false,
        .zero_encoder_deg = 0.0f,
        .min_angle_deg = -20.0f,
        .max_angle_deg = 20.0f,
        .max_speed_rad_s = 320.0f * 0.10471975512f,
        .max_current_a = 0.5f,
        .angle_pid =
        {
            .k = 1.0f,
            .i_limit = 1.0f,
            .out_limit = 320.0f * 0.10471975512f,
        },
        .speed_pid =
        {
            .k = 1.0f,
            .out_limit = 0.5f,
        },
    },
    .feedback_timeout_s = 0.09f,
};

/**
 * @brief 获取整机云台默认配置
 *
 * @return 指向只读云台配置的指针，生命周期为整个程序运行期
 */
const Gimbal_Config_t *Config_GetGimbal(void)
{
    return &gimbal_config;
}
