/*
 * 底盘功率限制模块实现
 *
 * 按电机功率模型预测四轮消耗功率，超限时按速度误差与指令功率分配功率并反解电流
 */

/* Includes ----------------------------------------------------------------- */
#include "module/power.h"

#include "component/user_math.h"

#include <math.h>

/* Private function prototypes ---------------------------------------------- */
static bool Power_IsConfigValid(const Power_Config_t *config);
static float Power_PredictWheel(const Power_Config_t *config, float current_a, float speed_rad_s);
static float Power_Confidence(const Power_Config_t *config, float sum_error_rad_s);
static float Power_SolveCurrent(const Power_Config_t *config, float speed_rad_s, float calculated_power_w,
                                 float current_a);

/* Exported functions ------------------------------------------------------- */
/**
 * @brief 复制配置并校验功率模型参数
 *
 * @param power 待初始化的功率限制实例
 * @param config 默认配置
 * @return int8_t 成功返回 0，配置非法返回 -1
 */
int8_t Power_Init(Power_t *power, const Power_Config_t *config)
{
    /* 复制配置并初始化观测字段，底盘实例已由调用方清空 */
    power->config = *config;
    power->initialized = Power_IsConfigValid(config);
    power->config_valid = false;
    power->limited = false;
    power->limit_w = 0.0F;
    power->predicted_power_w = 0.0F;
    power->applied_power_w = 0.0F;
    return power->initialized ? 0 : -1;
}

/**
 * @brief 按功率上限重新分配四轮电流，原地写回
 *
 * @param power 功率限制实例
 * @param wheels 四轮功率环输入
 * @param current_a 四轮电流指令，单位：A，原地写回
 * @return 无返回值
 */
void Power_Limit(Power_t *power, const Power_WheelInput_t *wheels, float *current_a)
{
    /* 入口快照：调试器异步写入配置，本周期只使用复制值 */
    const Power_Config_t snapshot = *(const volatile Power_Config_t *)&power->config;

    /* 未启用时完全直通，用于接入前后逐位回归 */
    if (!snapshot.enabled)
    {
        power->config_valid = true;
        power->limited = false;
        power->limit_w = snapshot.limit_w;
        power->predicted_power_w = 0.0F;
        power->applied_power_w = 0.0F;
        return;
    }

    /* 配置校验只在本入口执行一次，非法时四轮零电流 */
    power->config_valid = Power_IsConfigValid(&snapshot);
    if (!power->config_valid)
    {
        for (uint32_t i = 0U; i < POWER_WHEEL_COUNT; i++)
        {
            current_a[i] = 0.0F;
        }
        power->limited = false;
        power->limit_w = 0.0F;
        power->predicted_power_w = 0.0F;
        power->applied_power_w = 0.0F;
        return;
    }
    power->limit_w = snapshot.limit_w;

    /* 逐轮预测本周期功率，负功率轮保持原指令并计入可分配功率 */
    float wheel_power[POWER_WHEEL_COUNT];
    float sum_power = 0.0F;
    float sum_error = 0.0F;
    float sum_positive_power = 0.0F;
    float allocatable_power = snapshot.limit_w;
    for (uint32_t i = 0U; i < POWER_WHEEL_COUNT; i++)
    {
        wheel_power[i] = 0.0F;
        if (!wheels[i].online)
        {
            continue;
        }
        wheel_power[i] = Power_PredictWheel(&snapshot, wheels[i].current_a, wheels[i].measured_speed_rad_s);
        sum_power += wheel_power[i];
        if (wheel_power[i] > 0.0F)
        {
            sum_error += fabsf(wheels[i].target_speed_rad_s - wheels[i].measured_speed_rad_s);
            sum_positive_power += wheel_power[i];
        }
        else
        {
            allocatable_power -= wheel_power[i];
        }
    }
    power->predicted_power_w = sum_power;

    /* 未超限时不改变任何电流指令 */
    if (sum_power <= snapshot.limit_w)
    {
        power->limited = false;
        power->applied_power_w = sum_power;
        return;
    }

    /* 速度误差置信度决定误差分配与指令功率分配的比重 */
    const float confidence = Power_Confidence(&snapshot, sum_error);

    /* 逐轮分配功率并反解电流，离线轮与负功率轮保持原指令 */
    float applied_power = 0.0F;
    for (uint32_t i = 0U; i < POWER_WHEEL_COUNT; i++)
    {
        if (!wheels[i].online)
        {
            continue;
        }
        if (wheel_power[i] <= 0.0F || wheels[i].current_a == 0.0F)
        {
            applied_power += Power_PredictWheel(&snapshot, current_a[i], wheels[i].measured_speed_rad_s);
            continue;
        }

        /* 进入本分支时正功率轮总和必大于零，分配权重之和为一 */
        const float power_weight = wheel_power[i] / sum_positive_power;
        float weight = (1.0F - confidence) * power_weight;
        if (confidence > 0.0F)
        {
            weight += confidence * fabsf(wheels[i].target_speed_rad_s - wheels[i].measured_speed_rad_s) / sum_error;
        }
        const float calculated_power = weight * allocatable_power;

        /* 按原指令方向选取二次方程解，结果统一限幅到该轮电流上限 */
        const float solved_current = Power_SolveCurrent(&snapshot, wheels[i].measured_speed_rad_s, calculated_power,
                                                         wheels[i].current_a);
        current_a[i] = AbsClip(solved_current, wheels[i].max_current_a);
        applied_power += Power_PredictWheel(&snapshot, current_a[i], wheels[i].measured_speed_rad_s);
    }
    power->applied_power_w = applied_power;
    power->limited = true;
}

/* Private functions -------------------------------------------------------- */
/**
 * @brief 校验功率模型配置的全部数值范围
 *
 * @param config 待校验配置
 * @return bool 所有数值有限且范围合法时返回 true
 */
static bool Power_IsConfigValid(const Power_Config_t *config)
{
    return isfinite(config->limit_w) && config->limit_w >= 0.0F &&
           isfinite(config->torque_const) && config->torque_const > 0.0F &&
           isfinite(config->k_abs) && config->k_abs >= 0.0F &&
           isfinite(config->k_sq) && config->k_sq >= 0.0F &&
           isfinite(config->k_cur) && config->k_cur > 0.0F &&
           isfinite(config->k0) && config->k0 >= 0.0F &&
           isfinite(config->error_low_rad_s) && config->error_low_rad_s >= 0.0F &&
           isfinite(config->error_high_rad_s) && config->error_high_rad_s > config->error_low_rad_s;
}

/**
 * @brief 按单轮电流与转速预测该轮消耗功率
 *
 * @param config 已校验配置
 * @param current_a 电流指令，单位：A
 * @param speed_rad_s 输出轴角速度，单位：rad/s
 * @return float 预测功率，单位：W
 */
static float Power_PredictWheel(const Power_Config_t *config, float current_a, float speed_rad_s)
{
    const float abs_speed = fabsf(speed_rad_s);
    return config->torque_const * current_a * speed_rad_s + config->k_abs * abs_speed +
           config->k_sq * speed_rad_s * speed_rad_s + config->k_cur * current_a * current_a +
           config->k0 / (float)POWER_WHEEL_COUNT;
}

/**
 * @brief 按分配功率与当前转速反解目标电流
 *
 * @param config 已校验配置
 * @param speed_rad_s 输出轴角速度，单位：rad/s
 * @param calculated_power_w 分配给该轮的功率，单位：W
 * @param current_a 原电流指令，单位：A，用于确定解的符号
 * @return float 反解电流，单位：A
 */
static float Power_SolveCurrent(const Power_Config_t *config, float speed_rad_s, float calculated_power_w,
                                 float current_a)
{
    const float a = config->k_cur;
    const float b = config->torque_const * speed_rad_s;
    const float c = config->k_abs * fabsf(speed_rad_s) + config->k_sq * speed_rad_s * speed_rad_s +
                    config->k0 / (float)POWER_WHEEL_COUNT - calculated_power_w;
    const float delta = b * b - 4.0F * a * c;
    if (delta > 0.0F)
    {
        const float root = sqrtf(delta);
        return current_a > 0.0F ? (-b + root) / (2.0F * a) : (-b - root) / (2.0F * a);
    }
    return -b / (2.0F * a);
}

/**
 * @brief 按四轮转速误差和计算功率分配置信度
 *
 * @param config 已校验配置
 * @param sum_error_rad_s 四轮目标与实测转速误差绝对值之和，单位：rad/s
 * @return float 置信度，0 表示按指令功率分配，1 表示按误差分配
 */
static float Power_Confidence(const Power_Config_t *config, float sum_error_rad_s)
{
    if (sum_error_rad_s >= config->error_high_rad_s)
    {
        return 1.0F;
    }
    if (sum_error_rad_s > config->error_low_rad_s)
    {
        return (sum_error_rad_s - config->error_low_rad_s) / (config->error_high_rad_s - config->error_low_rad_s);
    }
    return 0.0F;
}
