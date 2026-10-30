#include "module/shoot.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

// 与现有电机在线窗口保持一致，反馈超过 100 ms 或控制周期超过 0.1 s 时两轮撤驱
#define SHOOT_TIMEOUT_US (100000U)
#define SHOOT_MAX_PERIOD_S (0.1f)

static bool Shoot_ConfigValid(const Shoot_Config_t *config, float frequency_hz);

/**
 * @brief 校验双摩擦轮速度环配置与协议电流上限
 *
 * @param[in] config 待校验配置，调用方保证非空
 * @param[in] frequency_hz 控制频率，单位 Hz
 * @return 全部参数合法返回 true，否则返回 false
 */
static bool Shoot_ConfigValid(const Shoot_Config_t *config, float frequency_hz)
{
    const KPID_Params_t *pid = &config->speed_pid;
    return isfinite(frequency_hz) && frequency_hz > 0.0f &&
        isfinite(config->max_speed_rpm) && config->max_speed_rpm > 0.0f &&
        isfinite(config->max_current_a) && config->max_current_a > 0.0f && config->max_current_a <= 20.0f &&
        isfinite(pid->k) && pid->k > 0.0f && isfinite(pid->p) && pid->p >= 0.0f &&
        isfinite(pid->i) && pid->i >= 0.0f && isfinite(pid->d) && pid->d >= 0.0f &&
        isfinite(pid->i_limit) && pid->i_limit >= 0.0f && isfinite(pid->out_limit) &&
        pid->out_limit > 0.0f && pid->out_limit <= config->max_current_a &&
        isfinite(pid->d_cutoff_freq) && pid->d_cutoff_freq < frequency_hz * 0.5f && pid->range == 0.0f;
}

/**
 * @brief 复制摩擦轮配置并初始化两个速度控制器
 *
 * @param[out] shoot 双摩擦轮实例，初始化后不得整体复制
 * @param[in] config 默认配置，空指针时初始化失败
 * @param[in] frequency_hz 控制频率，单位 Hz，须为有限正值
 * @return 成功返回 0，失败返回 -1 并保留两轮零输出
 */
int8_t Shoot_Init(Shoot_t *shoot, const Shoot_Config_t *config, float frequency_hz)
{
    if (shoot == NULL)
    {
        return -1;
    }

    // 初始化失败保留故障状态与零电流，两轮源时间分别从无反馈开始
    memset(shoot, 0, sizeof(*shoot));
    shoot->state = SHOOT_FAULT;
    for (uint8_t index = 0U; index < SHOOT_MOTOR_COUNT; index++)
    {
        shoot->feedback_time_us[index] = UINT64_MAX;
    }
    if (config == NULL || !Shoot_ConfigValid(config, frequency_hz))
    {
        return -1;
    }

    // 两个控制器引用同一份参数，各自独立保存积分与微分历史
    shoot->config = *config;
    shoot->frequency_hz = frequency_hz;
    shoot->applied_speed_pid = config->speed_pid;
    for (uint8_t index = 0U; index < SHOOT_MOTOR_COUNT; index++)
    {
        if (PID_Init(&shoot->speed_pid[index], KPID_MODE_CALC_D, frequency_hz, &shoot->applied_speed_pid) != 0)
        {
            return -1;
        }
    }
    shoot->initialized = true;
    shoot->config_valid = true;
    shoot->state = SHOOT_RELAX;
    return 0;
}

/**
 * @brief 独立接纳两轮新反馈并检查各自的缓存时效
 *
 * @param[in,out] shoot 双摩擦轮实例
 * @param[in] fric1 第一轮新反馈，空指针时仅检查该轮缓存是否超过 100 ms
 * @param[in] fric2 第二轮新反馈，空指针时仅检查该轮缓存是否超过 100 ms
 * @param[in] now_us 读取两路反馈后的完整单调时刻，单位微秒
 * @return 无返回值
 */
void Shoot_UpdateFeedback(Shoot_t *shoot, const MOTOR_RM_Feedback_t *fric1,
                          const MOTOR_RM_Feedback_t *fric2, uint64_t now_us)
{
    if (shoot == NULL)
    {
        return;
    }
    const MOTOR_RM_Feedback_t *feedback[SHOOT_MOTOR_COUNT] =
    {
        fric1, fric2,
    };

    // 两路只在实际取得新帧时恢复完整源时间，空邮箱保留缓存但不刷新源时间
    for (uint8_t index = 0U; index < SHOOT_MOTOR_COUNT; index++)
    {
        const MOTOR_RM_Feedback_t *motor = feedback[index];
        if (motor != NULL)
        {
            const uint32_t age_us = (uint32_t)now_us - motor->last_update_time;
            const uint64_t source_us = now_us >= age_us ? now_us - age_us : UINT64_MAX;
            shoot->feedback_valid[index] = age_us <= SHOOT_TIMEOUT_US && source_us != UINT64_MAX &&
                isfinite(motor->rotor_speed) &&
                (shoot->feedback_time_us[index] == UINT64_MAX || source_us > shoot->feedback_time_us[index]);
            if (shoot->feedback_valid[index])
            {
                shoot->feedback[index] = *motor;
                shoot->feedback_time_us[index] = source_us;
            }
        }

        // 失效标志只由后续新帧恢复，低位计时回绕不得复活过期缓存
        if (shoot->feedback_time_us[index] == UINT64_MAX || now_us < shoot->feedback_time_us[index] ||
            now_us - shoot->feedback_time_us[index] > SHOOT_TIMEOUT_US)
        {
            shoot->feedback_valid[index] = false;
        }
    }
}

/**
 * @brief 推进双轮状态机并执行两个独立的速度闭环
 *
 * @param[in,out] shoot 双摩擦轮实例
 * @param[in] command 双轮连续速度命令，空指针按失能处理
 * @param[in] period_s 实测控制周期，单位 s，须在 (0, 0.1] 内
 * @return 无返回值
 */
void Shoot_Control(Shoot_t *shoot, const Shoot_Command_t *command, float period_s)
{
    if (shoot == NULL)
    {
        return;
    }

    // 本周期先建立两轮零输出，再校验配置、周期与命令
    memset(shoot->current_a, 0, sizeof(shoot->current_a));
    memset(shoot->target_speed_rpm, 0, sizeof(shoot->target_speed_rpm));
    shoot->config_valid = Shoot_ConfigValid(&shoot->config, shoot->frequency_hz);
    const bool enabled = command != NULL && command->enabled;
    const bool valid = shoot->initialized && shoot->config_valid && isfinite(period_s) &&
        period_s > 0.0f && period_s <= SHOOT_MAX_PERIOD_S &&
        (!enabled || (isfinite(command->target_speed_rpm[0]) && isfinite(command->target_speed_rpm[1])));
    const Shoot_State_t previous_state = shoot->state;

    // 两轮共用唯一状态，单轮离线撤驱整对摩擦轮，故障须先失能解除
    switch (shoot->state)
    {
    case SHOOT_RELAX:
    case SHOOT_RUNNING:
    case SHOOT_OFFLINE:
        shoot->state = valid ? SHOOT_RELAX : SHOOT_FAULT;
        if (valid && enabled)
        {
            shoot->state = shoot->feedback_valid[0] && shoot->feedback_valid[1] ? SHOOT_RUNNING : SHOOT_OFFLINE;
        }
        break;
    case SHOOT_FAULT:
        if (valid && !enabled)
        {
            shoot->state = SHOOT_RELAX;
        }
        break;
    default:
        shoot->state = SHOOT_FAULT;
        break;
    }

    // 停机与异常入口清除两轮控制历史，避免下一次恢复沿用旧积分
    if (shoot->state != SHOOT_RUNNING)
    {
        if (shoot->initialized)
        {
            for (uint8_t index = 0U; index < SHOOT_MOTOR_COUNT; index++)
            {
                PID_Reset(&shoot->speed_pid[index]);
            }
        }
        return;
    }

    // 同一周期同时采纳公共参数变更，恢复起控时分别按各轮实测转速预置微分
    const bool pid_changed = memcmp(&shoot->applied_speed_pid, &shoot->config.speed_pid,
                                    sizeof(shoot->applied_speed_pid)) != 0;
    shoot->applied_speed_pid = shoot->config.speed_pid;
    for (uint8_t index = 0U; index < SHOOT_MOTOR_COUNT; index++)
    {
        KPID_t *pid = &shoot->speed_pid[index];
        if (pid_changed || previous_state != SHOOT_RUNNING)
        {
            if (PID_Init(pid, KPID_MODE_CALC_D, shoot->frequency_hz, &shoot->applied_speed_pid) != 0)
            {
                shoot->state = SHOOT_FAULT;
                break;
            }
            pid->last.k_fb = LowPassFilter2p_Reset(&pid->dfilter,
                shoot->applied_speed_pid.k * shoot->feedback[index].rotor_speed);
        }

        // 两路目标独立限幅，最终输出均为转子侧安培
        shoot->target_speed_rpm[index] = AbsClip(command->target_speed_rpm[index], shoot->config.max_speed_rpm);
        const float current_a = PID_Calc(pid, shoot->target_speed_rpm[index],
                                         shoot->feedback[index].rotor_speed, 0.0f, period_s);
        shoot->current_a[index] = AbsClip(current_a, shoot->config.max_current_a);
    }

    // 任一控制器失败时撤销另一轮本周期已算出的电流
    if (shoot->state != SHOOT_RUNNING)
    {
        memset(shoot->current_a, 0, sizeof(shoot->current_a));
        for (uint8_t index = 0U; index < SHOOT_MOTOR_COUNT; index++)
        {
            PID_Reset(&shoot->speed_pid[index]);
        }
    }
}

/**
 * @brief 导出初始化或本周期控制已经确定的两轮电流
 *
 * @param[in] shoot 已调用 Init 或 Control 的双摩擦轮实例，须非空
 * @param[out] output 两轮电流，单位 A，须非空
 * @return 无返回值
 */
void Shoot_DumpOutput(const Shoot_t *shoot, Shoot_Output_t *output)
{
    for (uint8_t index = 0U; index < SHOOT_MOTOR_COUNT; index++)
    {
        output->current_a[index] = shoot->current_a[index];
    }
}
