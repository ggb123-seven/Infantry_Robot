#include "module/trigger.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

// 与现有电机在线窗口保持一致，反馈超过 100 ms 或控制周期超过 0.1 s 时撤驱
#define TRIGGER_TIMEOUT_US (100000U)
#define TRIGGER_MAX_PERIOD_S (0.1f)

static bool Trigger_ConfigValid(const Trigger_Config_t *config, float frequency_hz);

/**
 * @brief 校验拨弹速度环配置与协议电流上限
 *
 * @param[in] config 待校验的配置，调用方保证非空
 * @param[in] frequency_hz 控制频率，单位 Hz
 * @return 全部参数合法返回 true，否则返回 false
 */
static bool Trigger_ConfigValid(const Trigger_Config_t *config, float frequency_hz)
{
    const KPID_Params_t *pid = &config->speed_pid;
    return isfinite(frequency_hz) && frequency_hz > 0.0f &&
        isfinite(config->max_speed_rpm) && config->max_speed_rpm > 0.0f &&
        isfinite(config->max_current_a) && config->max_current_a > 0.0f && config->max_current_a <= 10.0f &&
        isfinite(pid->k) && pid->k > 0.0f && isfinite(pid->p) && pid->p >= 0.0f &&
        isfinite(pid->i) && pid->i >= 0.0f && isfinite(pid->d) && pid->d >= 0.0f &&
        isfinite(pid->i_limit) && pid->i_limit >= 0.0f && isfinite(pid->out_limit) &&
        pid->out_limit > 0.0f && pid->out_limit <= config->max_current_a &&
        isfinite(pid->d_cutoff_freq) && pid->d_cutoff_freq < frequency_hz * 0.5f && pid->range == 0.0f;
}

/**
 * @brief 复制拨弹配置并初始化速度控制器
 *
 * @param[out] trigger 拨弹实例，初始化后不得整体复制
 * @param[in] config 默认配置，空指针时初始化失败
 * @param[in] frequency_hz 控制频率，单位 Hz，须为有限正值
 * @return 成功返回 0，失败返回 -1 并保留零输出
 */
int8_t Trigger_Init(Trigger_t *trigger, const Trigger_Config_t *config, float frequency_hz)
{
    if (trigger == NULL)
    {
        return -1;
    }

    // 清空旧实例，所有失败路径均从未初始化、无反馈和零电流开始
    memset(trigger, 0, sizeof(*trigger));
    trigger->feedback_time_us = UINT64_MAX;
    trigger->state = TRIGGER_FAULT;
    if (config == NULL || !Trigger_ConfigValid(config, frequency_hz))
    {
        return -1;
    }

    // 控制器引用实例内参数副本，后续调参通过比较副本重新初始化
    trigger->config = *config;
    trigger->frequency_hz = frequency_hz;
    trigger->applied_speed_pid = config->speed_pid;
    if (PID_Init(&trigger->speed_pid, KPID_MODE_CALC_D, frequency_hz, &trigger->applied_speed_pid) != 0)
    {
        return -1;
    }
    trigger->initialized = true;
    trigger->config_valid = true;
    trigger->state = TRIGGER_RELAX;
    return 0;
}

/**
 * @brief 接纳拨弹新反馈并检查缓存时效
 *
 * @param[in,out] trigger 拨弹实例
 * @param[in] feedback 本周期新反馈，空指针时仅检查缓存是否超过 100 ms
 * @param[in] now_us 读取反馈后的完整单调时刻，单位微秒
 * @return 无返回值
 */
void Trigger_UpdateFeedback(Trigger_t *trigger, const MOTOR_RM_Feedback_t *feedback, uint64_t now_us)
{
    if (trigger == NULL)
    {
        return;
    }

    // 只在取得新帧时恢复完整源时间，禁止对历史缓存持续做低位时间模减
    if (feedback != NULL)
    {
        const uint32_t age_us = (uint32_t)now_us - feedback->last_update_time;
        const uint64_t source_us = now_us >= age_us ? now_us - age_us : UINT64_MAX;
        trigger->feedback_valid = age_us <= TRIGGER_TIMEOUT_US && source_us != UINT64_MAX &&
            isfinite(feedback->rotor_speed) &&
            (trigger->feedback_time_us == UINT64_MAX || source_us > trigger->feedback_time_us);
        if (trigger->feedback_valid)
        {
            trigger->feedback = *feedback;
            trigger->feedback_time_us = source_us;
        }
    }

    // 时间回退或源反馈超时均撤销有效标志，只有后续新帧能够恢复
    if (trigger->feedback_time_us == UINT64_MAX || now_us < trigger->feedback_time_us ||
        now_us - trigger->feedback_time_us > TRIGGER_TIMEOUT_US)
    {
        trigger->feedback_valid = false;
    }
}

/**
 * @brief 推进拨弹状态机并执行速度闭环
 *
 * @param[in,out] trigger 拨弹实例
 * @param[in] command 连续速度命令，空指针按失能处理
 * @param[in] period_s 实测控制周期，单位 s，须在 (0, 0.1] 内
 * @return 无返回值
 */
void Trigger_Control(Trigger_t *trigger, const Trigger_Command_t *command, float period_s)
{
    if (trigger == NULL)
    {
        return;
    }

    // 先建立零输出，再校验本周期输入，避免异常路径复用上一周期电流
    trigger->current_a = 0.0f;
    trigger->target_speed_rpm = 0.0f;
    trigger->config_valid = Trigger_ConfigValid(&trigger->config, trigger->frequency_hz);
    const bool enabled = command != NULL && command->enabled;
    const bool valid = trigger->initialized && trigger->config_valid && isfinite(period_s) &&
        period_s > 0.0f && period_s <= TRIGGER_MAX_PERIOD_S &&
        (!enabled || isfinite(command->target_speed_rpm));
    const Trigger_State_t previous_state = trigger->state;

    // 状态转换集中于此：失联可随新反馈恢复，故障须先失能才能解除
    switch (trigger->state)
    {
    case TRIGGER_RELAX:
    case TRIGGER_RUNNING:
    case TRIGGER_OFFLINE:
        trigger->state = valid ? TRIGGER_RELAX : TRIGGER_FAULT;
        if (valid && enabled)
        {
            trigger->state = trigger->feedback_valid ? TRIGGER_RUNNING : TRIGGER_OFFLINE;
        }
        break;
    case TRIGGER_FAULT:
        if (valid && !enabled)
        {
            trigger->state = TRIGGER_RELAX;
        }
        break;
    default:
        trigger->state = TRIGGER_FAULT;
        break;
    }

    // 停止、失联与故障入口清除控制历史，零电流保持到状态机允许运行
    if (trigger->state != TRIGGER_RUNNING)
    {
        if (trigger->initialized)
        {
            PID_Reset(&trigger->speed_pid);
        }
        return;
    }

    // 调参和重新起控时清除历史，并按实测转速预置微分，避免恢复瞬间的微分冲击
    const bool pid_changed = memcmp(&trigger->applied_speed_pid, &trigger->config.speed_pid,
                                    sizeof(trigger->applied_speed_pid)) != 0;
    if (pid_changed || previous_state != TRIGGER_RUNNING)
    {
        trigger->applied_speed_pid = trigger->config.speed_pid;
        if (PID_Init(&trigger->speed_pid, KPID_MODE_CALC_D, trigger->frequency_hz,
                     &trigger->applied_speed_pid) != 0)
        {
            trigger->state = TRIGGER_FAULT;
            return;
        }
        trigger->speed_pid.last.k_fb = LowPassFilter2p_Reset(&trigger->speed_pid.dfilter,
            trigger->applied_speed_pid.k * trigger->feedback.rotor_speed);
    }

    // 目标与最终电流分别限幅，输出单位始终为转子侧安培
    trigger->target_speed_rpm = AbsClip(command->target_speed_rpm, trigger->config.max_speed_rpm);
    const float current_a = PID_Calc(&trigger->speed_pid, trigger->target_speed_rpm,
                                     trigger->feedback.rotor_speed, 0.0f, period_s);
    if (!isfinite(current_a))
    {
        trigger->state = TRIGGER_FAULT;
        PID_Reset(&trigger->speed_pid);
        return;
    }
    trigger->current_a = AbsClip(current_a, trigger->config.max_current_a);
}

/**
 * @brief 导出拨弹电流，未初始化或非运行状态时输出零
 *
 * @param[in] trigger 拨弹实例，允许为空
 * @param[out] output 拨弹电流输出，单位 A，允许为空
 * @return 无返回值
 */
void Trigger_DumpOutput(const Trigger_t *trigger, Trigger_Output_t *output)
{
    if (output == NULL)
    {
        return;
    }
    output->current_a = trigger != NULL && trigger->initialized && trigger->state == TRIGGER_RUNNING &&
        isfinite(trigger->current_a) ? trigger->current_a : 0.0f;
}
