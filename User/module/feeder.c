#include "module/feeder.h"
#include "module/config.h"

#include <math.h>
#include <string.h>

/*
 * 业务参数统一来自 config.c 的 feeder_config，模块只复制到可调实例和本周期快照
 * 当前调试阶段旁路位置外环，按命令层给出的归一化比例乘速度上限得到输出轴 rpm 目标，逐齿参数和位置 PID 保留待恢复
 * 度数仅用于观察，速度内环使用减速后的输出轴 rpm，目标差分前馈未标定时默认关闭
 */
#define FEEDER_TWO_PI (6.28318530717958647692f)
#define FEEDER_MAX_CURRENT_A (10.0f)
#define FEEDER_MAX_PERIOD_S (0.1f)
// 500 Hz 通信允许最多五个周期未取得新帧，超过 10 ms 撤驱且恢复时重捕获位置
#define FEEDER_FEEDBACK_TIMEOUT_US (10000U)

/**
 * @brief 在配置入口校验单个控制器的公共参数
 *
 * @param[in] pid 待校验参数，调用方保证非空
 * @param[in] frequency_hz 控制采样频率，单位 Hz
 * @return 增益、积分和滤波参数合法时返回 true
 */
static bool Feeder_PidConfigValid(const KPID_Params_t *pid, float frequency_hz)
{
    return isfinite(pid->k) && pid->k > 0.0f && isfinite(pid->p) && pid->p >= 0.0f &&
           isfinite(pid->i) && pid->i >= 0.0f && isfinite(pid->d) && pid->d >= 0.0f &&
           isfinite(pid->i_limit) && pid->i_limit >= 0.0f &&
           isfinite(pid->out_limit) && pid->out_limit > 0.000001f &&
           isfinite(pid->d_cutoff_freq) && pid->d_cutoff_freq < frequency_hz * 0.5f;
}

/**
 * @brief 在初始化和控制入口集中校验配置快照
 *
 * @param[in] config 待校验快照，调用方保证非空
 * @param[in] frequency_hz 已确认的控制采样频率，单位 Hz
 * @return 齿距、调度和控制器量程均合法时返回 true
 */
static bool Feeder_ConfigValid(const Feeder_Config_t *config, float frequency_hz)
{
    return isfinite(config->tooth_count) && config->tooth_count >= 2.0f &&
           isfinite(config->feed_hz) && config->feed_hz > 0.0f && config->feed_hz <= frequency_hz &&
           (config->direction == -1.0f || config->direction == 1.0f) &&
           isfinite(config->max_speed_rpm) && config->max_speed_rpm > 0.0f &&
           isfinite(config->max_current_a) && config->max_current_a > 0.0f &&
           config->max_current_a <= FEEDER_MAX_CURRENT_A &&
           Feeder_PidConfigValid(&config->position_pid, frequency_hz) &&
           config->position_pid.out_limit <= config->max_speed_rpm * (FEEDER_TWO_PI / 60.0f) &&
           config->position_pid.range == FEEDER_TWO_PI &&
           Feeder_PidConfigValid(&config->speed_pid, frequency_hz) &&
           config->speed_pid.out_limit <= config->max_current_a && config->speed_pid.range == 0.0f &&
           isfinite(config->speed_feedforward_gain_a_per_rpm) && config->speed_feedforward_gain_a_per_rpm >= 0.0f;
}

/**
 * @brief 复制默认配置并建立指向持久快照的 PID 参数引用
 *
 * @param[out] feeder 拨弹实例，调用方保证非空
 * @param[in] frequency_hz 控制频率，单位 Hz，须为有限正值
 * @return 初始化成功返回 0，否则返回 -1
 */
int8_t Feeder_Init(Feeder_t *feeder, float frequency_hz)
{
    // 初始化从安全状态开始，配置副本保留给运行期调参
    *feeder = (Feeder_t)
    {
        .config = feeder_config,
        .cycle_config = feeder_config,
        .frequency_hz = frequency_hz,
        .state = FEEDER_FAULT,
        .speed_pid.measured_speed_rpm = NAN,
    };
    // 配置只在统一入口校验，两个控制器长期引用实例内持久快照
    feeder->config_valid = isfinite(frequency_hz) && frequency_hz > 0.0f &&
                           Feeder_ConfigValid(&feeder->cycle_config, frequency_hz);
    if (!feeder->config_valid)
    {
        return -1;
    }

    // 用实测速度提供微分，避免位置在 0 与 2π 之间环绕时产生虚假的微分脉冲
    feeder->initialized = PID_Init(&feeder->position_pid, KPID_MODE_SET_D, frequency_hz,
                                   &feeder->cycle_config.position_pid) == 0 &&
                          PID_Init(&feeder->speed_pid.controller, KPID_MODE_CALC_D, frequency_hz,
                                   &feeder->cycle_config.speed_pid) == 0;
    feeder->state = feeder->initialized ? FEEDER_RELAX : FEEDER_FAULT;
    return feeder->initialized ? 0 : -1;
}

/**
 * @brief 在反馈入口接收驱动已校验完整性的物理量
 *
 * @param[in,out] feeder 拨弹实例，调用方保证非空
 * @param[in] feedback 输出轴反馈，调用方保证非空
 * @param[in] received 驱动本周期取得新帧时为 true
 * @param[in] now_us 读取反馈后的完整单调微秒时间
 * @return 无返回值
 */
void Feeder_UpdateFeedback(Feeder_t *feeder, const MOTOR_RM_Feedback_t *feedback, bool received, uint64_t now_us)
{
    // 纯速度模式接收驱动已校验的完整新帧，角度连续性丢失不影响独立的转速反馈
    if (received)
    {
        feeder->feedback = *feedback;
        feeder->feedback_valid = true;
        feeder->measured_angle_deg = feedback->angle_valid ? feedback->rotor_single_angle * M_RAD2DEG_MULT : NAN;
        feeder->speed_rad_s = feedback->rotor_speed * (FEEDER_TWO_PI / 60.0f);
        feeder->measured_speed_deg_s = feedback->rotor_speed * 6.0f;
    }

    // 驱动保存微秒时间低 32 位，无符号差值允许正常回绕并拒绝启动时的未来帧
    const uint32_t age_us = (uint32_t)now_us - feeder->feedback.last_update_time;
    feeder->feedback_valid = feeder->feedback_valid && age_us <= FEEDER_FEEDBACK_TIMEOUT_US && now_us >= age_us;
    feeder->speed_pid.measured_speed_rpm = feeder->feedback_valid ? feeder->feedback.rotor_speed : NAN;
}

/**
 * @brief 校验周期配置，按命令档位推进状态机并执行速度控制
 *
 * @param[in,out] feeder 拨弹实例，调用方保证非空
 * @param[in] command 本周期拨弹命令，无命令时传 NULL，按离线档处理且不解除故障锁存
 * @param[in] period_s 实测控制周期，单位秒，合法范围 (0,0.1]
 * @return 无返回值
 */
void Feeder_Control(Feeder_t *feeder, const Feeder_Command_t *command, float period_s)
{
    // 每周期入口只读取一次可调配置，后续控制使用本周期快照
    const Feeder_Config_t previous_config = feeder->cycle_config;
    feeder->cycle_config = *(const volatile Feeder_Config_t *)&feeder->config;
    feeder->config_valid = Feeder_ConfigValid(&feeder->cycle_config, feeder->frequency_hz);
    const bool config_changed = memcmp(&previous_config, &feeder->cycle_config, sizeof(previous_config)) != 0;
    const bool period_valid = isfinite(period_s) && period_s > 0.0f && period_s <= FEEDER_MAX_PERIOD_S;
    // 命令缺失按离线档处理，只有显式待机档能解除故障锁存
    const Feeder_CmdMode_t mode = command != NULL ? command->mode : FEEDER_CMD_OFFLINE;
    const float speed_ratio = command != NULL ? command->speed_ratio : 0.0f;
    Feeder_State_t next_state;

    // 停机、配置非法和周期异常锁存故障，等待遥控待机档人工解除
    if (!feeder->initialized || !feeder->config_valid || !period_valid)
    {
        next_state = FEEDER_FAULT;
    }
    else
    {
        switch (feeder->state)
        {
            case FEEDER_RELAX:
            case FEEDER_RUNNING:
            case FEEDER_OFFLINE:
                next_state = mode != FEEDER_CMD_RUN ? FEEDER_RELAX :
                             feeder->feedback_valid ? FEEDER_RUNNING : FEEDER_OFFLINE;
                break;
            case FEEDER_FAULT:
                // 故障只能由待机档解除，断链或反馈恢复不会自动绕过故障
                next_state = mode == FEEDER_CMD_RELAX ? FEEDER_RELAX : FEEDER_FAULT;
                break;
            default:
                next_state = FEEDER_FAULT;
                break;
        }
    }

    // 入口动作只在状态或配置变化时重置控制历史
    const bool restart = next_state != feeder->state || config_changed;
    if (restart)
    {
        PID_Reset(&feeder->position_pid);
        PID_Reset(&feeder->speed_pid.controller);
        feeder->phase_s = 0.0f;
        feeder->speed_pid.target_speed_rpm = 0.0f;
        feeder->target_speed_deg_s = 0.0f;
        if (next_state == FEEDER_RUNNING)
        {
            // 恢复与在线改参时按本周期实测速度预置微分，避免沿用停机前的历史
            LowPassFilter2p_Init(&feeder->position_pid.dfilter, feeder->frequency_hz,
                feeder->cycle_config.position_pid.d_cutoff_freq);
            LowPassFilter2p_Init(&feeder->speed_pid.controller.dfilter, feeder->frequency_hz,
                feeder->cycle_config.speed_pid.d_cutoff_freq);
            feeder->speed_pid.controller.last.k_fb = LowPassFilter2p_Reset(&feeder->speed_pid.controller.dfilter,
                feeder->cycle_config.speed_pid.k * feeder->speed_pid.measured_speed_rpm);
            feeder->target_angle_rad = feeder->feedback.rotor_single_angle;
            feeder->target_angle_deg = feeder->measured_angle_deg;
        }
    }
    feeder->state = next_state;
    if (feeder->state != FEEDER_RUNNING)
    {
        return;
    }

    // 速度调试阶段不推进逐齿目标，也不计算位置 PID，直接把命令比例换算成本周期 rpm 目标
    const Feeder_Config_t *config = &feeder->cycle_config;
    const float previous_target_rpm = feeder->speed_pid.target_speed_rpm;
    feeder->speed_pid.target_speed_rpm = speed_ratio * config->max_speed_rpm;
    feeder->target_speed_deg_s = feeder->speed_pid.target_speed_rpm * 6.0f;
    const float feedforward_a = (feeder->speed_pid.target_speed_rpm - previous_target_rpm) *
                               config->speed_feedforward_gain_a_per_rpm;
    const float current_a = PID_Calc(&feeder->speed_pid.controller, feeder->speed_pid.target_speed_rpm,
        feeder->speed_pid.measured_speed_rpm, 0.0f, period_s) + feedforward_a;

    // 前馈乘法和叠加在控制计算出口检查一次，异常只改变状态，由输出闸门统一撤驱
    if (!isfinite(current_a))
    {
        feeder->state = FEEDER_FAULT;
        return;
    }
    feeder->output_current_a = AbsClip(current_a, config->max_current_a);
}

/**
 * @brief 以运行状态作为唯一输出闸门
 *
 * @param[in] feeder 拨弹实例，调用方保证非空
 * @return 运行时的转子侧电流，其他状态为 0 A
 */
float Feeder_DumpOutput(const Feeder_t *feeder)
{
    return feeder->state == FEEDER_RUNNING ? feeder->output_current_a : 0.0f;
}
