#include "module/feeder.h"
#include "module/config.h"

#include <math.h>
#include <string.h>

/*
 * 业务参数统一来自 config.c 的 feeder_config，模块只复制到可调实例和本周期快照
 * 环路结构由 feeder_pid_mode 决定：仅速度环用于连发，串级在速度环外再套角度外环用于逐齿与单发定位
 * 仅速度环时摇杆倍率直接映射输出轴转速目标；串级时角度外环推进环绕目标角并输出 rpm 目标
 * 每秒进给齿数为 config.feed_hz 与进给倍率的乘积，方向由 config.direction 决定，倍率为零按待机撤驱
 * 单发模式不累计相位，由拨杆上升沿推进一个齿距并等待到位，超时判定堵转
 * 度数仅用于观察，角度外环使用输出轴弧度，速度内环使用减速后的输出轴 rpm，目标差分前馈未标定时默认关闭
 */
#define FEEDER_TWO_PI (6.28318530717958647692f)
#define FEEDER_MAX_CURRENT_A (10.0f)
#define FEEDER_MAX_PERIOD_S (0.1f)
// 500 Hz 通信允许最多五个周期未取得新帧，超过 10 ms 撤驱且恢复时重捕获齿位
#define FEEDER_FEEDBACK_TIMEOUT_US (10000U)
// 单发到位窗口与超时：8 齿每发 45°，4° 窗口约占齿距的 8.9%，0.3 s 未到位判定堵转
#define FEEDER_SINGLE_IN_POSITION_RAD (0.06981317f)
#define FEEDER_SINGLE_TIMEOUT_S (0.3f)
// 单发与自动连发模式切换入口，上电默认自动，运行期由 EmberProbe 写入 0 或 1
volatile uint8_t feeder_fire_mode = FEEDER_FIRE_MODE_AUTO;
// 环路结构切换入口，上电默认仅速度环，单发模式强制串级而忽略该开关
volatile uint8_t feeder_pid_mode = FEEDER_PID_MODE_SPEED_ONLY;

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
 * @brief 将有限角度映射到一圈内
 *
 * @param[in] angle_rad 调用方保证有限的输出轴角度，单位 rad
 * @return [0,2π) 内的角度，单位 rad
 */
static float Feeder_WrapAngle(float angle_rad)
{
    const float wrapped = fmodf(angle_rad, FEEDER_TWO_PI);
    return wrapped < 0.0f ? wrapped + FEEDER_TWO_PI : wrapped;
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
    // 初始化从安全状态开始，配置副本保留给运行期调参，未取得反馈前观察量均为非数值
    *feeder = (Feeder_t)
    {
        .config = feeder_config,
        .cycle_config = feeder_config,
        .frequency_hz = frequency_hz,
        .state = FEEDER_FAULT,
        .measured_angle_rad = NAN,
        .measured_angle_deg = NAN,
        .measured_speed_rpm = NAN,
        .measured_speed_rad_s = NAN,
        // 单发默认参数在实例内初始化，运行期可被 EmberProbe 覆盖
        .single_in_position_rad = FEEDER_SINGLE_IN_POSITION_RAD,
        .single_timeout_s = FEEDER_SINGLE_TIMEOUT_S,
    };
    // 配置只在统一入口校验，两个控制器长期引用实例内持久快照
    feeder->config_valid = isfinite(frequency_hz) && frequency_hz > 0.0f &&
                           Feeder_ConfigValid(&feeder->cycle_config, frequency_hz);
    if (!feeder->config_valid)
    {
        return -1;
    }

    // 外环用实测角速度提供微分，避免位置在 0 与 2π 之间环绕时产生虚假的微分脉冲
    feeder->initialized = PID_Init(&feeder->position_pid, KPID_MODE_SET_D, frequency_hz,
                                   &feeder->cycle_config.position_pid) == 0 &&
                          PID_Init(&feeder->speed_pid, KPID_MODE_CALC_D, frequency_hz,
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
    // 角度外环依赖角度连续性，新帧的有效性由驱动给出的角度连续性标志决定
    if (received)
    {
        feeder->feedback = *feedback;
        feeder->feedback_valid = feedback->angle_valid;
        feeder->measured_speed_rpm = feedback->rotor_speed;
        feeder->measured_speed_rad_s = feedback->rotor_speed * (FEEDER_TWO_PI / 60.0f);
    }

    // 驱动保存微秒时间低 32 位，无符号差值允许正常回绕并拒绝启动时的未来帧
    const uint32_t age_us = (uint32_t)now_us - feeder->feedback.last_update_time;
    const bool frame_fresh = age_us <= FEEDER_FEEDBACK_TIMEOUT_US && now_us >= age_us;
    feeder->feedback_valid = feeder->feedback_valid && frame_fresh;

    // 单发定位需要角度连续，连发速度环只需要帧新鲜，两类观察量分别置为非数值
    if (feeder->feedback_valid)
    {
        feeder->measured_angle_rad = feeder->feedback.rotor_single_angle;
        feeder->measured_angle_deg = feeder->feedback.rotor_single_angle * M_RAD2DEG_MULT;
    }
    else
    {
        feeder->measured_angle_rad = NAN;
        feeder->measured_angle_deg = NAN;
    }
    if (!frame_fresh)
    {
        feeder->measured_speed_rpm = NAN;
        feeder->measured_speed_rad_s = NAN;
    }
}

/**
 * @brief 校验周期配置，按命令档位推进状态机并执行逐齿位置与速度串级控制
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
    // 命令缺失按离线档处理，倍率取绝对值，零倍率视同待机以避免角度环把转子锁死在齿位
    const Feeder_CmdMode_t mode = command != NULL ? command->mode : FEEDER_CMD_OFFLINE;
    const float feed_ratio = command != NULL ? fabsf(command->speed_ratio) : 0.0f;
    // 单发参数同属在线调参入口，只在控制入口校验一次，非法值按故障处理
    const uint8_t fire_mode = feeder_fire_mode;
    const bool single_mode = fire_mode == FEEDER_FIRE_MODE_SINGLE;
    const bool single_config_valid = !single_mode ||
        (isfinite(feeder->single_in_position_rad) && feeder->single_in_position_rad > 0.0f &&
         feeder->single_in_position_rad < FEEDER_TWO_PI &&
         isfinite(feeder->single_timeout_s) && feeder->single_timeout_s > 0.0f);
    // 拨杆电平只在命令有效时更新，断链恢复后不会把保持下位误判成新的触发
    const bool trigger_known = command != NULL && mode != FEEDER_CMD_OFFLINE;
    const bool trigger_level = trigger_known && command->trigger_level;
    const bool single_trigger = single_mode && trigger_known && trigger_level && !feeder->trigger_level_last;
    if (trigger_known)
    {
        feeder->trigger_level_last = trigger_level;
    }
    // 单发只在摇杆回中时接受触发，执行中推动摇杆由自动连续进给接管
    const bool single_start = single_trigger && !feeder->single_active && feed_ratio <= 0.0f &&
                              feeder->feedback_valid;
    if (feed_ratio > 0.0f)
    {
        feeder->single_active = false;
        feeder->single_elapsed_s = 0.0f;
    }
    // 单发起发与执行必须串级，其余情况按环路开关决定，连发默认只走速度环
    const bool cascade = feeder->single_active || single_start || feeder_pid_mode == FEEDER_PID_MODE_CASCADE;
    // 串级需要角度连续，纯速度环只要求转速反馈新鲜，状态机统一使用该判据
    const bool feedback_drive_valid = cascade ? feeder->feedback_valid : isfinite(feeder->measured_speed_rpm);
    const bool feed_requested = mode == FEEDER_CMD_RUN &&
                                (feed_ratio > 0.0f || feeder->single_active || single_start);
    Feeder_State_t next_state;

    // 停机、配置非法、周期异常或单发参数非法锁存故障，等待遥控待机档人工解除
    if (!feeder->initialized || !feeder->config_valid || !period_valid || !single_config_valid)
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
                next_state = !feed_requested ? FEEDER_RELAX :
                             feedback_drive_valid ? FEEDER_RUNNING : FEEDER_OFFLINE;
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
        PID_Reset(&feeder->speed_pid);
        feeder->phase_s = 0.0;
        feeder->target_speed_rpm = 0.0f;
        if (next_state == FEEDER_RUNNING)
        {
            // 起控与在线改参时按本周期实测转速预置微分，避免沿用停机前的历史
            LowPassFilter2p_Init(&feeder->speed_pid.dfilter, feeder->frequency_hz,
                feeder->cycle_config.speed_pid.d_cutoff_freq);
            feeder->speed_pid.last.k_fb = LowPassFilter2p_Reset(&feeder->speed_pid.dfilter,
                feeder->cycle_config.speed_pid.k * feeder->measured_speed_rpm);
            // 串级才维护目标角与相位，仅速度环模式的目标由摇杆倍率每周期直接给出
            if (cascade)
            {
                LowPassFilter2p_Init(&feeder->position_pid.dfilter, feeder->frequency_hz,
                    feeder->cycle_config.position_pid.d_cutoff_freq);
                // 单发起控目标固定推进一个齿距，自动逐齿档从当前齿位起控并预置一个满速进给间隔
                feeder->target_angle_rad = feeder->measured_angle_rad;
                feeder->target_angle_deg = feeder->measured_angle_deg;
                if (single_start)
                {
                    feeder->target_angle_rad = Feeder_WrapAngle(feeder->target_angle_rad +
                        feeder->cycle_config.direction * FEEDER_TWO_PI / feeder->cycle_config.tooth_count);
                    feeder->target_angle_deg = feeder->target_angle_rad * M_RAD2DEG_MULT;
                    feeder->single_active = true;
                    feeder->single_elapsed_s = 0.0f;
                }
                else
                {
                    feeder->phase_s = 1.0 / (double)feeder->cycle_config.feed_hz;
                }
            }
        }
        else
        {
            // 任何非运行状态都终止单发，恢复后需要重新拨杆触发
            feeder->single_active = false;
            feeder->single_elapsed_s = 0.0f;
        }
    }
    feeder->state = next_state;
    if (feeder->state != FEEDER_RUNNING)
    {
        return;
    }

    // 连发只走速度环：摇杆倍率直接映射输出轴转速，角度外环完全不参与
    if (!cascade)
    {
        const Feeder_Config_t *config = &feeder->cycle_config;
        const float previous_target_rpm = feeder->target_speed_rpm;
        feeder->target_speed_rpm = config->direction * feed_ratio * config->max_speed_rpm;
        const float feedforward_a = (feeder->target_speed_rpm - previous_target_rpm) *
            config->speed_feedforward_gain_a_per_rpm;
        const float current_a = PID_Calc(&feeder->speed_pid, feeder->target_speed_rpm,
            feeder->measured_speed_rpm, 0.0f, period_s) + feedforward_a;
        // 前馈乘法和叠加在控制计算出口检查一次，异常只改变状态，由输出闸门统一撤驱
        if (!isfinite(current_a))
        {
            feeder->state = FEEDER_FAULT;
            return;
        }
        feeder->output_current_a = AbsClip(current_a, config->max_current_a);
        return;
    }

    // 单发执行中只等待到位或超时：到位当周期回待机，超时锁存故障等待人工解除
    if (feeder->single_active)
    {
        feeder->single_elapsed_s += period_s;
        const float position_error = CircleError(feeder->target_angle_rad, feeder->measured_angle_rad,
                                                 FEEDER_TWO_PI);
        if (fabsf(position_error) <= feeder->single_in_position_rad)
        {
            feeder->single_active = false;
            feeder->single_elapsed_s = 0.0f;
            feeder->state = FEEDER_RELAX;
            return;
        }
        if (feeder->single_elapsed_s > feeder->single_timeout_s)
        {
            feeder->single_active = false;
            feeder->single_elapsed_s = 0.0f;
            feeder->state = FEEDER_FAULT;
            return;
        }
    }

    // 逐齿推进：按进给倍率缩放每秒齿数，用双精度秒累积保留不足一齿的余量
    // 单发模式摇杆回中时目标已由触发推进，不再按相位累积
    const Feeder_Config_t *config = &feeder->cycle_config;
    if (feed_ratio > 0.0f)
    {
        const double interval_s = 1.0 / ((double)config->feed_hz * (double)feed_ratio);
        if (!restart)
        {
            // 实测周期来自整数微秒，恢复其分辨率并以双精度累积，避免长时间运行后整齿边界漂移
            feeder->phase_s += (double)roundf(period_s * 1000000.0f) * 0.000001;
        }
        const double teeth = floor((feeder->phase_s + 0.000000001) / interval_s);
        feeder->phase_s -= teeth * interval_s;
        feeder->target_angle_rad = Feeder_WrapAngle(feeder->target_angle_rad +
            config->direction * (float)teeth * FEEDER_TWO_PI / config->tooth_count);
        feeder->target_angle_deg = feeder->target_angle_rad * M_RAD2DEG_MULT;
    }

    // 角度外环用实测角速度提供微分，避免角度在 0 与 2π 之间环绕时产生虚假的微分脉冲
    const float derivative = config->position_pid.k * feeder->measured_speed_rad_s;
    // 目标推进与微分换算属于本层计算链，在进入外环前统一拒绝溢出
    if (!isfinite(feeder->target_angle_rad) || !isfinite(derivative))
    {
        feeder->state = FEEDER_FAULT;
        return;
    }

    // 外环按环绕误差输出 rad/s，换算为输出轴 rpm 后限幅，交给已整定的速度内环
    const float target_rad_s = PID_Calc(&feeder->position_pid, feeder->target_angle_rad,
        feeder->feedback.rotor_single_angle, derivative, period_s);
    const float previous_target_rpm = feeder->target_speed_rpm;
    feeder->target_speed_rpm = AbsClip(target_rad_s * (60.0f / FEEDER_TWO_PI), config->max_speed_rpm);
    const float feedforward_a = (feeder->target_speed_rpm - previous_target_rpm) *
        config->speed_feedforward_gain_a_per_rpm;
    const float current_a = PID_Calc(&feeder->speed_pid, feeder->target_speed_rpm,
        feeder->measured_speed_rpm, 0.0f, period_s) + feedforward_a;

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
