#include "module/feeder.h"
#include "module/config.h"

#include <math.h>
#include <string.h>

/*
 * 业务参数统一来自 config.c 的 feeder_config，模块只复制到可调实例和本周期快照
 * 环路结构由 feeder_fire_mode 决定：连发只走速度环，模式 1 在速度环外套角度外环用于在线调试
 * 连发时摇杆倍率直接映射输出轴转速目标；模式 1 忽略摇杆倍率，右拨杆下位时采用 EmberProbe 目标角
 * 连发方向由 config.direction 决定，倍率为零按待机撤驱，齿数与节拍参数保留给后续单发策略
 * 模式 1 进入角度环时捕获当前位置，不自动推进齿距，单发策略留待后续确定
 * 度数仅用于观察，角度外环使用输出轴弧度，速度内环使用减速后的输出轴 rpm，目标差分前馈未标定时默认关闭
 */
#define FEEDER_TWO_PI (6.28318530717958647692f)
#define FEEDER_MAX_CURRENT_A (10.0f)
#define FEEDER_MAX_PERIOD_S (0.1f)
// 500 Hz 通信允许最多五个周期未取得新帧，超过 10 ms 撤驱且恢复时重捕获当前位置
#define FEEDER_FEEDBACK_TIMEOUT_US (10000U)
// 速度环与角度环调试模式切换入口，上电默认速度环，运行期由 EmberProbe 写入 0 或 1
volatile uint8_t feeder_fire_mode = FEEDER_FIRE_MODE_AUTO;
// 角度环调试目标，单位度，右拨杆下位进入角度环后由 EmberProbe 写入
volatile float feeder_angle_target_deg = 0.0f;

/*
 * 拨弹私有运行数据仅服务于唯一的拨弹实例，EmberProbe 不直接显示其中的控制历史
 * cycle_config 为 Control 每周期校验的持久配置快照
 * feedback_time_us 为驱动反馈源时间低 32 位，单位微秒
 * feedback_angle_rad 为最近一帧输出轴单圈角度，单位 rad
 * initialized 表示两路 PID 初始化成功
 * feedback_valid 表示最近一帧角度连续且未超时
 * cascade_active 表示上一周期角度与速度串级控制处于使能状态
 * position_pid 与 speed_pid 分别保存角度外环和速度内环历史，参数指向 cycle_config
 * frequency_hz 为初始化采样频率，单位 Hz
 */
typedef struct
{
    Feeder_Config_t cycle_config;
    uint32_t feedback_time_us;
    float feedback_angle_rad;
    bool initialized;
    bool feedback_valid;
    bool cascade_active;
    KPID_t position_pid;
    KPID_t speed_pid;
    float frequency_hz;
} Feeder_Internal_t;

static Feeder_Internal_t feeder_internal;

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
    Feeder_Internal_t *internal = &feeder_internal;

    // 初始化私有控制历史并建立公开调参副本，未取得反馈前观测量保持非数值
    memset(internal, 0, sizeof(*internal));
    *feeder = (Feeder_t)
    {
        .config = feeder_config,
        .state = FEEDER_FAULT,
        .measured_angle_deg = NAN,
        .measured_speed_rpm = NAN,
    };
    internal->cycle_config = feeder_config;
    internal->frequency_hz = frequency_hz;
    // 配置只在统一入口校验，两个控制器长期引用模块私有的持久快照
    feeder->config_valid = isfinite(frequency_hz) && frequency_hz > 0.0f &&
                           Feeder_ConfigValid(&internal->cycle_config, frequency_hz);
    if (!feeder->config_valid)
    {
        return -1;
    }

    // 外环用实测角速度提供微分，避免位置在 0 与 2π 之间环绕时产生虚假的微分脉冲
    internal->initialized = PID_Init(&internal->position_pid, KPID_MODE_SET_D, frequency_hz,
                                     &internal->cycle_config.position_pid) == 0 &&
                            PID_Init(&internal->speed_pid, KPID_MODE_CALC_D, frequency_hz,
                                     &internal->cycle_config.speed_pid) == 0;
    feeder->state = internal->initialized ? FEEDER_RELAX : FEEDER_FAULT;
    return internal->initialized ? 0 : -1;
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
    Feeder_Internal_t *internal = &feeder_internal;

    // 角度外环依赖角度连续性，新帧的有效性由驱动给出的角度连续性标志决定
    if (received)
    {
        internal->feedback_time_us = feedback->last_update_time;
        internal->feedback_angle_rad = feedback->rotor_single_angle;
        internal->feedback_valid = feedback->angle_valid;
        feeder->measured_speed_rpm = feedback->rotor_speed;
    }

    // 驱动保存微秒时间低 32 位，无符号差值允许正常回绕并拒绝启动时的未来帧
    const uint32_t age_us = (uint32_t)now_us - internal->feedback_time_us;
    const bool frame_fresh = age_us <= FEEDER_FEEDBACK_TIMEOUT_US && now_us >= age_us;
    internal->feedback_valid = internal->feedback_valid && frame_fresh;

    // 角度环调试需要角度连续，连发速度环只需要帧新鲜，两类观察量分别置为非数值
    if (internal->feedback_valid)
    {
        feeder->measured_angle_deg = internal->feedback_angle_rad * M_RAD2DEG_MULT;
    }
    else
    {
        feeder->measured_angle_deg = NAN;
    }
    if (!frame_fresh)
    {
        feeder->measured_speed_rpm = NAN;
    }
}

/**
 * @brief 校验周期配置，按命令档位推进状态机并执行速度环或手动目标角串级控制
 *
 * @param[in,out] feeder 拨弹实例，调用方保证非空
 * @param[in] command 本周期拨弹命令，无命令时传 NULL，按离线档处理且不解除故障锁存
 * @param[in] period_s 实测控制周期，单位秒，合法范围 (0,0.1]
 * @return 无返回值
 */
void Feeder_Control(Feeder_t *feeder, const Feeder_Command_t *command, float period_s)
{
    Feeder_Internal_t *internal = &feeder_internal;

    // 每周期入口只读取一次可调配置，后续控制使用本周期快照
    const Feeder_Config_t previous_config = internal->cycle_config;
    internal->cycle_config = *(const volatile Feeder_Config_t *)&feeder->config;
    feeder->config_valid = Feeder_ConfigValid(&internal->cycle_config, internal->frequency_hz);
    const bool config_changed = memcmp(&previous_config, &internal->cycle_config, sizeof(previous_config)) != 0;
    const bool period_valid = isfinite(period_s) && period_s > 0.0f && period_s <= FEEDER_MAX_PERIOD_S;
    // 命令缺失按离线档处理，角度模式隔离摇杆倍率，连发模式保留摇杆正负方向
    const Feeder_CmdMode_t mode = command != NULL ? command->mode : FEEDER_CMD_OFFLINE;
    // EmberProbe 输入在控制入口只读取一次，角度环已运行时目标非法按故障处理
    const uint8_t fire_mode = feeder_fire_mode;
    const bool fire_mode_valid = fire_mode == FEEDER_FIRE_MODE_AUTO || fire_mode == FEEDER_FIRE_MODE_SINGLE;
    const bool angle_mode = fire_mode == FEEDER_FIRE_MODE_SINGLE;
    const float feed_ratio = !angle_mode && command != NULL ? command->speed_ratio : 0.0f;
    const bool cascade = angle_mode && mode == FEEDER_CMD_RUN;
    const bool cascade_changed = cascade != internal->cascade_active;
    float angle_target_deg = feeder_angle_target_deg;
    const bool angle_target_valid = !cascade || !internal->cascade_active || isfinite(angle_target_deg);
    // 串级需要角度连续，纯速度环只要求转速反馈新鲜，状态机统一使用该判据
    const bool feedback_drive_valid = cascade ? internal->feedback_valid : isfinite(feeder->measured_speed_rpm);
    const bool feed_requested = mode == FEEDER_CMD_RUN && (cascade || feed_ratio != 0.0f);
    Feeder_State_t next_state;

    // 停机、配置非法、周期异常或在线目标非法锁存故障，等待遥控待机档人工解除
    if (!internal->initialized || !feeder->config_valid || !period_valid || !fire_mode_valid || !angle_target_valid)
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

    // 状态、配置或环路结构变化时执行入口动作并重置控制历史
    const bool restart = next_state != feeder->state || config_changed || cascade_changed;
    if (restart)
    {
        PID_Reset(&internal->position_pid);
        PID_Reset(&internal->speed_pid);
        feeder->target_speed_rpm = 0.0f;
        if (next_state == FEEDER_RUNNING)
        {
            // 起控与在线改参时按本周期实测转速预置微分，避免沿用停机前的历史
            LowPassFilter2p_Init(&internal->speed_pid.dfilter, internal->frequency_hz,
                internal->cycle_config.speed_pid.d_cutoff_freq);
            internal->speed_pid.last.k_fb = LowPassFilter2p_Reset(&internal->speed_pid.dfilter,
                internal->cycle_config.speed_pid.k * feeder->measured_speed_rpm);
            // 串级才维护目标角，纯速度环模式的目标由摇杆倍率每周期直接给出
            if (cascade)
            {
                LowPassFilter2p_Init(&internal->position_pid.dfilter, internal->frequency_hz,
                    internal->cycle_config.position_pid.d_cutoff_freq);
                // 首次使能或反馈恢复时捕获当前位置，防止切换模式或拨杆下位产生自动转动
                if (!internal->cascade_active || feeder->state != FEEDER_RUNNING)
                {
                    angle_target_deg = feeder->measured_angle_deg;
                    feeder_angle_target_deg = angle_target_deg;
                }
            }
        }
    }
    feeder->state = next_state;
    internal->cascade_active = cascade;
    if (feeder->state != FEEDER_RUNNING)
    {
        return;
    }

    // 连发只走速度环：摇杆倍率直接映射输出轴转速，角度外环完全不参与
    if (!cascade)
    {
        const Feeder_Config_t *config = &internal->cycle_config;
        const float previous_target_rpm = feeder->target_speed_rpm;
        feeder->target_speed_rpm = config->direction * feed_ratio * config->max_speed_rpm;
        const float feedforward_a = (feeder->target_speed_rpm - previous_target_rpm) *
            config->speed_feedforward_gain_a_per_rpm;
        const float current_a = PID_Calc(&internal->speed_pid, feeder->target_speed_rpm,
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

    // 角度调试模式每周期采用一次 EmberProbe 目标快照，不执行自动齿距推进
    const Feeder_Config_t *config = &internal->cycle_config;
    feeder->target_angle_deg = angle_target_deg;
    const float target_angle_rad = Feeder_WrapAngle(angle_target_deg * M_DEG2RAD_MULT);

    // 角度外环用实测角速度提供微分，避免角度在 0 与 2π 之间环绕时产生虚假的微分脉冲
    const float measured_speed_rad_s = feeder->measured_speed_rpm * (FEEDER_TWO_PI / 60.0f);
    const float derivative = config->position_pid.k * measured_speed_rad_s;
    // 目标推进与微分换算属于本层计算链，在进入外环前统一拒绝溢出
    if (!isfinite(target_angle_rad) || !isfinite(derivative))
    {
        feeder->state = FEEDER_FAULT;
        return;
    }

    // 外环按环绕误差输出 rad/s，换算为输出轴 rpm 后限幅，交给已整定的速度内环
    const float target_rad_s = PID_Calc(&internal->position_pid, target_angle_rad,
        internal->feedback_angle_rad, derivative, period_s);
    const float previous_target_rpm = feeder->target_speed_rpm;
    feeder->target_speed_rpm = AbsClip(target_rad_s * (60.0f / FEEDER_TWO_PI), config->max_speed_rpm);
    const float feedforward_a = (feeder->target_speed_rpm - previous_target_rpm) *
        config->speed_feedforward_gain_a_per_rpm;
    const float current_a = PID_Calc(&internal->speed_pid, feeder->target_speed_rpm,
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
