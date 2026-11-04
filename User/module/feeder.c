#include "module/feeder.h"
#include "module/config.h"

#include <math.h>
#include <string.h>

/*
 * 业务参数统一来自 config.c 的 feeder_config，模块只复制到可调实例和本周期快照
 * 环路结构由命令层选择：连发只走速度环，单发使用角度外环与速度内环串级控制
 * 连发时摇杆倍率直接映射输出轴转速目标，倍率为零按待机撤驱，方向由 config.direction 决定
 * 单发每次固定推进一齿，只有实际到位且满足 feed_hz 间隔后才允许下一齿，松杆后完成当前一齿
 * 单发进入及反馈恢复时捕获当前位置并等待回中，摩擦轮使能保持由任务层独立编排
 * 度数仅用于观察，角度外环使用输出轴弧度，速度内环使用减速后的输出轴 rpm，目标差分前馈未标定时默认关闭
 */
#define FEEDER_TWO_PI (6.28318530717958647692f)
#define FEEDER_MAX_CURRENT_A (10.0f)
#define FEEDER_MAX_PERIOD_S (0.1f)
/*
 * 单发到位参数：角度容差由用户确定为输出轴正负 0.5 度
 * 输出轴低速阈值 1 rpm、连续确认 20 毫秒、执行超时 1 秒为本轮方案确认的首版值，待实物复核
 */
#define FEEDER_SINGLE_ERROR_DEG (0.5f)
#define FEEDER_SINGLE_SPEED_RPM (1.0f)
#define FEEDER_SINGLE_SETTLE_S (0.020f)
#define FEEDER_SINGLE_TIMEOUT_S (1.0f)
// 500 Hz 通信允许最多五个周期未取得新帧，超过 10 ms 撤驱且恢复时重捕获当前位置
#define FEEDER_FEEDBACK_TIMEOUT_US (10000U)
// 命令层选择的实际进给模式，仅供观测，上电默认连发速度环
volatile uint8_t feeder_fire_mode = FEEDER_FIRE_MODE_AUTO;
// 单发输出轴目标角，单位度，仅供观测，不再接收手动目标输入
volatile float feeder_angle_target_deg = 0.0f;

// 单发阶段分别表示恢复后等待回中、就绪、执行一齿以及到位后的间隔等待
typedef enum
{
    FEEDER_SINGLE_WAIT_CENTER = 0,
    FEEDER_SINGLE_READY,
    FEEDER_SINGLE_EXECUTING,
    FEEDER_SINGLE_INTERVAL,
} Feeder_SingleState_t;

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
 * single_state 保存单发阶段，只有执行阶段检测本发到位和超时
 * step_elapsed_s 为本发目标发布后的时间，单位秒，到位后保留以限制下一发启动间隔
 * settled_s 为角度及低速条件连续满足的时间，单位秒，条件被打断即清零
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
    Feeder_SingleState_t single_state;
    float step_elapsed_s;
    float settled_s;
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
 * @brief 从上一齿目标推进一齿并开始新的执行阶段
 *
 * @param[in,out] feeder 拨弹实例，调用方保证配置有效且当前角度连续
 * @param[in,out] internal 任务独占私有运行数据，配置为本周期已校验快照
 * @return 无返回值
 */
static void Feeder_StartSingleStep(Feeder_t *feeder, Feeder_Internal_t *internal)
{
    const float step_rad = internal->cycle_config.direction * FEEDER_TWO_PI / internal->cycle_config.tooth_count;
    feeder->target_angle_deg = Feeder_WrapAngle(feeder->target_angle_deg * M_DEG2RAD_MULT + step_rad) * M_RAD2DEG_MULT;
    internal->step_elapsed_s = 0.0f;
    internal->settled_s = 0.0f;
    internal->single_state = FEEDER_SINGLE_EXECUTING;
}

/**
 * @brief 按实际到位状态逐发推进，执行期间不接受追加目标
 *
 * @param[in,out] feeder 拨弹实例，调用方保证运行状态、配置和反馈均有效
 * @param[in,out] internal 任务独占的单发阶段和周期快照
 * @param[in] command 本周期已由命令层确认的单发命令，调用方保证非空
 * @param[in] period_s 已在控制入口校验的实际周期，单位秒
 * @return 无返回值，执行超时或非法阶段使模块进入故障，由唯一输出闸门撤驱
 */
static void Feeder_UpdateSingle(Feeder_t *feeder, Feeder_Internal_t *internal,
                               const Feeder_Command_t *command, float period_s)
{
    // 到位条件只服务于本发完成判定，使用 M2006 输出轴角度和转速，不依赖摩擦轮转速
    switch (internal->single_state)
    {
        case FEEDER_SINGLE_WAIT_CENTER:
            if (command->speed_ratio == 0.0f)
            {
                internal->single_state = FEEDER_SINGLE_READY;
            }
            break;

        case FEEDER_SINGLE_READY:
            if (command->single_fire)
            {
                Feeder_StartSingleStep(feeder, internal);
            }
            break;

        case FEEDER_SINGLE_EXECUTING:
        {
            internal->step_elapsed_s += period_s;
            const float error_deg = CircleError(feeder->target_angle_deg, feeder->measured_angle_deg, 360.0f);
            const bool settled = fabsf(error_deg) <= FEEDER_SINGLE_ERROR_DEG &&
                                 fabsf(feeder->measured_speed_rpm) <= FEEDER_SINGLE_SPEED_RPM;
            internal->settled_s = settled ? internal->settled_s + period_s : 0.0f;
            if (internal->settled_s >= FEEDER_SINGLE_SETTLE_S)
            {
                internal->single_state = FEEDER_SINGLE_INTERVAL;
            }
            else if (internal->step_elapsed_s >= FEEDER_SINGLE_TIMEOUT_S)
            {
                feeder->state = FEEDER_FAULT;
            }
            break;
        }

        case FEEDER_SINGLE_INTERVAL:
            // 保留上一发启动时间以限制重复频率，只在请求仍有效时安排下一发
            internal->step_elapsed_s += period_s;
            if (internal->step_elapsed_s >= 1.0f / internal->cycle_config.feed_hz)
            {
                if (command->single_fire)
                {
                    Feeder_StartSingleStep(feeder, internal);
                }
                else
                {
                    internal->single_state = FEEDER_SINGLE_READY;
                }
            }
            break;

        default:
            feeder->state = FEEDER_FAULT;
            break;
    }
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

    // 单发串级需要角度连续，连发速度环只需要帧新鲜，两类观察量分别置为非数值
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
 * @brief 校验周期配置，按命令档位执行连发速度环或逐发到位串级控制
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
    // 命令缺失按离线档处理，进给模式由命令层唯一生成，调试观测不作为控制输入
    const Feeder_Command_t cycle_command = command != NULL ? *command : (Feeder_Command_t)
    {
        .mode = FEEDER_CMD_OFFLINE,
    };
    const Feeder_CmdMode_t mode = cycle_command.mode;
    const uint8_t fire_mode = cycle_command.fire_mode;
    feeder_fire_mode = fire_mode;
    const bool angle_mode = fire_mode == FEEDER_FIRE_MODE_SINGLE;
    const float feed_ratio = !angle_mode ? cycle_command.speed_ratio : 0.0f;
    const bool cascade = angle_mode && mode == FEEDER_CMD_RUN;
    const bool cascade_changed = cascade != internal->cascade_active;
    // 串级需要角度连续，纯速度环只要求转速反馈新鲜，状态机统一使用该判据
    const bool feedback_drive_valid = cascade ? internal->feedback_valid : isfinite(feeder->measured_speed_rpm);
    const bool feed_requested = mode == FEEDER_CMD_RUN && (cascade || feed_ratio != 0.0f);
    Feeder_State_t next_state;

    // 初始化失败、配置非法或周期异常锁存故障，等待遥控待机档人工解除
    if (!internal->initialized || !feeder->config_valid || !period_valid)
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
        // 停机、恢复或控制方式变化时取消旧单发阶段，在线改参保留本发固定目标和执行超时
        if (next_state != feeder->state || cascade_changed)
        {
            internal->single_state = FEEDER_SINGLE_WAIT_CENTER;
            internal->step_elapsed_s = 0.0f;
            internal->settled_s = 0.0f;
        }
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
                    feeder->target_angle_deg = feeder->measured_angle_deg;
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

    // 本发完成前保持固定目标，长推只在完成及间隔满足后推进下一发，超时由输出闸门撤驱
    Feeder_UpdateSingle(feeder, internal, &cycle_command, period_s);
    feeder_angle_target_deg = feeder->target_angle_deg;
    if (feeder->state != FEEDER_RUNNING)
    {
        return;
    }

    // 单发目标由模块持有，外环和后续计算只读取本周期已校验的配置快照
    const Feeder_Config_t *config = &internal->cycle_config;
    const float target_angle_rad = Feeder_WrapAngle(feeder->target_angle_deg * M_DEG2RAD_MULT);

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
