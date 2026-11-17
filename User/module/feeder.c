/*
 * 拨弹控制模块
 *
 * 消费命令层拨弹命令与电机反馈，按环路使能执行角度环、速度环或串级，输出转子电流
 */

/* Includes ----------------------------------------------------------------- */
#include "module/feeder.h"
#include "module/config.h"

#include <math.h>
#include <string.h>

/* Private define ----------------------------------------------------------- */
/* 业务参数统一来自 config.c 的 feeder_config，模块只复制到可调实例和本周期快照 */
#define FEEDER_TWO_PI (6.28318530717958647692f) /* 2π，弧度换算常数 */
#define FEEDER_MAX_CURRENT_A (10.0f)            /* 转子电流硬上限，单位：A */
#define FEEDER_MAX_PERIOD_S (0.1f)              /* 控制周期合法上限，单位：s */

/* 单发到位判据由用户确定为输出轴角度容差、低速阈值和连续确认时间，待实物复核 */
#define FEEDER_SINGLE_ERROR_DEG (0.5f)  /* 角度容差，单位：度 */
#define FEEDER_SINGLE_SPEED_RPM (1.0f)  /* 低速阈值，单位：rpm */
#define FEEDER_SINGLE_SETTLE_S (0.020f) /* 连续确认时间，单位：s */

/* 每组五发由用户确定，按实际逐齿到位计数，不按时间估算 */
#define FEEDER_BURST_SHOTS (5U) /* 每组发数 */

/* 500 Hz 通信最多容许五个周期缺少新反馈，超过 10 毫秒即撤销反馈资格 */
#define FEEDER_FEEDBACK_TIMEOUT_US (10000U) /* 反馈帧龄上限，单位：us */

volatile uint8_t feeder_fire_mode = FEEDER_FIRE_MODE_BURST; /* 实际进给模式，仅供观测 */
volatile float feeder_angle_target_deg = 0.0f;              /* 单发输出轴目标角，单位：度，仅本模块写入 */

/* 在线使能接口由 Control 入口逐周期读取，默认角度环直接输出电流且速度环关闭 */
volatile bool feeder_angle_loop_enabled = true;  /* 角度环使能，默认开启 */
volatile bool feeder_speed_loop_enabled = false; /* 速度环使能，默认关闭 */

/* Private typedef ---------------------------------------------------------- */
/* 有效环路状态由命令类型和使能接口集中选择，切换时取消旧目标并重置控制历史 */
typedef enum
{
    FEEDER_LOOP_DISABLED = 0, /* 全部关闭，直接待机 */
    FEEDER_LOOP_ANGLE,        /* 单角度环，直接输出电流 A */
    FEEDER_LOOP_SPEED,        /* 仅速度环，连发倍率进给 */
    FEEDER_LOOP_CASCADE,      /* 角度速度串级，角度环输出 rad/s */
} Feeder_LoopMode_t;

/* 定量进给阶段，分别表示恢复后待命、就绪、执行一齿、齿间等待和完成后等待释放 */
typedef enum
{
    FEEDER_SINGLE_WAIT_CENTER = 0, /* 待命，等待开火请求 */
    FEEDER_SINGLE_READY,           /* 就绪，可启动下一齿 */
    FEEDER_SINGLE_EXECUTING,       /* 执行中，等待实际到位 */
    FEEDER_SINGLE_INTERVAL,        /* 齿间等待，限制组内启动频率 */
    FEEDER_SINGLE_WAIT_RELEASE,    /* 组完成，等待请求释放 */
} Feeder_SingleState_t;

/* 拨弹私有运行数据，仅服务于唯一实例，EmberProbe 不直接显示其中的控制历史 */
typedef struct
{
    Feeder_Config_t cycle_config;      /* Control 每周期校验的配置快照 */
    uint32_t feedback_time_us;         /* 反馈源时间低 32 位，单位：us */
    float feedback_angle_rad;          /* 最近一帧输出轴单圈角度，单位：rad */
    bool initialized;                  /* 两路 PID 初始化成功 */
    bool feedback_valid;               /* 最近一帧角度连续且源时间未超时 */
    KPID_t position_pid;               /* 角度外环历史，参数指向 cycle_config */
    KPID_t speed_pid;                  /* 速度内环历史，参数指向 cycle_config */
    float frequency_hz;                /* 初始化采样频率，单位：Hz */
    Feeder_SingleState_t single_state; /* 逐齿阶段，完成后保留等待释放 */
    float step_elapsed_s;              /* 本发目标发布后时间，单位：s */
    float settled_s;                   /* 到位条件连续满足时间，单位：s */
} Feeder_Internal_t;

/* Private variables -------------------------------------------------------- */
static Feeder_Internal_t feeder_internal; /* 唯一实例的私有运行数据 */

/* 组内剩余未启动齿数与上周期选择，不增加既有结构体字段 */
static uint8_t feeder_steps_remaining;            /* 剩余未启动齿数 */
static uint8_t feeder_active_fire_mode;           /* 上周期实际进给模式 */
static Feeder_LoopMode_t feeder_active_loop_mode; /* 上周期有效控制结构 */

/* Private function --------------------------------------------------------- */
/**
 * @brief 根据命令类型与环路使能选择本周期唯一控制结构
 *
 * @param angle_mode 命令是否采用逐齿角度目标
 * @param angle_enabled 本周期角度环使能快照
 * @param speed_enabled 本周期速度环使能快照
 * @return Feeder_LoopMode_t 选中的控制结构
 */
static Feeder_LoopMode_t Feeder_SelectLoop(bool angle_mode, bool angle_enabled, bool speed_enabled)
{
    if (angle_mode && angle_enabled)
    {
        return speed_enabled ? FEEDER_LOOP_CASCADE : FEEDER_LOOP_ANGLE;
    }
    return speed_enabled ? FEEDER_LOOP_SPEED : FEEDER_LOOP_DISABLED;
}

/**
 * @brief 在配置入口校验单个控制器的公共参数
 *
 * @param pid 待校验参数
 * @param frequency_hz 控制采样频率，单位：Hz
 * @return bool 增益、积分和滤波参数合法时返回 true
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
 * @param config 待校验快照
 * @param frequency_hz 已确认的控制采样频率，单位：Hz
 * @param loop_mode 本周期控制结构，串级模式的角度输出量程按速度上限校验
 * @return bool 齿距、调度和控制器量程均合法时返回 true
 */
static bool Feeder_ConfigValid(const Feeder_Config_t *config, float frequency_hz, Feeder_LoopMode_t loop_mode)
{
    return isfinite(config->tooth_count) && config->tooth_count >= 2.0f &&
           isfinite(config->feed_hz) && config->feed_hz > 0.0f && config->feed_hz <= frequency_hz &&
           (config->direction == -1.0f || config->direction == 1.0f) &&
           isfinite(config->max_speed_rpm) && config->max_speed_rpm > 0.0f &&
           isfinite(config->max_current_a) && config->max_current_a > 0.0f &&
           config->max_current_a <= FEEDER_MAX_CURRENT_A &&
           Feeder_PidConfigValid(&config->position_pid, frequency_hz) &&
           (loop_mode != FEEDER_LOOP_CASCADE ||
            config->position_pid.out_limit <= config->max_speed_rpm * (FEEDER_TWO_PI / 60.0f)) &&
           config->position_pid.range == FEEDER_TWO_PI &&
           Feeder_PidConfigValid(&config->speed_pid, frequency_hz) &&
           config->speed_pid.out_limit <= config->max_current_a && config->speed_pid.range == 0.0f &&
           isfinite(config->speed_feedforward_gain_a_per_rpm) && config->speed_feedforward_gain_a_per_rpm >= 0.0f;
}

/**
 * @brief 将有限角度映射到一圈内
 *
 * @param angle_rad 输出轴角度，单位：rad
 * @return float [0,2π) 内的角度，单位：rad
 */
static float Feeder_WrapAngle(float angle_rad)
{
    const float wrapped = fmodf(angle_rad, FEEDER_TWO_PI);
    return wrapped < 0.0f ? wrapped + FEEDER_TWO_PI : wrapped;
}

/**
 * @brief 从上一齿目标推进一齿并开始新的执行阶段
 *
 * @param feeder 拨弹实例
 * @param internal 任务独占私有运行数据，配置为本周期已校验快照
 * @return 无返回值
 */
static void Feeder_StartSingleStep(Feeder_t *feeder, Feeder_Internal_t *internal)
{
    /* 调用方保证尚有未启动齿数，每启动一齿只消费一次 */
    feeder_steps_remaining--;
    const float step_rad = internal->cycle_config.direction * FEEDER_TWO_PI / internal->cycle_config.tooth_count;
    feeder->target_angle_deg = Feeder_WrapAngle(feeder->target_angle_deg * M_DEG2RAD_MULT + step_rad) * M_RAD2DEG_MULT;
    internal->step_elapsed_s = 0.0f;
    internal->settled_s = 0.0f;
    internal->single_state = FEEDER_SINGLE_EXECUTING;
}

/**
 * @brief 按实际到位状态逐发推进，执行期间不接受追加目标
 *
 * @param feeder 拨弹实例
 * @param internal 任务独占的单发阶段和周期快照
 * @param command 本周期已由命令层确认的定量命令
 * @param period_s 已在控制入口校验的实际周期，单位：s
 * @return 无返回值
 */
static void Feeder_UpdateSingle(Feeder_t *feeder, Feeder_Internal_t *internal,
                               const Feeder_Command_t *command, float period_s)
{
    /* 到位条件只服务于本发完成判定，不依赖摩擦轮转速 */
    switch (internal->single_state)
    {
        case FEEDER_SINGLE_WAIT_CENTER:
        case FEEDER_SINGLE_READY:
            /* 按进给模式装载组内齿数，随后启动第一齿 */
            feeder_steps_remaining = command->fire_mode == FEEDER_FIRE_MODE_BURST ? FEEDER_BURST_SHOTS : 1U;
            Feeder_StartSingleStep(feeder, internal);
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
                /* 最后一齿实际到位就完成整组，立即撤驱且保留完成状态，不等待下一齿间隔 */
                if (feeder_steps_remaining == 0U)
                {
                    internal->single_state = FEEDER_SINGLE_WAIT_RELEASE;
                    feeder->state = FEEDER_RELAX;
                    feeder->target_speed_rpm = 0.0f;
                    PID_Reset(&internal->position_pid);
                    PID_Reset(&internal->speed_pid);
                }
                else
                {
                    internal->single_state = FEEDER_SINGLE_INTERVAL;
                }
            }
            break;
        }

        case FEEDER_SINGLE_INTERVAL:
            /* 调用方保证该阶段尚有未启动齿数，沿用上一发起点限制组内启动频率 */
            internal->step_elapsed_s += period_s;
            if (internal->step_elapsed_s >= 1.0f / internal->cycle_config.feed_hz)
            {
                Feeder_StartSingleStep(feeder, internal);
            }
            break;

        case FEEDER_SINGLE_WAIT_RELEASE:
            break;

        default:
            feeder->state = FEEDER_FAULT;
            break;
    }
}

/* Exported functions ------------------------------------------------------- */
/**
 * @brief 复制默认拨弹配置并建立指向持久快照的 PID 参数引用
 *
 * @param feeder 拨弹实例
 * @param frequency_hz 控制频率，单位：Hz
 * @return int8_t 初始化成功返回 0，否则返回 -1
 */
int8_t Feeder_Init(Feeder_t *feeder, float frequency_hz)
{
    Feeder_Internal_t *internal = &feeder_internal;

    /* 初始化私有控制历史并建立公开调参副本，未取得反馈前观测量保持非数值 */
    memset(internal, 0, sizeof(*internal));
    feeder_steps_remaining = 0U;
    feeder_active_fire_mode = FEEDER_FIRE_MODE_BURST;
    feeder_active_loop_mode = FEEDER_LOOP_DISABLED;
    *feeder = (Feeder_t)
    {
        .config = feeder_config,
        .state = FEEDER_FAULT,
        .measured_angle_deg = NAN,
        .measured_speed_rpm = NAN,
    };
    internal->cycle_config = feeder_config;
    internal->frequency_hz = frequency_hz;
    /* 配置只在统一入口校验，两个控制器长期引用模块私有的持久快照 */
    const Feeder_LoopMode_t loop_mode = Feeder_SelectLoop(true, feeder_angle_loop_enabled, feeder_speed_loop_enabled);
    feeder->config_valid = isfinite(frequency_hz) && frequency_hz > 0.0f &&
                           Feeder_ConfigValid(&internal->cycle_config, frequency_hz, loop_mode);
    if (!feeder->config_valid)
    {
        return -1;
    }

    /* 单角度环直接输出安培，在持久快照应用最终电流边界，避免积分按更大的旧速度量程累积 */
    if (loop_mode == FEEDER_LOOP_ANGLE)
    {
        internal->cycle_config.position_pid.out_limit =
            fminf(internal->cycle_config.position_pid.out_limit, internal->cycle_config.max_current_a);
    }

    /* 外环用实测角速度提供微分，避免位置在 0 与 2π 之间环绕时产生虚假的微分脉冲 */
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
 * @param feeder 拨弹实例
 * @param feedback 输出轴反馈
 * @param received 驱动本周期取得新帧时为 true
 * @param now_us 读取反馈后的完整单调微秒时间
 * @return 无返回值
 */
void Feeder_UpdateFeedback(Feeder_t *feeder, const MOTOR_RM_Feedback_t *feedback, bool received, uint64_t now_us)
{
    Feeder_Internal_t *internal = &feeder_internal;

    /* 角度外环依赖角度连续性，新帧的有效性由驱动给出的角度连续性标志决定 */
    if (received)
    {
        internal->feedback_time_us = feedback->last_update_time;
        internal->feedback_angle_rad = feedback->rotor_single_angle;
        internal->feedback_valid = feedback->angle_valid;
        feeder->measured_speed_rpm = feedback->rotor_speed;
    }

    /* 驱动源时间只保存低 32 位，无符号差值支持回绕，并拒绝启动时晚于当前时间的帧 */
    const uint32_t age_us = (uint32_t)now_us - internal->feedback_time_us;
    const bool frame_fresh = age_us <= FEEDER_FEEDBACK_TIMEOUT_US && now_us >= age_us;
    internal->feedback_valid = internal->feedback_valid && frame_fresh;

    /* 角度连续性与反馈帧龄在此统一生效，下游只读取本次输入边界的结果 */
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
 * @brief 校验周期配置，按在线使能执行速度环、单角度环或逐发串级控制
 *
 * @param feeder 拨弹实例
 * @param command 本周期拨弹命令，无命令时传 NULL，按离线档处理
 * @param period_s 实测控制周期，单位：s
 * @return 无返回值
 */
void Feeder_Control(Feeder_t *feeder, const Feeder_Command_t *command, float period_s)
{
    Feeder_Internal_t *internal = &feeder_internal;

    /* 命令缺失按离线处理，环路使能各读取一次，与命令类型共同确定本周期唯一控制结构 */
    const bool angle_enabled = feeder_angle_loop_enabled;
    const bool speed_enabled = feeder_speed_loop_enabled;
    const Feeder_Command_t cycle_command = command != NULL ? *command : (Feeder_Command_t)
    {
        .mode = FEEDER_CMD_OFFLINE,
    };
    const Feeder_CmdMode_t mode = cycle_command.mode;
    const uint8_t fire_mode = cycle_command.fire_mode;
    feeder_fire_mode = fire_mode;
    const bool angle_mode = fire_mode != FEEDER_FIRE_MODE_AUTO;
    const Feeder_LoopMode_t loop_mode = Feeder_SelectLoop(angle_mode, angle_enabled, speed_enabled);
    const bool loop_mode_changed = loop_mode != feeder_active_loop_mode;

    /* 配置只在入口集中校验，直接电流模式先应用有效限幅，再与上周期已应用快照比较 */
    const Feeder_Config_t previous_config = internal->cycle_config;
    internal->cycle_config = *(const volatile Feeder_Config_t *)&feeder->config;
    feeder->config_valid = Feeder_ConfigValid(&internal->cycle_config, internal->frequency_hz, loop_mode);
    if (feeder->config_valid && loop_mode == FEEDER_LOOP_ANGLE)
    {
        internal->cycle_config.position_pid.out_limit =
            fminf(internal->cycle_config.position_pid.out_limit, internal->cycle_config.max_current_a);
    }
    const bool config_changed = memcmp(&previous_config, &internal->cycle_config, sizeof(previous_config)) != 0;
    const bool period_valid = isfinite(period_s) && period_s > 0.0f && period_s <= FEEDER_MAX_PERIOD_S;
    const bool fire_mode_changed = fire_mode != feeder_active_fire_mode;
    /* 完成状态属于当前射击组，不随反馈离线和恢复变化，模式或环路切换建立新组 */
    const bool group_complete = internal->single_state == FEEDER_SINGLE_WAIT_RELEASE &&
                                !loop_mode_changed && !fire_mode_changed;
    const float feed_ratio = !angle_mode ? cycle_command.speed_ratio : 0.0f;
    const bool position_active = mode == FEEDER_CMD_RUN &&
        (loop_mode == FEEDER_LOOP_ANGLE || loop_mode == FEEDER_LOOP_CASCADE);
    /* 角度控制需要连续角度，纯速度环只要求转速新鲜，关闭所有有效环路时直接待机 */
    const bool feedback_drive_valid = position_active ? internal->feedback_valid : isfinite(feeder->measured_speed_rpm);
    const bool feed_requested = mode == FEEDER_CMD_RUN &&
        ((position_active && cycle_command.single_fire && !group_complete) ||
         (loop_mode == FEEDER_LOOP_SPEED && feed_ratio != 0.0f));
    Feeder_State_t next_state;

    /* 初始化失败、配置非法或周期异常锁存故障，等待遥控待机档人工解除 */
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
                /* 故障只能由待机档解除，断链或反馈恢复不会自动绕过故障 */
                next_state = mode == FEEDER_CMD_RELAX ? FEEDER_RELAX : FEEDER_FAULT;
                break;

            default:
                next_state = FEEDER_FAULT;
                break;
        }
    }

    /* 释放或切换控制方式取消当前组，未完成的组在反馈状态变化后重新计发，完成状态持续保留 */
    const bool reset_group = mode != FEEDER_CMD_RUN || !cycle_command.single_fire ||
                             loop_mode_changed || fire_mode_changed ||
                             (next_state != feeder->state && !group_complete);
    if (reset_group && internal->single_state != FEEDER_SINGLE_WAIT_CENTER)
    {
        feeder_steps_remaining = 0U;
        internal->single_state = FEEDER_SINGLE_WAIT_CENTER;
        internal->step_elapsed_s = 0.0f;
        internal->settled_s = 0.0f;
    }

    /* 状态、配置或环路结构变化时重置控制历史，回中只撤驱而不继续计算位置保持 */
    const bool restart = next_state != feeder->state || config_changed || loop_mode_changed || fire_mode_changed;
    if (restart)
    {
        PID_Reset(&internal->position_pid);
        PID_Reset(&internal->speed_pid);
        feeder->target_speed_rpm = 0.0f;
        if (next_state == FEEDER_RUNNING)
        {
            /* 只为参与控制的速度环预置微分，关闭速度环时不更新其滤波与计算历史 */
            if (loop_mode == FEEDER_LOOP_SPEED || loop_mode == FEEDER_LOOP_CASCADE)
            {
                LowPassFilter2p_Init(&internal->speed_pid.dfilter, internal->frequency_hz,
                    internal->cycle_config.speed_pid.d_cutoff_freq);
                internal->speed_pid.last.k_fb = LowPassFilter2p_Reset(&internal->speed_pid.dfilter,
                    internal->cycle_config.speed_pid.k * feeder->measured_speed_rpm);
            }
            /* 单角度环与串级均维护逐齿目标，恢复或切换控制结构时捕获当前位置并清理当前进给阶段 */
            if (position_active)
            {
                LowPassFilter2p_Init(&internal->position_pid.dfilter, internal->frequency_hz,
                    internal->cycle_config.position_pid.d_cutoff_freq);
                /* 首次使能或反馈恢复时捕获当前位置，避免切换模式时追赶旧目标 */
                if (loop_mode_changed || feeder->state != FEEDER_RUNNING || fire_mode_changed)
                {
                    feeder->target_angle_deg = feeder->measured_angle_deg;
                }
            }
        }
    }
    feeder->state = next_state;
    feeder_active_fire_mode = fire_mode;
    feeder_active_loop_mode = loop_mode;
    if (feeder->state != FEEDER_RUNNING)
    {
        return;
    }

    /* 连发只走速度环：命令倍率直接映射输出轴转速，角度外环完全不参与 */
    if (loop_mode == FEEDER_LOOP_SPEED)
    {
        const Feeder_Config_t *config = &internal->cycle_config;
        const float previous_target_rpm = feeder->target_speed_rpm;
        feeder->target_speed_rpm = config->direction * feed_ratio * config->max_speed_rpm;
        const float feedforward_a = (feeder->target_speed_rpm - previous_target_rpm) *
            config->speed_feedforward_gain_a_per_rpm;
        const float current_a = PID_Calc(&internal->speed_pid, feeder->target_speed_rpm,
            feeder->measured_speed_rpm, 0.0f, period_s) + feedforward_a;
        /* 前馈乘法和叠加在控制计算出口检查一次，异常只改变状态，由输出闸门统一撤驱 */
        if (!isfinite(current_a))
        {
            feeder->state = FEEDER_FAULT;
            return;
        }
        feeder->output_current_a = AbsClip(current_a, config->max_current_a);
        return;
    }

    /* 本发完成前保持固定目标，组内逐齿推进，整组完成由运行状态切换和唯一输出闸门撤驱 */
    Feeder_UpdateSingle(feeder, internal, &cycle_command, period_s);
    feeder_angle_target_deg = feeder->target_angle_deg;
    if (feeder->state != FEEDER_RUNNING)
    {
        return;
    }

    /* 单发目标由模块持有，外环和后续计算只读取本周期已校验的配置快照 */
    const Feeder_Config_t *config = &internal->cycle_config;
    const float target_angle_rad = Feeder_WrapAngle(feeder->target_angle_deg * M_DEG2RAD_MULT);

    /* 角度外环用实测角速度提供微分，避免角度在 0 与 2π 之间环绕时产生虚假的微分脉冲 */
    const float measured_speed_rad_s = feeder->measured_speed_rpm * (FEEDER_TWO_PI / 60.0f);
    const float derivative = config->position_pid.k * measured_speed_rad_s;
    /* 目标推进与微分换算属于本层计算链，在进入外环前统一拒绝溢出 */
    if (!isfinite(target_angle_rad) || !isfinite(derivative))
    {
        feeder->state = FEEDER_FAULT;
        return;
    }

    /* 两种角度路径共用环绕误差计算，输出单位与本周期有效控制结构一致 */
    const float position_output = PID_Calc(&internal->position_pid, target_angle_rad,
        internal->feedback_angle_rad, derivative, period_s);
    switch (loop_mode)
    {
    case FEEDER_LOOP_ANGLE:
        /* 角度环的持久参数已在入口限制为电流量程，不再经过速度环或速度差分前馈 */
        feeder->target_speed_rpm = 0.0f;
        feeder->output_current_a = position_output;
        break;

    case FEEDER_LOOP_CASCADE:
    {
        /* 串级按原单位将角度环输出换算为 rpm，再计算速度内环与差分前馈 */
        const float previous_target_rpm = feeder->target_speed_rpm;
        feeder->target_speed_rpm = AbsClip(position_output * (60.0f / FEEDER_TWO_PI), config->max_speed_rpm);
        const float feedforward_a = (feeder->target_speed_rpm - previous_target_rpm) *
            config->speed_feedforward_gain_a_per_rpm;
        const float current_a = PID_Calc(&internal->speed_pid, feeder->target_speed_rpm,
            feeder->measured_speed_rpm, 0.0f, period_s) + feedforward_a;

        /* 前馈叠加属于新的计算出口，异常只改变状态，由输出闸门统一撤驱 */
        if (!isfinite(current_a))
        {
            feeder->state = FEEDER_FAULT;
        }
        feeder->output_current_a = AbsClip(current_a, config->max_current_a);
        break;
    }

    default:
        feeder->state = FEEDER_FAULT;
        break;
    }
}

/**
 * @brief 以运行状态作为唯一输出闸门
 *
 * @param feeder 拨弹实例
 * @return float 运行时的转子侧电流，其他状态为 0 A
 */
float Feeder_DumpOutput(const Feeder_t *feeder)
{
    return feeder->state == FEEDER_RUNNING ? feeder->output_current_a : 0.0f;
}
