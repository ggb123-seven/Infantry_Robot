#include "module/feeder.h"

#include <math.h>
#include <string.h>

/*
 * 参数来自用户确认的参考工程 shoot.c 连续模式与默认步兵配置
 * 每秒 20 齿、一圈 10 齿，平均输出轴 120 rpm，M2006 转子约 4320 rpm
 * 正面顺时针采用正常拨弹的负角度方向，实物安装方向仍须上板核对
 * 位置 PID 沿用 k=12、p=1、i=0、d=0.0450000018，归一化输出换算为 M2006 的 ±10 A
 */
#define FEEDER_TWO_PI (6.28318530717958647692f)
#define FEEDER_CURRENT_RANGE_A (10.0f)
#define FEEDER_MAX_PERIOD_S (0.1f)
// 500 Hz 通信允许最多五个周期未取得新帧，超过 10 ms 撤驱且恢复时重捕获位置
#define FEEDER_FEEDBACK_TIMEOUT_US (10000U)

static const Feeder_Config_t feeder_default_config =
{
    .tooth_count = 10.0f,
    .feed_hz = 20.0f,
    .direction = -1.0f,
    .position_pid =
    {
        .k = 12.0f,
        .p = 1.0f,
        .i = 0.0f,
        .d = 0.0450000018f,
        .i_limit = 1.0f,
        .out_limit = 1.0f,
        .d_cutoff_freq = -1.0f,
        .range = FEEDER_TWO_PI,
    },
};

/**
 * @brief 在初始化和控制入口集中校验配置快照
 *
 * @param[in] config 待校验快照，调用方保证非空
 * @param[in] frequency_hz 已确认的控制采样频率，单位 Hz
 * @return 齿距、调度和控制器量程均合法时返回 true
 */
static bool Feeder_ConfigValid(const Feeder_Config_t *config, float frequency_hz)
{
    const KPID_Params_t *pid = &config->position_pid;
    return isfinite(config->tooth_count) && config->tooth_count >= 2.0f &&
           isfinite(config->feed_hz) && config->feed_hz > 0.0f && config->feed_hz <= frequency_hz &&
           (config->direction == -1.0f || config->direction == 1.0f) &&
           isfinite(pid->k) && pid->k > 0.0f && isfinite(pid->p) && pid->p >= 0.0f &&
           isfinite(pid->i) && pid->i >= 0.0f && isfinite(pid->d) && pid->d >= 0.0f &&
           isfinite(pid->i_limit) && pid->i_limit >= 0.0f && pid->i_limit <= 1.0f &&
           isfinite(pid->out_limit) && pid->out_limit > 0.000001f && pid->out_limit <= 1.0f &&
           isfinite(pid->d_cutoff_freq) && pid->d_cutoff_freq <= 0.0f && pid->range == FEEDER_TWO_PI;
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
    // 初始化从安全状态开始，配置副本保留给运行期调参
    *feeder = (Feeder_t)
    {
        .config = feeder_default_config,
        .cycle_config = feeder_default_config,
        .frequency_hz = frequency_hz,
        .state = FEEDER_FAULT,
    };
    feeder->config_valid = isfinite(frequency_hz) && frequency_hz > 0.0f &&
                           Feeder_ConfigValid(&feeder->cycle_config, frequency_hz);
    if (!feeder->config_valid)
    {
        return -1;
    }

    // 用实测速度提供微分，避免位置在 0 与 2π 之间环绕时产生虚假的微分脉冲
    feeder->initialized = PID_Init(&feeder->position_pid, KPID_MODE_SET_D, frequency_hz,
                                   &feeder->cycle_config.position_pid) == 0;
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
    // 新帧只在输入边界检查一次，无新帧时保留缓存和逐齿进给相位
    if (received)
    {
        // 角度和速度由驱动内整数协议换算，直接信任已完成的解码，仅检查角度连续性标志
        feeder->feedback_valid = feedback->angle_valid;
        if (feeder->feedback_valid)
        {
            feeder->feedback = *feedback;
        }
    }

    // 驱动保存微秒时间低 32 位，无符号差值允许正常回绕并拒绝启动时的未来帧
    const uint32_t age_us = (uint32_t)now_us - feeder->feedback.last_update_time;
    feeder->feedback_valid = feeder->feedback_valid && age_us <= FEEDER_FEEDBACK_TIMEOUT_US && now_us >= age_us;
}

/**
 * @brief 集中处理配置、遥控时效、逐齿目标与状态转换
 *
 * @param[in,out] feeder 拨弹实例，调用方保证非空
 * @param[in] remote 遥控一致快照，调用方保证非空
 * @param[in] remote_valid 最近邮箱读取和缓存有效性
 * @param[in] now_us 读取遥控后的完整单调时间，单位微秒
 * @param[in] period_s 实测控制周期，单位秒
 * @return 无返回值
 */
void Feeder_Control(Feeder_t *feeder, const DR16_State_t *remote, bool remote_valid, uint64_t now_us, float period_s)
{
    // 每周期只读取一次可调副本，之后 PID、目标生成和输出只使用持久快照
    const Feeder_Config_t previous_config = feeder->cycle_config;
    feeder->cycle_config = *(const volatile Feeder_Config_t *)&feeder->config;
    feeder->config_valid = Feeder_ConfigValid(&feeder->cycle_config, feeder->frequency_hz);
    const bool config_changed = memcmp(&previous_config, &feeder->cycle_config, sizeof(previous_config)) != 0;
    const bool period_valid = isfinite(period_s) && period_s > 0.0f && period_s <= FEEDER_MAX_PERIOD_S;
    const bool remote_online = remote_valid && remote->header.online && now_us >= remote->header.last_online_time &&
        now_us - remote->header.last_online_time <= DR16_RECEIVER_OFFLINE_TIMEOUT_US;
    const bool requested = remote_online && remote->data.sw_r == DR16_SWITCH_DOWN;
    Feeder_State_t next_state;

    // 停机和通信恢复都重新捕获实际位置，禁止追赶失联期间累积的旧目标
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
                next_state = !requested ? FEEDER_RELAX :
                             feeder->feedback_valid ? FEEDER_RUNNING : FEEDER_OFFLINE;
                break;
            case FEEDER_FAULT:
                // 故障只能由在线遥控的停机档解除，断链或反馈恢复不会自动绕过故障
                next_state = remote_online && (remote->data.sw_r == DR16_SWITCH_UP ||
                             remote->data.sw_r == DR16_SWITCH_MIDDLE) ? FEEDER_RELAX : FEEDER_FAULT;
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
        feeder->phase_s = 0.0f;
        if (next_state == FEEDER_RUNNING)
        {
            feeder->target_angle_rad = feeder->feedback.rotor_single_angle;
            feeder->phase_s = 1.0 / (double)feeder->cycle_config.feed_hz;
        }
    }
    feeder->state = next_state;
    if (feeder->state != FEEDER_RUNNING)
    {
        return;
    }

    // 参考工程每 50 ms 前进一齿，按实际周期累计时间并保留不足一齿的余量
    const Feeder_Config_t *config = &feeder->cycle_config;
    const double interval_s = 1.0 / (double)config->feed_hz;
    // 首齿立即执行，后续间隔从本周期起算，避免把启动前的周期时间计入下一齿
    if (!restart)
    {
        // 实测周期来自整数微秒，恢复其分辨率并以双精度累积，避免长时间运行后整齿边界漂移
        feeder->phase_s += (double)roundf(period_s * 1000000.0f) * 0.000001;
    }
    const double teeth = floor((feeder->phase_s + 0.000000001) / interval_s);
    feeder->phase_s -= teeth * interval_s;
    feeder->target_angle_rad = Feeder_WrapAngle(feeder->target_angle_rad +
        config->direction * (float)teeth * FEEDER_TWO_PI / config->tooth_count);
    const float speed_rad_s = feeder->feedback.rotor_speed * (FEEDER_TWO_PI / 60.0f);
    const float derivative = config->position_pid.k * speed_rad_s;

    // 目标与微分换算属于本层计算链，在进入 PID 前统一拒绝溢出
    if (!isfinite(feeder->target_angle_rad) || !isfinite(derivative))
    {
        feeder->state = FEEDER_FAULT;
        return;
    }
    feeder->output_current_a = PID_Calc(&feeder->position_pid, feeder->target_angle_rad,
        feeder->feedback.rotor_single_angle, derivative, period_s) * FEEDER_CURRENT_RANGE_A;
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
