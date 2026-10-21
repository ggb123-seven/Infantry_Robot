#include "module/gimbal.h"

#include <math.h>

#include "component/pid.h"

volatile GimbalYawSpeed_t gimbal_yaw_speed =
{
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .target_rpm = 0.0f,
    .actual_rpm = 0.0f
};

/* 控制器未初始化时禁止输出，待机时清空历史量，运行入口使用当前反馈预置微分 */
typedef enum
{
    GIMBAL_YAW_UNINITIALIZED = 0,
    GIMBAL_YAW_READY,
    GIMBAL_YAW_RUNNING
} GimbalYawState_t;

static GimbalYawState_t yaw_state;
static KPID_t yaw_pid;
static KPID_Params_t yaw_pid_params;

/**
 * @brief 清空控制历史并回到待机状态，供禁用与输入异常路径共用
 *
 * @return 固定返回零电流，单位 A
 */
static float Gimbal_YawSpeedStop(void)
{
    if (yaw_state != GIMBAL_YAW_UNINITIALIZED)
    {
        yaw_state = PID_Reset(&yaw_pid) == 0 ? GIMBAL_YAW_READY : GIMBAL_YAW_UNINITIALIZED;
    }
    return 0.0f;
}

/**
 * @brief 初始化 yaw 速度控制器并清除上次运行历史
 *
 * @param[in] sample_frequency_hz 控制频率，单位 Hz，范围为 10 至 2000
 * @return 成功返回 0，输入非法或控制器初始化失败返回 -1
 */
int8_t Gimbal_Init(float sample_frequency_hz)
{
    // 初始化失败时保持未就绪，禁止沿用之前的控制历史
    yaw_state = GIMBAL_YAW_UNINITIALIZED;
    if (!isfinite(sample_frequency_hz) || sample_frequency_hz < 10.0f || sample_frequency_hz > 2000.0f)
    {
        return -1;
    }

    // 使用并行 PID，微分作用于反馈，当前关闭微分低通以便后续单独调参
    yaw_pid_params.k = 1.0f;
    yaw_pid_params.p = 0.0f;
    yaw_pid_params.i = 0.0f;
    yaw_pid_params.d = 0.0f;
    yaw_pid_params.i_limit = 0.0f;
    yaw_pid_params.out_limit = GIMBAL_YAW_CURRENT_LIMIT_A;
    yaw_pid_params.d_cutoff_freq = 0.0f;
    yaw_pid_params.range = 0.0f;
    if (PID_Init(&yaw_pid, KPID_MODE_CALC_D, sample_frequency_hz, &yaw_pid_params) != 0)
    {
        return -1;
    }

    yaw_state = GIMBAL_YAW_READY;
    return 0;
}

/**
 * @brief 将 yaw 反馈转换为角速度并执行一次速度闭环计算
 *
 * @param[in] feedback_rpm 驱动原始转子速度，单位 rpm，有效范围为有符号 16 位反馈范围
 * @param[in] feedback_age_ms 距最近有效帧的时间，单位 ms，尚无反馈时传入 UINT32_MAX
 * @param[in] enabled 是否允许速度控制，false 时输出零电流
 * @param[in] period_s 控制间隔，单位 s，范围为初始化周期至 0.1 s
 * @return 转子侧目标电流，单位 A，限幅为正负 1 A，异常路径返回 0
 */
float Gimbal_YawSpeedRun(float feedback_rpm, uint32_t feedback_age_ms, bool enabled, float period_s)
{
    // 帧过期或原始速度非法时明确标记反馈无效，避免调试区显示旧速度
    const bool feedback_valid = isfinite(feedback_rpm) && feedback_rpm >= -32768.0f &&
                                feedback_rpm <= 32767.0f &&
                                feedback_age_ms <= GIMBAL_YAW_FEEDBACK_TIMEOUT_MS;
    // 标准反馈单位 rpm，按每分钟 60 秒和每圈 2π 弧度转换为 rad/s
    const float actual_rad_s = feedback_valid ? feedback_rpm * 0.10471975512f : NAN;
    gimbal_yaw_speed.actual_rpm = actual_rad_s;

    // 每周期读取一次调试输入，实际速度始终来自反馈，不能被调试器写值替代
    const float kp = gimbal_yaw_speed.kp;
    const float ki = gimbal_yaw_speed.ki;
    const float kd = gimbal_yaw_speed.kd;
    const float requested_rad_s = gimbal_yaw_speed.target_rpm;
    if (!enabled || !feedback_valid || !isfinite(period_s) || period_s <= 0.0f ||
        period_s > GIMBAL_YAW_MAX_PERIOD_S)
    {
        return Gimbal_YawSpeedStop();
    }
    if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd) || !isfinite(requested_rad_s) ||
        kp < 0.0f || ki < 0.0f || kd < 0.0f || kp > 1000.0f || ki > 1000.0f || kd > 1000.0f)
    {
        return Gimbal_YawSpeedStop();
    }
    if (yaw_state == GIMBAL_YAW_UNINITIALIZED || period_s < yaw_pid.dt_min)
    {
        return Gimbal_YawSpeedStop();
    }

    // 增益改变时清空旧积分与微分历史，避免旧参数留下的积分影响当前输出
    if (kp != yaw_pid_params.p || ki != yaw_pid_params.i || kd != yaw_pid_params.d)
    {
        yaw_pid_params.p = kp;
        yaw_pid_params.i = ki;
        yaw_pid_params.d = kd;
        yaw_pid_params.i_limit = ki > 0.000001f ? GIMBAL_YAW_CURRENT_LIMIT_A / ki : 0.0f;
        Gimbal_YawSpeedStop();
    }

    // 待机进入运行时以当前反馈预置微分，停止和异常恢复均复用这一入口
    switch (yaw_state)
    {
        case GIMBAL_YAW_READY:
            yaw_pid.last.k_fb = actual_rad_s;
            LowPassFilter2p_Reset(&yaw_pid.dfilter, actual_rad_s);
            yaw_state = GIMBAL_YAW_RUNNING;
            break;

        case GIMBAL_YAW_RUNNING:
            break;

        case GIMBAL_YAW_UNINITIALIZED:
        default:
            yaw_state = GIMBAL_YAW_UNINITIALIZED;
            return 0.0f;
    }

    // 保留调试区请求值，计算时单独限幅目标与电流，输出留给通信任务统一发送
    const float target_rad_s = fmaxf(-GIMBAL_YAW_SPEED_LIMIT_RAD_S,
                                    fminf(requested_rad_s, GIMBAL_YAW_SPEED_LIMIT_RAD_S));
    const float current_a = PID_Calc(&yaw_pid, target_rad_s, actual_rad_s, 0.0f, period_s);
    if (!isfinite(current_a))
    {
        return Gimbal_YawSpeedStop();
    }
    return fmaxf(-GIMBAL_YAW_CURRENT_LIMIT_A, fminf(current_a, GIMBAL_YAW_CURRENT_LIMIT_A));
}

/**
 * @brief 使用完整时间戳校验反馈，防止计数回绕和毫秒截断重新放行过期反馈
 *
 * @param[in] feedback_rpm 原始转子转速，单位 rpm，无反馈时传入非数值
 * @param[in] feedback_time_us 最近反馈时刻，单位微秒，UINT64_MAX 表示无反馈
 * @param[in] now_us 当前单调时间，单位微秒
 * @param[in] enabled 是否允许控制，false 时返回零并清空控制历史
 * @param[in] period_s 控制间隔，单位秒，范围为初始化周期至 0.1 秒
 * @return 目标电流，单位 A，反馈无效或时间异常时返回零
 */
float Gimbal_YawSpeedRunAt(float feedback_rpm, uint64_t feedback_time_us, uint64_t now_us,
                         bool enabled, float period_s)
{
    // 先按微秒判断边界，再转换接口单位，避免超时不足一毫秒时被截断为合法值
    uint32_t feedback_age_ms = UINT32_MAX;
    if (feedback_time_us != UINT64_MAX && now_us >= feedback_time_us &&
        now_us - feedback_time_us <= (uint64_t)GIMBAL_YAW_FEEDBACK_TIMEOUT_MS * 1000U)
    {
        feedback_age_ms = (uint32_t)((now_us - feedback_time_us) / 1000U);
    }
    return Gimbal_YawSpeedRun(feedback_rpm, feedback_age_ms, enabled, period_s);
}
