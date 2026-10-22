#include "module/gimbal.h"

#include <math.h>

#include "bsp/time.h"
#include "component/pid.h"
#include "device/motor_rm.h"

volatile GimbalYawSpeed_t gimbal_yaw_speed =
{
    .kp = 0.05f,
    .ki = 0.177f,
    .kd = 0.0f,
    .target_rad_s = 0.0f,
    .actual_rad_s = 0.0f
};

volatile GimbalYawAngle_t gimbal_yaw_angle =
{
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .target_rad = 0.0f,
    .actual_rad = 0.0f,
    .enabled = true
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

// 角度外环独立保存状态与历史，由同一个调用任务串行初始化、使能和计算
static GimbalYawState_t yaw_angle_state;
static KPID_t yaw_angle_pid;

// 反馈缓存与邮箱句柄由云台模块独占使用，邮箱对象仍由系统初始化入口统一管理
static MOTOR_RM_Feedback_t yaw_feedback;
static uint64_t yaw_feedback_time_us = UINT64_MAX;
static osMessageQueueId_t yaw_feedback_queue;
static osMessageQueueId_t yaw_current_queue;

// 串级外环使用模块内常驻参数，单圈编码器按 2π 环绕，速度限幅与内环一致
static KPID_Params_t yaw_angle_params =
{
    .k = 1.0f,
    .p = 0.0f,
    .i = 0.0f,
    .d = 0.0f,
    .i_limit = 0.0f,
    .out_limit = GIMBAL_YAW_SPEED_LIMIT_RAD_S,
    .d_cutoff_freq = 0.0f,
    .range = 6.28318530718f
};

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
 * @brief 初始化 yaw 串级控制器并清除两环上次运行历史
 *
 * @param[in] sample_frequency_hz 控制频率，单位 Hz，范围为 10 至 2000
 * @return 成功返回 0，输入非法或控制器初始化失败返回 -1
 */
int8_t Gimbal_Init(float sample_frequency_hz)
{
    // 初始化失败时保持未就绪，禁止沿用之前的控制历史
    yaw_state = GIMBAL_YAW_UNINITIALIZED;
    yaw_angle_state = GIMBAL_YAW_UNINITIALIZED;
    yaw_feedback_time_us = UINT64_MAX;
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

    // 外环参数由模块常驻保存，初始化后由串级入口按全局使能请求进入运行
    if (Gimbal_YawAngleInit(sample_frequency_hz, &yaw_angle_params) != 0)
    {
        return -1;
    }
    yaw_state = GIMBAL_YAW_READY;
    return 0;
}

/**
 * @brief 保存系统创建的邮箱句柄，启动时丢弃模块缓存的旧反馈时效
 *
 * @param[in] feedback_queue 有效的最新编码器反馈邮箱，元素为 MOTOR_RM_Feedback_t
 * @param[in] current_queue 有效的最新电流命令邮箱，元素为 float，单位 A
 * @return 无返回值，不创建或销毁邮箱，仅在启动或停止后绑定
 */
void Gimbal_BindQueues(osMessageQueueId_t feedback_queue, osMessageQueueId_t current_queue)
{
    yaw_feedback_queue = feedback_queue;
    yaw_current_queue = current_queue;
    yaw_feedback_time_us = UINT64_MAX;
}

/**
 * @brief 消费最新反馈并发布串级控制电流，将数据处理与邮箱交互集中在业务模块
 *
 * @param[in] period_s 实际控制间隔，单位秒，由任务调度提供
 * @return 无返回值，发布失败时清空两环历史，CAN 任务按空邮箱路径归零
 */
void Gimbal_Update(float period_s)
{
    // 读取最新帧并恢复完整微秒时刻，空邮箱保留缓存，读取错误使反馈立即失效
    const uint64_t now_us = BSP_TIME_Get();
    const osStatus_t status = osMessageQueueGet(yaw_feedback_queue, &yaw_feedback, NULL, 0U);
    if (status == osOK)
    {
        const uint32_t age_us = (uint32_t)now_us - yaw_feedback.last_update_time;
        yaw_feedback_time_us = now_us >= age_us ? now_us - age_us : UINT64_MAX;
    }
    else if (status != osErrorResource)
    {
        yaw_feedback_time_us = UINT64_MAX;
    }

    // 编码器角度和转速进入串级控制，角度环输出速度目标，速度环输出电流
    const float current_a = Gimbal_YawRunAt(yaw_feedback.rotor_single_angle, yaw_feedback.rotor_speed,
                                           yaw_feedback_time_us, now_us, true, period_s);

    // 电流邮箱只保留最新命令，发布失败时同步停止两环，避免下周期沿用积分
    if (osMessageQueueReset(yaw_current_queue) != osOK ||
        osMessageQueuePut(yaw_current_queue, &current_a, 0U, 0U) != osOK)
    {
        Gimbal_YawRunAt(yaw_feedback.rotor_single_angle, yaw_feedback.rotor_speed,
                       yaw_feedback_time_us, now_us, false, period_s);
    }
}

/**
 * @brief 将 yaw 反馈转换为角速度并执行一次速度闭环计算
 *
 * @param[in] feedback_rpm 驱动原始转子速度，单位 rpm，有效范围为有符号 16 位反馈范围
 * @param[in] feedback_age_ms 距最近有效帧的时间，单位 ms，尚无反馈时传入 UINT32_MAX
 * @param[in] enabled 是否允许速度控制，false 时输出零电流
 * @param[in] period_s 控制间隔，单位 s，范围为初始化周期至 0.1 s
 * @return 转子侧目标电流，单位 A，限幅由 GIMBAL_YAW_CURRENT_LIMIT_A 指定，异常路径返回 0
 */
float Gimbal_YawSpeedRun(float feedback_rpm, uint32_t feedback_age_ms, bool enabled, float period_s)
{
    // 帧过期或原始速度非法时明确标记反馈无效，避免调试区显示旧速度
    const bool feedback_valid = isfinite(feedback_rpm) && feedback_rpm >= -32768.0f &&
                                feedback_rpm <= 32767.0f &&
                                feedback_age_ms <= GIMBAL_YAW_FEEDBACK_TIMEOUT_MS;
    // 标准反馈单位 rpm，按每分钟 60 秒和每圈 2π 弧度转换为 rad/s
    const float actual_rad_s = feedback_valid ? feedback_rpm * 0.10471975512f : NAN;
    gimbal_yaw_speed.actual_rad_s = actual_rad_s;

    // 每周期读取一次调试输入，实际速度始终来自反馈，不能被调试器写值替代
    const float kp = gimbal_yaw_speed.kp;
    const float ki = gimbal_yaw_speed.ki;
    const float kd = gimbal_yaw_speed.kd;
    const float requested_rad_s = gimbal_yaw_speed.target_rad_s;
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

/**
 * @brief 使用编码器角度和转速执行串级控制，角度外环生成速度内环目标
 *
 * @param[in] feedback_rad 编码器单圈角度，单位 rad，范围 [0, 2π)
 * @param[in] feedback_rpm 编码器转子转速，单位 rpm，有效范围为有符号 16 位反馈范围
 * @param[in] feedback_time_us 最近有效反馈的完整微秒时刻，无反馈为 UINT64_MAX
 * @param[in] now_us 当前完整单调时刻，单位微秒
 * @param[in] enabled 本次调用是否允许控制，false 时停止两环并返回零电流
 * @param[in] period_s 实际控制间隔，单位秒，须在初始化周期至 0.1 秒之间
 * @return 速度内环生成的转子侧电流，单位 A，禁用或反馈无效时为零
 */
float Gimbal_YawRunAt(float feedback_rad, float feedback_rpm, uint64_t feedback_time_us,
                     uint64_t now_us, bool enabled, float period_s)
{
    // 沿用速度控制的反馈时效和原始转速范围，阻止外环在同一无效帧上继续积分
    const bool feedback_valid = feedback_time_us != UINT64_MAX && now_us >= feedback_time_us &&
        now_us - feedback_time_us <= (uint64_t)GIMBAL_YAW_FEEDBACK_TIMEOUT_MS * 1000U &&
        isfinite(feedback_rad) && isfinite(feedback_rpm) && feedback_rpm >= -32768.0f && feedback_rpm <= 32767.0f;
    gimbal_yaw_angle.actual_rad = feedback_valid ? feedback_rad : NAN;
    const bool running = enabled && gimbal_yaw_angle.enabled && feedback_valid && isfinite(period_s) &&
        period_s >= yaw_angle_pid.dt_min && period_s <= GIMBAL_YAW_MAX_PERIOD_S;
    Gimbal_YawAngleSetEnabled(running);

    // 每周期采纳全局调参，增益改变时清空旧外环历史，避免沿用旧积分
    float target_rad_s = 0.0f;
    if (running)
    {
        const float kp = gimbal_yaw_angle.kp;
        const float ki = gimbal_yaw_angle.ki;
        const float kd = gimbal_yaw_angle.kd;
        if (kp != yaw_angle_params.p || ki != yaw_angle_params.i || kd != yaw_angle_params.d)
        {
            yaw_angle_params.p = kp;
            yaw_angle_params.i = ki;
            yaw_angle_params.d = kd;
            yaw_angle_params.i_limit = ki > 0.000001f ? GIMBAL_YAW_SPEED_LIMIT_RAD_S / ki : 0.0f;
            PID_Reset(&yaw_angle_pid);
        }

        // rpm 按每圈 2π 弧度和每分钟 60 秒转换为外环反馈微分，输出单位为 rad/s
        const float feedback_rad_s = feedback_rpm * 0.10471975512f;
        target_rad_s = Gimbal_YawAngleRun(gimbal_yaw_angle.target_rad, feedback_rad, feedback_rad_s, period_s);
    }

    // 外环速度目标直接交给内环，禁用时同步归零目标并清空速度控制历史
    gimbal_yaw_speed.target_rad_s = target_rad_s;
    return Gimbal_YawSpeedRunAt(feedback_rpm, feedback_time_us, now_us, running, period_s);
}

/**
 * @brief 初始化独立角度外环，使用外部角速度作为反馈微分，成功后保持禁用
 *
 * @param[in] sample_frequency_hz 控制频率，单位 Hz，须为有限正数
 * @param[in] params 持续有效的 PID 参数对象，各字段单位和约束见头文件集中说明
 * @return 成功返回 0，底层 PID 初始化失败返回 -1
 */
int8_t Gimbal_YawAngleInit(float sample_frequency_hz, const KPID_Params_t *params)
{
    // 单独初始化外环，不改变已有速度控制器或任务调度
    yaw_angle_state = GIMBAL_YAW_UNINITIALIZED;
    if (PID_Init(&yaw_angle_pid, KPID_MODE_SET_D, sample_frequency_hz, params) != 0)
    {
        return -1;
    }

    yaw_angle_state = GIMBAL_YAW_READY;
    return 0;
}

/**
 * @brief 切换角度外环使能状态，只在状态变化时清空积分与微分历史
 *
 * @param[in] enabled true 为运行，false 为待机，未初始化时不生效
 * @return 无返回值
 */
void Gimbal_YawAngleSetEnabled(bool enabled)
{
    // 使能入口集中管理外环状态，重复设置相同状态时保留运行历史
    switch (yaw_angle_state)
    {
        case GIMBAL_YAW_READY:
        case GIMBAL_YAW_RUNNING:
        {
            const GimbalYawState_t next_state = enabled ? GIMBAL_YAW_RUNNING : GIMBAL_YAW_READY;
            if (next_state != yaw_angle_state)
            {
                PID_Reset(&yaw_angle_pid);
                yaw_angle_state = next_state;
            }
            break;
        }

        case GIMBAL_YAW_UNINITIALIZED:
        default:
            yaw_angle_state = GIMBAL_YAW_UNINITIALIZED;
            break;
    }
}

/**
 * @brief 执行一次角度外环计算，返回速度目标供调用方选择何时接入
 *
 * @param[in] target_rad 目标角度，单位 rad，须与反馈采用相同坐标约定
 * @param[in] feedback_rad 实际角度，单位 rad，环绕范围由初始化参数指定
 * @param[in] feedback_rad_s 实际角速度，单位 rad/s，与角度反馈正方向一致
 * @param[in] period_s 实际控制间隔，单位 s，须为有限正数且不小于初始化周期
 * @return 速度目标，单位 rad/s，未初始化或禁用时为零
 */
float Gimbal_YawAngleRun(float target_rad, float feedback_rad, float feedback_rad_s, float period_s)
{
    // 仅运行状态计算角度到速度的外环输出，反馈有效性由调用方通过使能入口控制
    switch (yaw_angle_state)
    {
        case GIMBAL_YAW_RUNNING:
            return PID_Calc(&yaw_angle_pid, target_rad, feedback_rad, feedback_rad_s, period_s);

        case GIMBAL_YAW_READY:
        case GIMBAL_YAW_UNINITIALIZED:
        default:
            return 0.0f;
    }
}
