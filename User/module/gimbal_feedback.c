#include "module/gimbal_private.h"

#include <math.h>
#include <stddef.h>

#include "component/user_math.h"

/*
 * 反馈换算与校验边界，沿用拆分前参数：
 * - rpm 转 rad/s 使用每分钟 60 秒、每圈 2π 弧度；完整一圈为 360 度
 * - 电机原始转速按有符号 16 位协议范围校验，单位 rpm
 * - 惯性角速度上限 40 rad/s，覆盖当前量程余量；俯仰距欧拉奇异点保留 1 度
 * - 相邻姿态跳变量在物理角速度上界外额外允许 1 度解算余量
 */
#define GIMBAL_RPM_TO_RAD_S (0.10471975512f)
#define GIMBAL_RAW_SPEED_MIN_RPM (-32768.0f)
#define GIMBAL_RAW_SPEED_MAX_RPM (32767.0f)
#define GIMBAL_IMU_MAX_RAD_S (40.0f)
#define GIMBAL_IMU_MAX_PITCH_DEG (89.0f)
#define GIMBAL_IMU_JUMP_MARGIN_DEG (1.0f)

static void Gimbal_RejectAxisFeedback(Gimbal_Axis_t *axis, Gimbal_AxisState_t next_state);
static void Gimbal_InvalidateImu(Gimbal_t *gimbal);
static void Gimbal_UpdateImu(Gimbal_t *gimbal, const Gimbal_ImuFeedback_t *imu, uint64_t now_us);
static void Gimbal_UpdateAxisFeedback(Gimbal_Axis_t *axis, const Gimbal_AxisConfig_t *config,
                                      const MOTOR_RM_Feedback_t *motor, uint64_t now_us, uint64_t timeout_us,
                                      bool signed_angle);

void Gimbal_UpdateFeedback(Gimbal_t *gimbal, const MOTOR_RM_Feedback_t *yaw_motor,
                           const MOTOR_RM_Feedback_t *pitch_motor, const Gimbal_ImuFeedback_t *imu, uint64_t now_us)
{
    if (gimbal == NULL)
    {
        return;
    }

    // 超时配置非法时无法判断时效，两轴反馈全部视为失联
    const float timeout_s = gimbal->config.feedback_timeout_s;
    const bool timeout_valid = isfinite(timeout_s) && timeout_s > 0.0f && timeout_s <= 1.0f;
    const uint64_t timeout_us = timeout_valid ? (uint64_t)(timeout_s * 1.0e6f) : 0U;
    Gimbal_UpdateAxisFeedback(&gimbal->yaw, &gimbal->config.yaw, yaw_motor, now_us, timeout_us, false);
    Gimbal_UpdateAxisFeedback(&gimbal->pitch, &gimbal->config.pitch, pitch_motor, now_us, timeout_us, true);

    // 惯性快照独立校验时效和连续性，控制量只由该快照更新
    Gimbal_UpdateImu(gimbal, imu, now_us);

    // 两侧反馈更新后估计底座大地系角速度，底座水平时与关节共轴，任一侧为非数值时结果同为非数值
    gimbal->yaw.base_speed_rad_s = gimbal->yaw.speed_rad_s - gimbal->yaw.motor_speed_rad_s;
    gimbal->pitch.base_speed_rad_s = gimbal->pitch.speed_rad_s - gimbal->pitch.motor_speed_rad_s;
}

void Gimbal_ClearFeedback(Gimbal_Axis_t *axis)
{
    axis->encoder_deg = NAN;
    axis->mechanical_angle_deg = NAN;
    axis->motor_speed_rad_s = NAN;
    axis->base_speed_rad_s = NAN;
    axis->feedback_time_us = UINT64_MAX;
    AngleUnwrap_Reset(&axis->unwrap);
}

/**
 * @brief 拒绝机械反馈并集中处理失联等待与不可自动恢复的反馈故障
 *
 * @param[in,out] axis 单轴运行数据
 * @param[in] next_state 超时传 OFFLINE，其他反馈异常传 FAULT
 * @return 无返回值
 */
static void Gimbal_RejectAxisFeedback(Gimbal_Axis_t *axis, Gimbal_AxisState_t next_state)
{
    // 反馈事件只改变已运行轴的保护状态，待机继续等待，已有故障不得降级为失联
    switch (axis->state)
    {
        case GIMBAL_AXIS_RUNNING:
        case GIMBAL_AXIS_OFFLINE:
            Gimbal_StopAxis(axis, next_state);
            break;
        case GIMBAL_AXIS_RELAX:
        case GIMBAL_AXIS_FAULT:
            break;
        default:
            Gimbal_StopAxis(axis, GIMBAL_AXIS_FAULT);
            break;
    }
    Gimbal_ClearFeedback(axis);
}

/**
 * @brief 撤销惯性控制量并根据当前阶段进入等待或故障
 *
 * @param[in,out] gimbal 云台实例
 * @return 无返回值
 */
static void Gimbal_InvalidateImu(Gimbal_t *gimbal)
{
    gimbal->imu.valid = false;
    gimbal->yaw.angle_deg = gimbal->pitch.angle_deg = NAN;
    gimbal->yaw.speed_rad_s = gimbal->pitch.speed_rad_s = NAN;

    // 启动期间允许继续等待，运行后的共用反馈异常必须锁存至人工待机
    switch (gimbal->imu_state)
    {
        case GIMBAL_IMU_WAITING:
            gimbal->imu_state = GIMBAL_IMU_WAITING;
            break;

        case GIMBAL_IMU_READY:
        case GIMBAL_IMU_FAULT:
        default:
            gimbal->imu_state = GIMBAL_IMU_FAULT;
            break;
    }
}

/**
 * @brief 校验同帧惯性反馈并推进有效就绪与故障状态机
 *
 * @param[in,out] gimbal 云台实例
 * @param[in] imu 新快照，无新消息时为 NULL
 * @param[in] now_us 当前单调时刻，单位微秒
 * @return 无返回值
 */
static void Gimbal_UpdateImu(Gimbal_t *gimbal, const Gimbal_ImuFeedback_t *imu, uint64_t now_us)
{
    const float timeout_s = gimbal->config.imu_timeout_s;
    if (!isfinite(timeout_s) || timeout_s <= 0.0f || timeout_s > 0.1f)
    {
        Gimbal_InvalidateImu(gimbal);
        return;
    }
    const uint64_t timeout_us = (uint64_t)(timeout_s * 1.0e6f);
    const Gimbal_ImuFeedback_t *frame = imu != NULL ? imu : &gimbal->imu;
    bool valid = frame->valid && frame->sample_time_us <= now_us &&
                 now_us - frame->sample_time_us <= timeout_us;

    // 所有轴共同校验，任何非有限数据都不能进入任一轴的闭环
    for (uint32_t i = 0U; i < 3U; ++i)
    {
        valid = valid && isfinite(frame->angle_deg[i]) && isfinite(frame->gyro_rad_s[i]) &&
                fabsf(frame->gyro_rad_s[i]) <= GIMBAL_IMU_MAX_RAD_S;
    }
    valid = valid && fabsf(frame->angle_deg[0]) <= 180.0f &&
            fabsf(frame->angle_deg[1]) < GIMBAL_IMU_MAX_PITCH_DEG;

    // 新帧必须严格前进，重复帧、倒退及断流后突然恢复均不得掩盖坐标断开
    if (imu != NULL && gimbal->imu.valid)
    {
        const uint64_t previous_us = gimbal->imu.sample_time_us;
        const bool continuous = frame->sample_time_us > previous_us &&
                                frame->sample_time_us - previous_us <= timeout_us;
        valid = valid && continuous;
        const float dt_s = continuous ? (float)(frame->sample_time_us - previous_us) * 1.0e-6f : 0.0f;
        for (uint32_t i = 0U; i < 3U; ++i)
        {
            float delta_deg = frame->angle_deg[i] - gimbal->imu.angle_deg[i];
            if (i == 0U)
            {
                delta_deg = AngleUnwrap_Wrap(delta_deg + 180.0f, GIMBAL_FULL_TURN_DEG) - 180.0f;
            }
            const float max_delta_deg = GIMBAL_IMU_MAX_RAD_S * M_RAD2DEG_MULT * dt_s + GIMBAL_IMU_JUMP_MARGIN_DEG;
            valid = valid && fabsf(delta_deg) <= max_delta_deg;
        }
    }
    if (!valid)
    {
        Gimbal_InvalidateImu(gimbal);
        return;
    }

    // 有效新快照直接就绪，运动不阻止使能，故障状态仍须先由命令解除
    if (imu != NULL)
    {
        gimbal->imu = *imu;
        switch (gimbal->imu_state)
        {
            case GIMBAL_IMU_WAITING:
                gimbal->imu_state = GIMBAL_IMU_READY;
                break;

            case GIMBAL_IMU_READY:
            case GIMBAL_IMU_FAULT:
                break;

            default:
                Gimbal_InvalidateImu(gimbal);
                break;
        }
    }

    // 当前 INS 已完成安装修正，角度直接取 Pitch 与累计 Yaw，不叠加电机方向
    const bool ready = gimbal->imu_state == GIMBAL_IMU_READY;
    gimbal->yaw.angle_deg = ready ? gimbal->imu.angle_deg[2] : NAN;
    gimbal->pitch.angle_deg = ready ? gimbal->imu.angle_deg[1] : NAN;

    // 机体系角速度按 ZYX 欧拉运动学换算为大地系欧拉角速度，有效性校验已保证 |Pitch| < 89°
    const float roll_rad = gimbal->imu.angle_deg[0] * M_DEG2RAD_MULT;
    const float sin_roll = sinf(roll_rad);
    const float cos_roll = cosf(roll_rad);
    const float gyro_y = gimbal->imu.gyro_rad_s[1];
    const float gyro_z = gimbal->imu.gyro_rad_s[2];
    const float yaw_rate = (gyro_y * sin_roll + gyro_z * cos_roll) / cosf(gimbal->imu.angle_deg[1] * M_DEG2RAD_MULT);
    gimbal->yaw.speed_rad_s = ready ? yaw_rate : NAN;
    gimbal->pitch.speed_rad_s = ready ? gyro_y * cos_roll - gyro_z * sin_roll : NAN;
}

/**
 * @brief 校验单轴新反馈帧并换算编码器角度、转速和相对角度
 *
 * 超时进入单轴失联等待，非法帧、时间倒退或展开歧义锁存故障，惯性量独立处理
 *
 * @param[in,out] axis 单轴运行数据
 * @param[in] config 单轴配置
 * @param[in] motor 新反馈帧，本周期没有新帧时为 NULL
 * @param[in] now_us 当前单调时刻，单位微秒
 * @param[in] timeout_us 反馈最大间隔，单位微秒，为 0 时视为配置故障
 * @param[in] signed_angle 为 true 时使用零点两侧的有符号单圈角，否则展开连续角
 * @return 无返回值
 */
static void Gimbal_UpdateAxisFeedback(Gimbal_Axis_t *axis, const Gimbal_AxisConfig_t *config,
                                      const MOTOR_RM_Feedback_t *motor, uint64_t now_us, uint64_t timeout_us,
                                      bool signed_angle)
{
    if (timeout_us == 0U)
    {
        Gimbal_RejectAxisFeedback(axis, GIMBAL_AXIS_FAULT);
        return;
    }

    // 没有新帧时只检查旧帧时效，超时后清除反馈
    if (motor == NULL)
    {
        if (axis->feedback_time_us == UINT64_MAX || now_us < axis->feedback_time_us ||
            now_us - axis->feedback_time_us > timeout_us)
        {
            Gimbal_RejectAxisFeedback(axis, now_us < axis->feedback_time_us &&
                axis->feedback_time_us != UINT64_MAX ? GIMBAL_AXIS_FAULT : GIMBAL_AXIS_OFFLINE);
        }
        return;
    }

    // 驱动只保存时间戳低 32 位，按与当前时刻的差值恢复完整微秒时刻
    const uint32_t age_us = (uint32_t)now_us - motor->last_update_time;
    const uint64_t frame_time_us = now_us >= age_us ? now_us - age_us : UINT64_MAX;
    const float encoder_rad = motor->rotor_single_angle;
    const float speed_rpm = motor->rotor_speed;
    const bool frame_valid = motor->angle_valid && frame_time_us != UINT64_MAX && age_us <= timeout_us &&
                             isfinite(encoder_rad) && encoder_rad >= 0.0f && encoder_rad < M_2PI &&
                             isfinite(speed_rpm) && speed_rpm >= GIMBAL_RAW_SPEED_MIN_RPM &&
                             speed_rpm <= GIMBAL_RAW_SPEED_MAX_RPM;
    // 两种角度都拒绝旧帧和超时间隔，pitch 不依赖连续角展开器也保留时序校验
    const bool continuous = axis->feedback_time_us != UINT64_MAX && frame_time_us > axis->feedback_time_us &&
                            frame_time_us - axis->feedback_time_us <= timeout_us;
    const bool has_reference = signed_angle ? isfinite(axis->mechanical_angle_deg) : axis->unwrap.has_reference;
    if (!frame_valid || (has_reference && frame_time_us <= axis->feedback_time_us))
    {
        Gimbal_RejectAxisFeedback(axis, GIMBAL_AXIS_FAULT);
        return;
    }
    if (has_reference && !continuous)
    {
        Gimbal_RejectAxisFeedback(axis, GIMBAL_AXIS_OFFLINE);
        return;
    }

    // 反装轴先翻转角度和转速，使模块内统一以角度增大方向为正
    const float raw_encoder_deg = AngleUnwrap_Wrap(encoder_rad * M_RAD2DEG_MULT, GIMBAL_FULL_TURN_DEG);
    const float encoder_deg = config->reversed ?
        AngleUnwrap_Wrap(GIMBAL_FULL_TURN_DEG - raw_encoder_deg, GIMBAL_FULL_TURN_DEG) : raw_encoder_deg;
    const float speed_rad_s = speed_rpm * GIMBAL_RPM_TO_RAD_S;
    axis->encoder_deg = encoder_deg;
    axis->motor_speed_rad_s = config->reversed ? -speed_rad_s : speed_rad_s;
    axis->feedback_time_us = frame_time_us;

    // pitch 行程围绕水平零点，直接映射到 [-180, 180)，负角上电和失联恢复均不累计圈数
    if (signed_angle)
    {
        const float half_turn_deg = GIMBAL_FULL_TURN_DEG * 0.5f;
        axis->mechanical_angle_deg = AngleUnwrap_Wrap(encoder_deg - axis->applied_zero_encoder_deg + half_turn_deg,
                                                    GIMBAL_FULL_TURN_DEG) - half_turn_deg;
        return;
    }

    // yaw 保留原有连续角展开，跨编码器零点时按相邻帧最短路径累计
    const float relative_deg = AngleUnwrap_Wrap(encoder_deg - axis->applied_zero_encoder_deg, GIMBAL_FULL_TURN_DEG);
    float angle_deg = NAN;
    if (!AngleUnwrap_Update(&axis->unwrap, relative_deg, GIMBAL_FULL_TURN_DEG, &angle_deg))
    {
        Gimbal_RejectAxisFeedback(axis, GIMBAL_AXIS_FAULT);
        return;
    }
    axis->mechanical_angle_deg = angle_deg;
}

void Gimbal_StopAxis(Gimbal_Axis_t *axis, Gimbal_AxisState_t next_state)
{
    PID_Reset(&axis->angle_pid);
    PID_Reset(&axis->speed_pid);
    axis->target_angle_deg = NAN;
    axis->target_speed_rad_s = 0.0f;
    axis->output_current_a = 0.0f;
    axis->state = next_state == GIMBAL_AXIS_RELAX || next_state == GIMBAL_AXIS_OFFLINE ? next_state : GIMBAL_AXIS_FAULT;
}
