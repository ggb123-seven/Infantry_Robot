#include "module/cmd.h"

#include <math.h>
#include <stddef.h>

#include "component/user_math.h"

/*
 * 遥控映射参数，沿用现有底盘与云台参数：
 * - 满量程 660 计数，中心死区 5%，帧龄上限使用 DR16 的 100 毫秒协议约定
 * - 底盘轮速尺度 100 rpm，左拨杆下位叠加 +0.30 的归一化自旋
 * - 云台两轴方向取 -1，满杆角速度 6 rad/s，加减速上限 60000 度每平方秒
 * - 平滑时间常数 4 毫秒，误差在 0.1 度每秒内且满足变化上限时直接收敛
 * - 周期合法范围 (0, 0.1] 秒，度转弧度系数保留原任务精度
 */
#define CMD_CHANNEL_LIMIT (660.0f)
#define CMD_DEADZONE (0.05f)
#define CMD_CHASSIS_SCALE_RPM (100.0f)
#define CMD_CHASSIS_SPIN_WZ (0.30f)
#define CMD_GIMBAL_DIRECTION (-1.0f)
#define CMD_GIMBAL_RATE_RAD_S (6.0f)
#define CMD_GIMBAL_ACCEL_DEG_S2 (60000.0f)
#define CMD_GIMBAL_DECEL_DEG_S2 (60000.0f)
#define CMD_GIMBAL_SMOOTH_TIME_S (0.004f)
#define CMD_GIMBAL_SETTLE_DEG_S (0.1f)
#define CMD_GIMBAL_MAX_PERIOD_S (0.1f)
#define CMD_DEG_TO_RAD (0.0174532925199f)

/**
 * @brief 将原始通道计数限幅归一化并去除中心死区
 *
 * @param[in] channel 去中心后的原始计数
 * @param[in] channel_limit 满量程计数，须为有限正值
 * @param[in] deadzone 中心死区比例，须为有限值且位于 [0, 1)
 * @return 死区内为零，死区外线性映射到 [-1, 1]
 */
static float Cmd_NormalizeChannel(int16_t channel, float channel_limit, float deadzone)
{
    // 先限制满量程，再将死区外的有效行程重新映射到完整输出范围
    const float value = fmaxf(-1.0f, fminf(1.0f, (float)channel / channel_limit));
    if (fabsf(value) <= deadzone)
    {
        return 0.0f;
    }
    return copysignf((fabsf(value) - deadzone) / (1.0f - deadzone), value);
}

/**
 * @brief 按加减速上限和一阶平滑逼近速度请求，换向时先经过零速
 *
 * @param[in] current 上周期速度，须为有限值，与 requested 单位一致
 * @param[in] requested 本周期请求速度，须为有限值
 * @param[in] accel 加速上限，单位为速度单位每秒，须为有限正值
 * @param[in] decel 减速上限，单位为速度单位每秒，须为有限正值
 * @param[in] period_s 控制周期，单位秒，须为有限正值
 * @param[in] smooth_time_s 一阶平滑时间常数，单位秒，须为有限正值
 * @param[in] settle 直接收敛的误差阈值，与速度单位一致，须为有限非负值
 * @return 平滑后的本周期速度
 */
static float Cmd_SmoothRate(float current, float requested, float accel, float decel,
                            float period_s, float smooth_time_s, float settle)
{
    // 换向先以零速为本周期边界，避免跨过零速后仍沿用减速上限
    const float target = current * requested < 0.0f ? 0.0f : requested;
    const float rate_limit = fabsf(target) < fabsf(current) ? decel : accel;
    const float max_step = rate_limit * period_s;
    const float difference = fabsf(target - current);

    // 小误差有限时间收敛且仍受变化上限约束，避免零速请求留下长尾
    if (difference <= settle && difference <= max_step)
    {
        return target;
    }

    // 实际周期决定平滑比例，较大变化继续由单周期斜坡约束
    const float blend = 1.0f - expf(-period_s / smooth_time_s);
    return RampTo(current, target, fminf(max_step, difference * blend));
}

/**
 * @brief 将遥控快照转换为底盘命令，异常输入输出待机
 *
 * @param[in] rc 最近遥控快照，调用方保证非空
 * @param[in] rc_valid 已取得有效缓存时为 true
 * @param[in] now_us 读取快照后的当前时间，单位微秒
 * @param[out] out 底盘命令，调用方保证非空
 * @return 遥控快照在线且时效有效时返回 true，否则返回 false
 */
bool Cmd_ParseChassis(const DR16_State_t *rc, bool rc_valid, uint64_t now_us, Chassis_Command_t *out)
{
    // 默认撤销运动请求，旧帧、离线和时间回退均不得沿用上一周期输出
    *out = (Chassis_Command_t)
    {
        .mode = CHASSIS_MODE_RELAX,
        .wheel_speed_scale_rpm = CMD_CHASSIS_SCALE_RPM,
    };
    if (!rc_valid || !rc->header.online || now_us < rc->header.last_online_time ||
        now_us - rc->header.last_online_time > DR16_RECEIVER_OFFLINE_TIMEOUT_US)
    {
        return false;
    }

    // 两个运动档都允许平移，下位另叠加自旋，中位及非法拨杆保持零输出
    const float lateral = Cmd_NormalizeChannel(rc->data.ch_l_x, CMD_CHANNEL_LIMIT, CMD_DEADZONE);
    const float forward = Cmd_NormalizeChannel(rc->data.ch_l_y, CMD_CHANNEL_LIMIT, CMD_DEADZONE);
    switch (rc->data.sw_l)
    {
        case DR16_SWITCH_UP:
        case DR16_SWITCH_DOWN:
            out->mode = CHASSIS_MODE_INDEPENDENT;
            out->move.vx = -forward;
            out->move.vy = lateral;
            out->move.wz = rc->data.sw_l == DR16_SWITCH_DOWN ? CMD_CHASSIS_SPIN_WZ : 0.0f;
            break;

        case DR16_SWITCH_MIDDLE:
        case DR16_SWITCH_ERROR:
        default:
            break;
    }
    return true;
}

/**
 * @brief 将遥控快照转换为拨弹命令，异常输入输出离线档
 *
 * 右拨杆下位为运行并输出单发触发电平，右摇杆上下双向线性映射，上位和中位为待机并允许人工解除故障
 *
 * @param[in] rc 最近遥控快照，调用方保证非空
 * @param[in] rc_valid 已取得有效缓存时为 true
 * @param[in] now_us 读取快照后的当前时间，单位微秒
 * @param[out] out 拨弹命令，调用方保证非空
 * @return 遥控快照在线且时效有效时返回 true，否则返回 false
 */
bool Cmd_ParseFeeder(const DR16_State_t *rc, bool rc_valid, uint64_t now_us, Feeder_Command_t *out)
{
    // 默认离线档，失效遥控和非法档位都不得沿用上一周期的速度比例
    *out = (Feeder_Command_t)
    {
        .mode = FEEDER_CMD_OFFLINE,
    };
    if (!rc_valid || !rc->header.online || now_us < rc->header.last_online_time ||
        now_us - rc->header.last_online_time > DR16_RECEIVER_OFFLINE_TIMEOUT_US)
    {
        return false;
    }

    // 下位运行并读取右摇杆，上位和中位撤驱但保留人工解除故障的途径
    switch (rc->data.sw_r)
    {
        case DR16_SWITCH_DOWN:
            out->mode = FEEDER_CMD_RUN;
            // 下位电平同时作为单发触发输入，单发模式由模块取非下位到下位的上升沿
            out->trigger_level = true;
            // 摇杆居中为零，推到底为 +1、推到头为 -1，双向线性映射到速度上限
            out->speed_ratio = Cmd_NormalizeChannel(rc->data.ch_r_y, CMD_CHANNEL_LIMIT, CMD_DEADZONE);
            break;

        case DR16_SWITCH_UP:
        case DR16_SWITCH_MIDDLE:
            out->mode = FEEDER_CMD_RELAX;
            break;

        default:
            break;
    }
    return true;
}

/**
 * @brief 撤销遥控使能与速度历史，要求重新确认中位
 *
 * @param[out] ctx 云台命令上下文，调用方保证非空
 * @return 无返回值
 */
void Cmd_ResetGimbal(Cmd_Gimbal_t *ctx)
{
    *ctx = (Cmd_Gimbal_t)
    {
        .state = CMD_GIMBAL_WAIT_NEUTRAL,
    };
}

/**
 * @brief 解析云台遥控使能顺序与平滑速度，异常输入要求重新确认中位
 *
 * @param[in,out] ctx 云台任务独占上下文，调用方保证非空
 * @param[in] rc 最近遥控快照，调用方保证非空
 * @param[in] rc_valid 已取得有效缓存时为 true
 * @param[in] now_us 读取快照后的当前时间，单位微秒
 * @param[in] period_s 控制周期，单位秒，合法范围 (0, 0.1]
 * @param[in] gimbal_fault 云台任一轴或共用惯性源故障锁存时为 true
 * @param[out] out 云台命令，调用方保证非空
 * @return 快照与周期有效时为 true，否则为 false，调用方据此撤销缓存有效性
 */
bool Cmd_ParseGimbal(Cmd_Gimbal_t *ctx, const DR16_State_t *rc, bool rc_valid, uint64_t now_us,
                     float period_s, bool gimbal_fault, Gimbal_Command_t *out)
{
    // 先撤销本周期使能与前馈，所有异常路径均输出待机
    *out = (Gimbal_Command_t)
    {
        .mode = GIMBAL_MODE_RELAX,
    };
    if (!rc_valid || !rc->header.online ||
        now_us < rc->header.last_online_time ||
        now_us - rc->header.last_online_time > DR16_RECEIVER_OFFLINE_TIMEOUT_US ||
        !isfinite(period_s) || period_s <= 0.0f || period_s > CMD_GIMBAL_MAX_PERIOD_S)
    {
        Cmd_ResetGimbal(ctx);
        return false;
    }
    if (gimbal_fault)
    {
        Cmd_ResetGimbal(ctx);
        return true;
    }

    // 只有中位确认后的上位允许持续运行，停机档和非法状态要求重新确认
    const bool was_active = ctx->state == CMD_GIMBAL_ACTIVE;
    switch (ctx->state)
    {
        case CMD_GIMBAL_WAIT_NEUTRAL:
            if (rc->data.sw_r == DR16_SWITCH_MIDDLE)
            {
                ctx->state = CMD_GIMBAL_READY;
            }
            break;

        case CMD_GIMBAL_READY:
        case CMD_GIMBAL_ACTIVE:
            switch (rc->data.sw_r)
            {
                case DR16_SWITCH_MIDDLE:
                    ctx->state = CMD_GIMBAL_READY;
                    break;
                case DR16_SWITCH_UP:
                    ctx->state = CMD_GIMBAL_ACTIVE;
                    break;
                default:
                    ctx->state = CMD_GIMBAL_WAIT_NEUTRAL;
                    break;
            }
            break;

        default:
            ctx->state = CMD_GIMBAL_WAIT_NEUTRAL;
            break;
    }
    if (ctx->state != CMD_GIMBAL_ACTIVE)
    {
        ctx->yaw_rate_deg_s = ctx->pitch_rate_deg_s = 0.0f;
        return true;
    }
    out->mode = GIMBAL_MODE_ANGLE;
    if (!was_active)
    {
        // 首次使能先交给模块捕获姿态，下一周期才开始遥控速度过渡
        ctx->yaw_rate_deg_s = ctx->pitch_rate_deg_s = 0.0f;
        return true;
    }

    // 同一平滑速度同时用于目标增量和前馈，命令层不依赖云台轴的运行状态
    const float yaw_rate = CMD_GIMBAL_DIRECTION *
        Cmd_NormalizeChannel(rc->data.ch_r_x, CMD_CHANNEL_LIMIT, CMD_DEADZONE) *
        CMD_GIMBAL_RATE_RAD_S * M_RAD2DEG_MULT;
    const float pitch_rate = CMD_GIMBAL_DIRECTION *
        Cmd_NormalizeChannel(rc->data.ch_r_y, CMD_CHANNEL_LIMIT, CMD_DEADZONE) *
        CMD_GIMBAL_RATE_RAD_S * M_RAD2DEG_MULT;
    ctx->yaw_rate_deg_s = Cmd_SmoothRate(ctx->yaw_rate_deg_s, yaw_rate,
        CMD_GIMBAL_ACCEL_DEG_S2, CMD_GIMBAL_DECEL_DEG_S2,
        period_s, CMD_GIMBAL_SMOOTH_TIME_S, CMD_GIMBAL_SETTLE_DEG_S);
    ctx->pitch_rate_deg_s = Cmd_SmoothRate(ctx->pitch_rate_deg_s, pitch_rate,
        CMD_GIMBAL_ACCEL_DEG_S2, CMD_GIMBAL_DECEL_DEG_S2,
        period_s, CMD_GIMBAL_SMOOTH_TIME_S, CMD_GIMBAL_SETTLE_DEG_S);
    out->yaw.rate_rad_s = ctx->yaw_rate_deg_s * CMD_DEG_TO_RAD;
    out->pitch.rate_rad_s = ctx->pitch_rate_deg_s * CMD_DEG_TO_RAD;
    return true;
}
