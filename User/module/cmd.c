#include "module/cmd.h"

#include <math.h>
#include <stddef.h>

#include "component/input_shaping.h"
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
 * @brief 将遥控快照转换为底盘命令，异常输入输出待机
 *
 * @param[in] rc 最近遥控快照，可为 NULL
 * @param[in] rc_valid 已取得有效缓存时为 true
 * @param[in] now_us 读取快照后的当前时间，单位微秒
 * @param[out] out 底盘命令，可为 NULL
 * @return 无返回值
 */
void Cmd_ParseChassis(const DR16_State_t *rc, bool rc_valid, uint64_t now_us, Chassis_Command_t *out)
{
    if (out == NULL)
    {
        return;
    }

    // 默认撤销运动请求，旧帧、离线和时间回退均不得沿用上一周期输出
    *out = (Chassis_Command_t)
    {
        .mode = CHASSIS_MODE_RELAX,
        .wheel_speed_scale_rpm = CMD_CHASSIS_SCALE_RPM,
    };
    if (rc == NULL || !rc_valid || !rc->header.online || now_us < rc->header.last_online_time ||
        now_us - rc->header.last_online_time > DR16_RECEIVER_OFFLINE_TIMEOUT_US)
    {
        return;
    }

    // 两个运动档都允许平移，下位另叠加自旋，中位及非法拨杆保持零输出
    const float lateral = InputShaping_NormalizeChannel(rc->data.ch_l_x, CMD_CHANNEL_LIMIT, CMD_DEADZONE);
    const float forward = InputShaping_NormalizeChannel(rc->data.ch_l_y, CMD_CHANNEL_LIMIT, CMD_DEADZONE);
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
}

/**
 * @brief 撤销遥控使能与速度历史，要求重新确认中位
 *
 * @param[out] ctx 云台命令上下文，可为 NULL
 * @return 无返回值
 */
void Cmd_ResetGimbal(Cmd_Gimbal_t *ctx)
{
    if (ctx != NULL)
    {
        *ctx = (Cmd_Gimbal_t)
        {
            .state = CMD_GIMBAL_WAIT_NEUTRAL,
        };
    }
}

/**
 * @brief 解析云台遥控使能顺序与平滑速度，异常输入要求重新确认中位
 *
 * @param[in,out] ctx 云台任务独占上下文，可为 NULL
 * @param[in] rc 最近遥控快照，可为 NULL
 * @param[in] rc_valid 已取得有效缓存时为 true
 * @param[in] now_us 读取快照后的当前时间，单位微秒
 * @param[in] period_s 控制周期，单位秒，合法范围 (0, 0.1]
 * @param[in] gimbal_fault 云台任一轴或共用惯性源故障锁存时为 true
 * @param[out] out 云台命令，可为 NULL
 * @return 快照与周期有效时为 true，否则为 false，调用方据此撤销缓存有效性
 */
bool Cmd_ParseGimbal(Cmd_Gimbal_t *ctx, const DR16_State_t *rc, bool rc_valid, uint64_t now_us,
                     float period_s, bool gimbal_fault, Gimbal_Command_t *out)
{
    // 先撤销本周期使能与前馈，所有异常路径均输出待机
    if (out != NULL)
    {
        *out = (Gimbal_Command_t)
        {
            .mode = GIMBAL_MODE_RELAX,
        };
    }
    if (ctx == NULL || out == NULL || rc == NULL || !rc_valid || !rc->header.online ||
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
        InputShaping_NormalizeChannel(rc->data.ch_r_x, CMD_CHANNEL_LIMIT, CMD_DEADZONE) *
        CMD_GIMBAL_RATE_RAD_S * M_RAD2DEG_MULT;
    const float pitch_rate = CMD_GIMBAL_DIRECTION *
        InputShaping_NormalizeChannel(rc->data.ch_r_y, CMD_CHANNEL_LIMIT, CMD_DEADZONE) *
        CMD_GIMBAL_RATE_RAD_S * M_RAD2DEG_MULT;
    ctx->yaw_rate_deg_s = InputShaping_SmoothRate(ctx->yaw_rate_deg_s, yaw_rate,
        CMD_GIMBAL_ACCEL_DEG_S2, CMD_GIMBAL_DECEL_DEG_S2,
        period_s, CMD_GIMBAL_SMOOTH_TIME_S, CMD_GIMBAL_SETTLE_DEG_S);
    ctx->pitch_rate_deg_s = InputShaping_SmoothRate(ctx->pitch_rate_deg_s, pitch_rate,
        CMD_GIMBAL_ACCEL_DEG_S2, CMD_GIMBAL_DECEL_DEG_S2,
        period_s, CMD_GIMBAL_SMOOTH_TIME_S, CMD_GIMBAL_SETTLE_DEG_S);
    out->yaw.rate_rad_s = ctx->yaw_rate_deg_s * CMD_DEG_TO_RAD;
    out->pitch.rate_rad_s = ctx->pitch_rate_deg_s * CMD_DEG_TO_RAD;
    return true;
}
