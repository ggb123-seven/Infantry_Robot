/*
 * 遥控命令解析模块
 *
 * 消费 DR16 遥控快照，按拨杆与摇杆生成底盘、云台和拨弹命令，异常输入撤销运行请求
 */

/* Includes ----------------------------------------------------------------- */
#include "module/cmd.h"

#include <math.h>
#include <stddef.h>

#include "component/user_math.h"

/* Private define ----------------------------------------------------------- */
/* 遥控映射参数沿用现有底盘与云台参数，帧龄上限使用 DR16 的 100 毫秒协议约定 */
#define CMD_CHANNEL_LIMIT (660.0f)         /* 通道满量程计数 */
#define CMD_DEADZONE (0.05f)               /* 中心死区比例 */
#define CMD_CHASSIS_SCALE_RPM (100.0f)     /* 底盘轮速尺度，单位：rpm */
#define CMD_CHASSIS_SPIN_WZ (-0.30f)       /* 自旋角速度，单位：rad/s */
#define CMD_GIMBAL_DIRECTION (-1.0f)       /* 云台两轴方向系数 */
#define CMD_GIMBAL_RATE_RAD_S (6.0f)       /* 满杆角速度，单位：rad/s */
#define CMD_GIMBAL_ACCEL_DEG_S2 (60000.0f) /* 加速度上限，单位：度/s² */
#define CMD_GIMBAL_DECEL_DEG_S2 (60000.0f) /* 减速度上限，单位：度/s² */
#define CMD_GIMBAL_SMOOTH_TIME_S (0.004f)  /* 一阶平滑时间常数，单位：s */
#define CMD_GIMBAL_SETTLE_DEG_S (0.1f)     /* 直接收敛误差阈值，单位：度/s */

#if MOTOR_GIMBAL_ENABLE_SINE_TEST
/* yaw 正弦测试振幅 30 度、相位角速度 6 rad/s，相位按 2π 回绕后生成速度前馈 */
#define CMD_GIMBAL_SINE_AMPLITUDE_DEG (30.0f)         /* 振幅，单位：度 */
#define CMD_GIMBAL_SINE_OMEGA_RAD_S (6.0f)            /* 相位角速度，单位：rad/s */
#define CMD_GIMBAL_SINE_TWO_PI (6.28318530718f)       /* 2π，相位回绕周期，单位：rad */
#define CMD_GIMBAL_SINE_DEG_TO_RAD (0.0174532925199f) /* 度转弧度系数 */

/* 正弦轨迹在待机时捕获中心，运行时逐周期推进相位 */
typedef enum
{
    CMD_GIMBAL_SINE_IDLE = 0, /* 待机，等待进入测试 */
    CMD_GIMBAL_SINE_RUNNING,  /* 运行，逐周期推进相位 */
} Cmd_GimbalSineState_t;

static Cmd_GimbalSineState_t gimbal_sine_state; /* 轨迹阶段 */
static float gimbal_sine_phase_rad;             /* 轨迹相位，单位：rad */
static float gimbal_sine_center_deg;            /* 轨迹中心角，单位：度 */
#endif
#define CMD_GIMBAL_MAX_PERIOD_S (0.1f)    /* 云台控制周期合法上限，单位：s */
#define CMD_DEG_TO_RAD (0.0174532925199f) /* 度转弧度系数 */

/* Private function --------------------------------------------------------- */
/**
 * @brief 将原始通道计数限幅归一化并去除中心死区
 *
 * @param channel 去中心后的原始计数
 * @param channel_limit 满量程计数
 * @param deadzone 中心死区比例
 * @return float 死区内为零，死区外线性映射到 [-1, 1]
 */
static float Cmd_NormalizeChannel(int16_t channel, float channel_limit, float deadzone)
{
    /* 先限制满量程，再将死区外的有效行程重新映射到完整输出范围 */
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
 * @param current 上周期速度，与 requested 单位一致
 * @param requested 本周期请求速度
 * @param accel 加速上限，单位为速度单位每秒
 * @param decel 减速上限，单位为速度单位每秒
 * @param period_s 控制周期，单位：s
 * @param smooth_time_s 一阶平滑时间常数，单位：s
 * @param settle 直接收敛的误差阈值，与速度单位一致
 * @return float 平滑后的本周期速度
 */
static float Cmd_SmoothRate(float current, float requested, float accel, float decel,
                            float period_s, float smooth_time_s, float settle)
{
    /* 换向先以零速为本周期边界，避免跨过零速后仍沿用减速上限 */
    const float target = current * requested < 0.0f ? 0.0f : requested;
    const float rate_limit = fabsf(target) < fabsf(current) ? decel : accel;
    const float max_step = rate_limit * period_s;
    const float difference = fabsf(target - current);

    /* 小误差有限时间收敛且仍受变化上限约束，避免零速请求留下长尾 */
    if (difference <= settle && difference <= max_step)
    {
        return target;
    }

    /* 实际周期决定平滑比例，较大变化继续由单周期斜坡约束 */
    const float blend = 1.0f - expf(-period_s / smooth_time_s);
    return RampTo(current, target, fminf(max_step, difference * blend));
}

/* Exported functions ------------------------------------------------------- */
/**
 * @brief 将遥控快照转换为底盘命令，异常输入输出待机
 *
 * @param rc 最近遥控快照
 * @param rc_valid 已取得有效缓存时为 true
 * @param now_us 读取快照后的当前时间，单位：us
 * @param out 底盘命令
 * @return bool 遥控快照在线且时效有效时返回 true
 */
bool Cmd_ParseChassis(const DR16_State_t *rc, bool rc_valid, uint64_t now_us, Chassis_Command_t *out)
{
    /* 默认撤销运动请求，旧帧、离线和时间回退均不得沿用上一周期输出 */
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

    /* 上位零速闭环刹车，中位跟随，下位保持云台坐标平移并顺时针自旋 */
    const float lateral = Cmd_NormalizeChannel(rc->data.ch_l_x, CMD_CHANNEL_LIMIT, CMD_DEADZONE);
    const float forward = Cmd_NormalizeChannel(rc->data.ch_l_y, CMD_CHANNEL_LIMIT, CMD_DEADZONE);
    switch (rc->data.sw_l)
    {
        case DR16_SWITCH_UP:
            out->mode = CHASSIS_MODE_INDEPENDENT;
            break;

        case DR16_SWITCH_MIDDLE:
            out->mode = CHASSIS_MODE_FOLLOW_GIMBAL;
            out->move.vx = -forward;
            out->move.vy = lateral;
            break;

        case DR16_SWITCH_DOWN:
            out->mode = CHASSIS_MODE_SPIN;
            out->move.vx = -forward;
            out->move.vy = lateral;
            out->move.wz = CMD_CHASSIS_SPIN_WZ;
            break;

        case DR16_SWITCH_ERROR:
        default:
            break;
    }
    return true;
}

/**
 * @brief 按右拨杆生成安全、备弹和开火命令，模式切换后要求重新备弹
 *
 * @param ctx 任务独占命令上下文，初始为零
 * @param rc 最近遥控快照
 * @param rc_valid 已取得有效缓存时为 true
 * @param now_us 读取快照后的当前时间，单位：us
 * @param fire_mode 遥控进给模式，0 连发、1 单发、2 五连发
 * @param out 本周期拨弹命令
 * @return bool 遥控与模式有效时为 true，失效或非法档位时为 false
 */
bool Cmd_ParseFeeder(Cmd_Feeder_t *ctx, const DR16_State_t *rc, bool rc_valid, uint64_t now_us,
                     uint8_t fire_mode, Feeder_Command_t *out)
{
    /* 遥控边界统一拒绝旧帧，失联和非法输入撤销备弹记忆 */
    *out = (Feeder_Command_t)
    {
        .mode = FEEDER_CMD_OFFLINE,
    };
    if (!rc_valid || !rc->header.online || now_us < rc->header.last_online_time ||
        now_us - rc->header.last_online_time > DR16_RECEIVER_OFFLINE_TIMEOUT_US)
    {
        ctx->state = CMD_FEEDER_DISABLED;
        return false;
    }

    /* 在线模式入口只校验一次，状态同时记录备弹时选定的模式 */
    Cmd_FeederState_t ready;
    Cmd_FeederState_t active;
    switch (fire_mode)
    {
        case FEEDER_FIRE_MODE_AUTO:
            ready = CMD_FEEDER_AUTO_READY;
            active = CMD_FEEDER_AUTO_ACTIVE;
            break;

        case FEEDER_FIRE_MODE_SINGLE:
            ready = CMD_FEEDER_SINGLE_READY;
            active = CMD_FEEDER_SINGLE_ACTIVE;
            break;

        case FEEDER_FIRE_MODE_BURST:
            ready = CMD_FEEDER_BURST_READY;
            active = CMD_FEEDER_BURST_ACTIVE;
            break;

        default:
            ctx->state = CMD_FEEDER_DISABLED;
            return false;
    }
    out->fire_mode = fire_mode;

    /* 左拨杆和两路右摇杆不参与射击，右拨杆中位备弹、下位直接请求开火 */
    switch (rc->data.sw_r)
    {
        case DR16_SWITCH_UP:
            out->mode = FEEDER_CMD_RELAX;
            ctx->state = CMD_FEEDER_DISABLED;
            break;

        case DR16_SWITCH_MIDDLE:
            out->mode = FEEDER_CMD_RUN;
            ctx->state = ready;
            break;

        case DR16_SWITCH_DOWN:
            out->mode = FEEDER_CMD_RUN;
            ctx->state = active;
            out->speed_ratio = 1.0f;
            out->single_fire = true;
            break;

        default:
            ctx->state = CMD_FEEDER_DISABLED;
            return false;
    }
    return true;
}

/**
 * @brief 撤销遥控运行阶段与速度历史，恢复后重新捕获姿态
 *
 * @param ctx 云台命令上下文
 * @return 无返回值
 */
void Cmd_ResetGimbal(Cmd_Gimbal_t *ctx)
{
    *ctx = (Cmd_Gimbal_t)
    {
        .state = CMD_GIMBAL_INACTIVE,
    };
}

/**
 * @brief 根据遥控环路开关选择两轴共用模式
 *
 * @param command 已解析的遥控命令
 * @param speed_enabled 速度环使能
 * @param angle_enabled 角度环使能
 * @return 无返回值
 */
void Cmd_SelectGimbalLoops(Gimbal_Command_t *command, bool speed_enabled, bool angle_enabled)
{
    /* 失联及故障产生的待机命令不受环路开关覆盖 */
    if (command->mode == GIMBAL_MODE_ANGLE)
    {
        if (!speed_enabled)
        {
            command->mode = GIMBAL_MODE_RELAX;
        }
        else if (!angle_enabled)
        {
            command->mode = GIMBAL_MODE_SPEED;
        }
    }
}

#if MOTOR_GIMBAL_ENABLE_SINE_TEST
/**
 * @brief 根据云台状态生成正弦角度目标与速度前馈
 *
 * @param gimbal 云台实例
 * @param command 本周期命令
 * @param period_s 实际控制间隔，单位：s
 * @param debug_enabled 手动调试开关
 * @param enabled 正弦测试开关
 * @param feedforward_enabled 速度前馈开关
 * @return 无返回值
 */
void Cmd_UpdateGimbalSineTest(const Gimbal_t *gimbal, Gimbal_Command_t *command, float period_s,
                              bool debug_enabled, bool enabled, bool feedforward_enabled)
{
    /* 手动调试独占命令，重新进入遥控时捕获新的轨迹中心 */
    if (debug_enabled)
    {
        gimbal_sine_state = CMD_GIMBAL_SINE_IDLE;
        return;
    }

    const bool active = enabled && command->mode == GIMBAL_MODE_ANGLE &&
                        gimbal->yaw.state == GIMBAL_AXIS_RUNNING && gimbal->imu_state == GIMBAL_IMU_READY &&
                        isfinite(gimbal->yaw.angle_deg) && isfinite(period_s) && period_s > 0.0f;

    /* 依据轨迹运行状态捕获中心、推进相位或退出测试 */
    switch (gimbal_sine_state)
    {
        case CMD_GIMBAL_SINE_IDLE:
            if (!active)
            {
                return;
            }
            gimbal_sine_phase_rad = 0.0f;
            gimbal_sine_center_deg = gimbal->yaw.angle_deg;
            gimbal_sine_state = CMD_GIMBAL_SINE_RUNNING;
            break;

        case CMD_GIMBAL_SINE_RUNNING:
            if (!active)
            {
                command->yaw.rate_rad_s = 0.0f;
                gimbal_sine_phase_rad = 0.0f;
                gimbal_sine_state = CMD_GIMBAL_SINE_IDLE;
                return;
            }
            gimbal_sine_phase_rad = fmodf(gimbal_sine_phase_rad + CMD_GIMBAL_SINE_OMEGA_RAD_S * period_s,
                                          CMD_GIMBAL_SINE_TWO_PI);
            break;

        default:
            command->yaw.rate_rad_s = 0.0f;
            gimbal_sine_phase_rad = 0.0f;
            gimbal_sine_state = CMD_GIMBAL_SINE_IDLE;
            return;
    }

    /* 同一相位生成绝对角度与可选速度前馈 */
    command->yaw.source = GIMBAL_TARGET_ABSOLUTE;
    command->yaw.angle_deg = gimbal_sine_center_deg + CMD_GIMBAL_SINE_AMPLITUDE_DEG * sinf(gimbal_sine_phase_rad);
    command->yaw.rate_rad_s = feedforward_enabled ?
        CMD_GIMBAL_SINE_AMPLITUDE_DEG * CMD_GIMBAL_SINE_DEG_TO_RAD * CMD_GIMBAL_SINE_OMEGA_RAD_S *
        cosf(gimbal_sine_phase_rad) : 0.0f;
}
#endif

/**
 * @brief 解析在线遥控的云台平滑速度，异常输入撤销运行请求
 *
 * @param ctx 云台任务独占上下文
 * @param rc 最近遥控快照
 * @param rc_valid 已取得有效缓存时为 true
 * @param now_us 读取快照后的当前时间，单位：us
 * @param period_s 控制周期，单位：s
 * @param gimbal_fault 云台任一轴或共用惯性源故障锁存时为 true
 * @param out 云台命令
 * @return bool 快照与周期有效时为 true，调用方据此撤销缓存有效性
 */
bool Cmd_ParseGimbal(Cmd_Gimbal_t *ctx, const DR16_State_t *rc, bool rc_valid, uint64_t now_us,
                     float period_s, bool gimbal_fault, Gimbal_Command_t *out)
{
    /* 先撤销本周期使能与前馈，所有异常路径均输出待机 */
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

    /* 遥控正常通信即允许运行，两侧拨杆不参与云台使能，首次上线先捕获姿态 */
    const bool was_active = ctx->state == CMD_GIMBAL_ACTIVE;
    switch (ctx->state)
    {
        case CMD_GIMBAL_INACTIVE:
            ctx->state = CMD_GIMBAL_ACTIVE;
            break;

        case CMD_GIMBAL_ACTIVE:
            break;

        default:
            Cmd_ResetGimbal(ctx);
            return true;
    }
    out->mode = GIMBAL_MODE_ANGLE;
    if (!was_active)
    {
        /* 首次使能先交给模块捕获姿态，下一周期才开始遥控速度过渡 */
        ctx->yaw_rate_deg_s = ctx->pitch_rate_deg_s = 0.0f;
        return true;
    }

    /* 同一平滑速度同时用于目标增量和前馈，命令层不依赖云台轴的运行状态 */
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
