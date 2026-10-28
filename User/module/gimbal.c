#include "module/gimbal_private.h"
#include <math.h>
#include <stddef.h>
#include <string.h>
#include "component/user_math.h"
#include "module/gimbal_governor.h"

/*
 * 控制参数边界：周期上限 0.1 秒，PID 增益上限 1000，用于拒绝异常控制输入和调试器误写
 */
#define GIMBAL_MAX_PERIOD_S (0.1f)
#define GIMBAL_MAX_GAIN (1000.0f)

static bool Gimbal_IsPidConfigValid(const KPID_Params_t *pid);
static bool Gimbal_IsAxisConfigValid(const Gimbal_AxisConfig_t *config);
static bool Gimbal_IsPidChanged(const KPID_Params_t *config, const KPID_Params_t *applied);
static int8_t Gimbal_InitAxis(Gimbal_Axis_t *axis, const Gimbal_AxisConfig_t *config, float frequency_hz);
static bool Gimbal_ApplyAxisConfig(Gimbal_Axis_t *axis, const Gimbal_AxisConfig_t *config);
static void Gimbal_EnterRunning(Gimbal_Axis_t *axis);
static bool Gimbal_RunAxis(Gimbal_Axis_t *axis, const Gimbal_AxisConfig_t *config, Gimbal_Mode_t mode,
                           const Gimbal_AxisSetpoint_t *command, float period_s, bool motion_governed);
static void Gimbal_ControlAxis(Gimbal_Axis_t *axis, const Gimbal_AxisConfig_t *config, Gimbal_Mode_t mode,
                               bool mode_changed, const Gimbal_AxisSetpoint_t *command, float period_s,
                               bool motion_governed);

int8_t Gimbal_Init(Gimbal_t *gimbal, const Gimbal_Config_t *config, float frequency_hz)
{
    if (gimbal == NULL || config == NULL)
    {
        return -1;
    }

    // 先复制配置并清除全部运行数据，任一失败路径都保持两轴零输出
    const Gimbal_Config_t saved_config = *config;
    memset(gimbal, 0, sizeof(*gimbal));
    gimbal->config = saved_config;
    gimbal->mode = GIMBAL_MODE_RELAX;
    Gimbal_ClearFeedback(&gimbal->yaw);
    Gimbal_ClearFeedback(&gimbal->pitch);
    gimbal->yaw.angle_deg = gimbal->pitch.angle_deg = NAN;
    gimbal->yaw.speed_rad_s = gimbal->pitch.speed_rad_s = NAN;
    gimbal->yaw.base_speed_rad_s = gimbal->pitch.base_speed_rad_s = NAN;
    if (!isfinite(frequency_hz) || frequency_hz < 10.0f || frequency_hz > 2000.0f ||
        !isfinite(config->feedback_timeout_s) || config->feedback_timeout_s <= 0.0f ||
        config->feedback_timeout_s > 1.0f || !isfinite(config->imu_timeout_s) ||
        config->imu_timeout_s <= 0.0f || config->imu_timeout_s > 0.1f)
    {
        return -1;
    }

    // 两轴各自校验配置并初始化串级 PID
    if (Gimbal_InitAxis(&gimbal->yaw, &gimbal->config.yaw, frequency_hz) != 0 ||
        Gimbal_InitAxis(&gimbal->pitch, &gimbal->config.pitch, frequency_hz) != 0)
    {
        return -1;
    }
    GimbalGovernor_InitYaw(&gimbal->yaw);
    return 0;
}

/**
 * @brief 将只读单轴命令解析到持久设定值，增量以上周期实际限幅目标为基准
 *
 * @param[in,out] axis 单轴状态及模块持有的设定值
 * @param[in] command 只读单轴命令
 * @param[in] mode 当前控制模式
 * @param[in] period_s 控制周期，单位秒
 * @return 目标来源合法时为 true，非法来源置非数值并返回 false
 */
static bool Gimbal_ResolveSetpoint(Gimbal_Axis_t *axis, const Gimbal_AxisCommand_t *command,
                                   Gimbal_Mode_t mode, float period_s)
{
    // 速度模式直接采用角速度，角度模式再按来源解析姿态目标
    axis->setpoint.target_speed_rad_s = command->rate_rad_s;
    if (mode != GIMBAL_MODE_ANGLE)
    {
        return true;
    }
    switch (command->source)
    {
        case GIMBAL_TARGET_INCREMENT:
            if (axis->state == GIMBAL_AXIS_RUNNING && isfinite(axis->target_angle_deg))
            {
                axis->setpoint.target_angle_deg =
                    axis->target_angle_deg + command->rate_rad_s * M_RAD2DEG_MULT * period_s;
            }
            return true;
        case GIMBAL_TARGET_ABSOLUTE:
            axis->setpoint.target_angle_deg = command->angle_deg;
            return true;
        default:
            axis->setpoint.target_angle_deg = axis->setpoint.target_speed_rad_s = NAN;
            return false;
    }
}

void Gimbal_Control(Gimbal_t *gimbal, const Gimbal_Command_t *command, float period_s)
{
    if (gimbal == NULL)
    {
        return;
    }

    // 空命令按 RELAX 处理，保证调用方异常时两轴零输出
    Gimbal_Command_t relax_command =
    {
        .mode = GIMBAL_MODE_RELAX,
    };
    const Gimbal_Command_t *active_command = command != NULL ? command : &relax_command;
    const Gimbal_Mode_t mode = active_command->mode;
    const bool mode_changed = mode != gimbal->mode;

    // 公开命令先解析为轴内设定值，姿态捕获及保护的覆盖优先于增量
    const bool yaw_valid = Gimbal_ResolveSetpoint(&gimbal->yaw, &active_command->yaw, mode, period_s);
    const bool pitch_valid = Gimbal_ResolveSetpoint(&gimbal->pitch, &active_command->pitch, mode, period_s);

    // 共用源故障只在人工回待机后解除，下一轮等待新的有效快照
    if (mode == GIMBAL_MODE_RELAX && gimbal->imu_state == GIMBAL_IMU_FAULT)
    {
        gimbal->imu_state = GIMBAL_IMU_WAITING;
        gimbal->imu.valid = false;
    }

    // 待机起控或失联恢复时捕获当前姿态，避免追赶停止前的旧目标
    if (mode == GIMBAL_MODE_ANGLE && gimbal->imu_state == GIMBAL_IMU_READY)
    {
        if (yaw_valid &&
            (gimbal->yaw.state == GIMBAL_AXIS_RELAX || gimbal->yaw.state == GIMBAL_AXIS_OFFLINE || mode_changed) &&
            isfinite(gimbal->yaw.angle_deg))
        {
            gimbal->yaw.setpoint.target_angle_deg = gimbal->yaw.angle_deg;
            gimbal->yaw.setpoint.target_speed_rad_s = 0.0f;
        }
        if (pitch_valid &&
            (gimbal->pitch.state == GIMBAL_AXIS_RELAX || gimbal->pitch.state == GIMBAL_AXIS_OFFLINE || mode_changed) &&
            isfinite(gimbal->pitch.angle_deg))
        {
            gimbal->pitch.setpoint.target_angle_deg = gimbal->pitch.angle_deg;
            gimbal->pitch.setpoint.target_speed_rad_s = 0.0f;
        }
    }

    // yaw 保护先跟随当前姿态再捕获新目标，校验失败仍走原故障停机路径，pitch 独立控制
    if (GimbalGovernor_PrepareYaw(gimbal, mode, &gimbal->yaw.setpoint, period_s))
    {
        Gimbal_ControlAxis(&gimbal->yaw, &gimbal->config.yaw, mode, mode_changed,
                           &gimbal->yaw.setpoint, period_s, true);
    }
    else
    {
        Gimbal_StopAxis(&gimbal->yaw, GIMBAL_AXIS_FAULT);
    }
    Gimbal_ControlAxis(&gimbal->pitch, &gimbal->config.pitch, mode, mode_changed, &gimbal->pitch.setpoint,
                       period_s, false);
    gimbal->mode = mode;
}

void Gimbal_DumpOutput(const Gimbal_t *gimbal, Gimbal_Output_t *output)
{
    if (output == NULL)
    {
        return;
    }
    *output = (Gimbal_Output_t)
    {
        0,
    };
    if (gimbal == NULL)
    {
        return;
    }

    // 模块内电流以角度增大方向为正，反装轴在输出时换回电机方向
    const float yaw_current_a = gimbal->yaw.output_current_a;
    const float pitch_current_a = gimbal->pitch.output_current_a;
    output->yaw_current_a = gimbal->config.yaw.reversed ? -yaw_current_a : yaw_current_a;
    output->pitch_current_a = gimbal->config.pitch.reversed ? -pitch_current_a : pitch_current_a;
}

/**
 * @brief 校验单个 PID 参数是否可安全参与计算
 *
 * @param[in] pid PID 参数
 * @return 增益、限幅和截止频率均有限且在合法范围内时返回 true
 */
static bool Gimbal_IsPidConfigValid(const KPID_Params_t *pid)
{
    const bool gains_valid = isfinite(pid->k) && isfinite(pid->p) && isfinite(pid->i) && isfinite(pid->d) &&
                             pid->k >= 0.0f && pid->p >= 0.0f && pid->i >= 0.0f && pid->d >= 0.0f &&
                             pid->k <= GIMBAL_MAX_GAIN && pid->p <= GIMBAL_MAX_GAIN &&
                             pid->i <= GIMBAL_MAX_GAIN && pid->d <= GIMBAL_MAX_GAIN;
    // 输出限幅为零时底层 PID 永不累计积分，连续线性角度要求不启用环绕
    return gains_valid && isfinite(pid->i_limit) && pid->i_limit >= 0.0f && isfinite(pid->out_limit) &&
           pid->out_limit > 0.0f && isfinite(pid->d_cutoff_freq) && pid->range == 0.0f;
}

/**
 * @brief 校验单轴配置是否可安全参与控制
 *
 * @param[in] config 单轴配置
 * @return 零点、角度范围、限幅和两环参数均合法时返回 true
 */
static bool Gimbal_IsAxisConfigValid(const Gimbal_AxisConfig_t *config)
{
    return isfinite(config->zero_encoder_deg) && config->zero_encoder_deg >= 0.0f &&
           config->zero_encoder_deg < GIMBAL_FULL_TURN_DEG && isfinite(config->min_angle_deg) &&
           isfinite(config->max_angle_deg) && config->min_angle_deg <= config->max_angle_deg &&
           isfinite(config->max_speed_rad_s) && config->max_speed_rad_s > 0.0f &&
           isfinite(config->max_current_a) && config->max_current_a > 0.0f &&
           Gimbal_IsPidConfigValid(&config->angle_pid) && Gimbal_IsPidConfigValid(&config->speed_pid);
}

/**
 * @brief 判断配置中的 PID 参数是否与实际采用的副本不同
 *
 * @param[in] config 配置中的 PID 参数
 * @param[in] applied 实际采用的 PID 参数副本
 * @return 影响输出或积分的任一字段不同时返回 true
 */
static bool Gimbal_IsPidChanged(const KPID_Params_t *config, const KPID_Params_t *applied)
{
    return config->k != applied->k || config->p != applied->p || config->i != applied->i ||
           config->d != applied->d || config->i_limit != applied->i_limit || config->out_limit != applied->out_limit;
}

/**
 * @brief 校验单轴配置并初始化串级 PID
 *
 * @param[out] axis 单轴运行数据
 * @param[in] config 单轴配置
 * @param[in] frequency_hz 控制频率，单位 Hz
 * @return 成功返回 0，配置非法或 PID 初始化失败返回 -1
 */
static int8_t Gimbal_InitAxis(Gimbal_Axis_t *axis, const Gimbal_AxisConfig_t *config, float frequency_hz)
{
    if (!Gimbal_IsAxisConfigValid(config))
    {
        return -1;
    }

    // PID 指向实例内的参数副本，调试器修改配置后由每周期比较决定何时同步
    axis->applied_zero_encoder_deg = config->zero_encoder_deg;
    axis->applied_reversed = config->reversed;
    axis->applied_angle_pid = config->angle_pid;
    axis->applied_speed_pid = config->speed_pid;

    // 内环采用大地系欧拉角速度，外环保留当前零微分配置
    if (PID_Init(&axis->angle_pid, KPID_MODE_SET_D, frequency_hz, &axis->applied_angle_pid) != 0 ||
        PID_Init(&axis->speed_pid, KPID_MODE_CALC_D, frequency_hz, &axis->applied_speed_pid) != 0)
    {
        return -1;
    }
    axis->state = GIMBAL_AXIS_RELAX;
    return 0;
}

/**
 * @brief 校验单轴配置并同步零点和 PID 参数副本
 *
 * 零点或方向改变时断开机械连续角，运行中的轴进入故障；PID 参数改变时清空积分，避免旧积分按新增益突变
 *
 * @param[in,out] axis 单轴运行数据
 * @param[in] config 单轴配置
 * @return 配置合法时返回 true，非法时不采纳任何修改并返回 false
 */
static bool Gimbal_ApplyAxisConfig(Gimbal_Axis_t *axis, const Gimbal_AxisConfig_t *config)
{
    if (!Gimbal_IsAxisConfigValid(config))
    {
        return false;
    }

    // 零点或方向变化后机械坐标失效，运行中锁存故障，下一帧重新建立机械角
    if (config->zero_encoder_deg != axis->applied_zero_encoder_deg || config->reversed != axis->applied_reversed)
    {
        axis->applied_zero_encoder_deg = config->zero_encoder_deg;
        axis->applied_reversed = config->reversed;
        Gimbal_ClearFeedback(axis);
        if (axis->state == GIMBAL_AXIS_RUNNING || axis->state == GIMBAL_AXIS_OFFLINE)
        {
            Gimbal_StopAxis(axis, GIMBAL_AXIS_FAULT);
        }
    }

    // 参数副本与配置不同时同步并清空积分，比例和微分项不受历史影响
    if (Gimbal_IsPidChanged(&config->angle_pid, &axis->applied_angle_pid))
    {
        axis->applied_angle_pid = config->angle_pid;
        PID_ResetIntegral(&axis->angle_pid);
    }
    if (Gimbal_IsPidChanged(&config->speed_pid, &axis->applied_speed_pid))
    {
        axis->applied_speed_pid = config->speed_pid;
        PID_ResetIntegral(&axis->speed_pid);
    }
    return true;
}

/**
 * @brief 进入运行或切换模式时清空两环历史，并以当前转速预置速度环微分
 *
 * @param[in,out] axis 单轴运行数据，转速须为有限值
 * @return 无返回值
 */
static void Gimbal_EnterRunning(Gimbal_Axis_t *axis)
{
    PID_Reset(&axis->angle_pid);
    PID_Reset(&axis->speed_pid);

    // 速度环微分作用于反馈，预置上一反馈可避免入口处的微分冲击
    const float scaled_speed = axis->applied_speed_pid.k * axis->speed_rad_s;
    axis->speed_pid.last.k_fb = LowPassFilter2p_Reset(&axis->speed_pid.dfilter, scaled_speed);
    axis->state = GIMBAL_AXIS_RUNNING;
}

/**
 * @brief 执行单轴一个周期的闭环计算
 *
 * @param[in,out] axis 单轴运行数据，须处于 RUNNING
 * @param[in] config 单轴配置，已通过校验
 * @param[in] mode 控制模式，只能为 SPEED 或 ANGLE
 * @param[in] command 单轴命令
 * @param[in] period_s 实际控制间隔，单位 s
 * @param[in] motion_governed 为 true 时按 yaw 转速残差限制电流，pitch 传 false
 * @return 计算成功返回 true；反馈失联、命令或周期非法、计算结果异常时返回 false
 */
static bool Gimbal_RunAxis(Gimbal_Axis_t *axis, const Gimbal_AxisConfig_t *config, Gimbal_Mode_t mode,
                           const Gimbal_AxisSetpoint_t *command, float period_s, bool motion_governed)
{
    // 反馈、命令和周期任一无效时停止计算，由调用方进入故障
    const bool angle_needed = mode == GIMBAL_MODE_ANGLE;
    if (!isfinite(period_s) || period_s <= 0.0f || period_s > GIMBAL_MAX_PERIOD_S ||
        !isfinite(axis->speed_rad_s) || !isfinite(axis->mechanical_angle_deg) ||
        !isfinite(axis->motor_speed_rad_s) || !isfinite(command->target_speed_rad_s) ||
        (angle_needed && (!isfinite(axis->angle_deg) || !isfinite(command->target_angle_deg))))
    {
        return false;
    }
    // SPEED 模式直接采用命令角速度，ANGLE 模式将其作为前馈与角度环修正量合成
    const float max_speed_rad_s = config->max_speed_rad_s;
    const float command_speed_rad_s = AbsClip(command->target_speed_rad_s, max_speed_rad_s);
    float target_speed_rad_s = command_speed_rad_s;
    const float previous_angle_integral = axis->angle_pid.i;
    bool mechanically_limited = false;
    if (angle_needed)
    {
        float target_angle_deg = command->target_angle_deg;
        if (config->mechanical_limit_enabled)
        {
            // 用目标姿态增量预测机械角，底座倾斜时不能直接夹住世界姿态目标
            const float min_target_deg = axis->angle_deg + config->min_angle_deg - axis->mechanical_angle_deg;
            const float max_target_deg = axis->angle_deg + config->max_angle_deg - axis->mechanical_angle_deg;
            Clip(&target_angle_deg, min_target_deg, max_target_deg);
            mechanically_limited = target_angle_deg != command->target_angle_deg;
        }
        axis->target_angle_deg = target_angle_deg;
        // 角度环输入为弧度，微分反馈为大地系欧拉角速度，当前外环微分增益为零
        const float correction_rad_s = PID_Calc(&axis->angle_pid, target_angle_deg * M_DEG2RAD_MULT,
                                                axis->angle_deg * M_DEG2RAD_MULT, axis->speed_rad_s, period_s);
        const float combined_rad_s = correction_rad_s + command_speed_rad_s;
        target_speed_rad_s = AbsClip(combined_rad_s, max_speed_rad_s);
        // 合成速度饱和且误差继续推向饱和方向时撤销本周期积分，保留反向退饱和能力
        if ((combined_rad_s > max_speed_rad_s && target_angle_deg > axis->angle_deg) ||
            (combined_rad_s < -max_speed_rad_s && target_angle_deg < axis->angle_deg))
        {
            axis->angle_pid.i = previous_angle_integral;
        }
    }
    else
    {
        axis->target_angle_deg = NAN;
    }
    // 速度模式及前馈同样受机械边界约束，越界时只允许零速或向行程内部运动
    if (config->mechanical_limit_enabled)
    {
        const float scale = M_DEG2RAD_MULT / period_s;
        const float min_speed = fminf(0.0f, (config->min_angle_deg - axis->mechanical_angle_deg) * scale);
        const float max_speed = fmaxf(0.0f, (config->max_angle_deg - axis->mechanical_angle_deg) * scale);
        const float requested_speed = target_speed_rad_s;
        Clip(&target_speed_rad_s, min_speed, max_speed);
        mechanically_limited = mechanically_limited || requested_speed != target_speed_rad_s;
    }
    // 制动及目标跟随时暂停姿态纠偏，捕获新目标后平滑恢复速度权限
    if (motion_governed)
    {
        target_speed_rad_s = GimbalGovernor_LimitSpeed(target_speed_rad_s);
    }
    // 速度环输出电流，非有限结果视为计算异常
    const float previous_speed_integral = axis->speed_pid.i;
    const float current_a = PID_Calc(&axis->speed_pid, target_speed_rad_s, axis->speed_rad_s, 0.0f, period_s);
    if (!isfinite(target_speed_rad_s) || !isfinite(current_a))
    {
        return false;
    }
    axis->target_speed_rad_s = target_speed_rad_s;
    axis->output_current_a = AbsClip(current_a, config->max_current_a);
    const float candidate_a = axis->output_current_a;
    if (motion_governed)
    {
        axis->output_current_a = GimbalGovernor_ControlCurrent(axis->motor_speed_rad_s, axis->speed_rad_s,
                                                              candidate_a, config->max_current_a, period_s);
    }
    // 边界处阻断指向行程外侧的电流，避免底座运动或积分残留绕过目标限幅
    if (config->mechanical_limit_enabled &&
        ((axis->mechanical_angle_deg >= config->max_angle_deg && axis->output_current_a > 0.0f) ||
         (axis->mechanical_angle_deg <= config->min_angle_deg && axis->output_current_a < 0.0f)))
    {
        axis->output_current_a = 0.0f;
        axis->speed_pid.i = previous_speed_integral;
        mechanically_limited = true;
    }
    // 用机械边界处理后的实际电流反算，避免下游阻断制动后仍按未执行的电流调整积分
    if (motion_governed)
    {
        GimbalGovernor_TrackCurrent(axis, candidate_a, previous_speed_integral, previous_angle_integral, period_s);
    }
    if (mechanically_limited)
    {
        axis->angle_pid.i = previous_angle_integral;
    }
    return isfinite(axis->output_current_a) && isfinite(axis->speed_pid.i);
}

/**
 * @brief 单轴状态机，集中决定起控、失联恢复、停止和故障锁存
 *
 * @param[in,out] axis 单轴运行数据
 * @param[in] config 单轴配置
 * @param[in] mode 本周期控制模式
 * @param[in] mode_changed 本周期模式与上一周期不同时为 true
 * @param[in] command 单轴命令
 * @param[in] period_s 实际控制间隔，单位 s
 * @param[in] motion_governed 为 true 时启用 yaw 残差电流包络
 * @return 无返回值
 */
static void Gimbal_ControlAxis(Gimbal_Axis_t *axis, const Gimbal_AxisConfig_t *config, Gimbal_Mode_t mode,
                               bool mode_changed, const Gimbal_AxisSetpoint_t *command, float period_s,
                               bool motion_governed)
{
    // 先采纳配置修改，再根据模式和反馈判断本周期能否闭环
    const bool config_valid = Gimbal_ApplyAxisConfig(axis, config);
    const bool run_requested = config->enabled && (mode == GIMBAL_MODE_SPEED || mode == GIMBAL_MODE_ANGLE);
    const bool feedback_ready = isfinite(axis->speed_rad_s) && isfinite(axis->angle_deg) &&
                                isfinite(axis->mechanical_angle_deg) && isfinite(axis->motor_speed_rad_s);

    switch (axis->state)
    {
        // 待机与失联恢复共用起控路径，失联期间仍校验共用反馈和控制输入
        case GIMBAL_AXIS_RELAX:
        case GIMBAL_AXIS_OFFLINE:
            if (axis->state == GIMBAL_AXIS_OFFLINE && run_requested &&
                (!config_valid || !isfinite(axis->angle_deg) || !isfinite(axis->speed_rad_s) ||
                !isfinite(period_s) || period_s <= 0.0f || period_s > GIMBAL_MAX_PERIOD_S ||
                !isfinite(command->target_speed_rad_s) ||
                (mode == GIMBAL_MODE_ANGLE && !isfinite(command->target_angle_deg))))
            {
                Gimbal_StopAxis(axis, GIMBAL_AXIS_FAULT);
                break;
            }
            if (!run_requested || !config_valid || !feedback_ready)
            {
                Gimbal_StopAxis(axis, run_requested && axis->state == GIMBAL_AXIS_OFFLINE ?
                    GIMBAL_AXIS_OFFLINE : GIMBAL_AXIS_RELAX);
                break;
            }
            Gimbal_EnterRunning(axis);
            if (!Gimbal_RunAxis(axis, config, mode, command, period_s, motion_governed))
            {
                Gimbal_StopAxis(axis, GIMBAL_AXIS_FAULT);
            }
            break;

        // 运行中撤销请求正常停机，配置或反馈异常锁存故障，切换模式时清空两环历史
        case GIMBAL_AXIS_RUNNING:
            if (!run_requested)
            {
                Gimbal_StopAxis(axis, GIMBAL_AXIS_RELAX);
                break;
            }
            if (mode_changed)
            {
                Gimbal_EnterRunning(axis);
            }
            if (!config_valid || !Gimbal_RunAxis(axis, config, mode, command, period_s, motion_governed))
            {
                Gimbal_StopAxis(axis, GIMBAL_AXIS_FAULT);
            }
            break;

        // 故障锁存至命令回到 RELAX，避免反馈恢复后在未知位置自动重新出力
        case GIMBAL_AXIS_FAULT:
            Gimbal_StopAxis(axis, mode == GIMBAL_MODE_RELAX ? GIMBAL_AXIS_RELAX : GIMBAL_AXIS_FAULT);
            break;

        default:
            Gimbal_StopAxis(axis, GIMBAL_AXIS_FAULT);
            break;
    }
}
