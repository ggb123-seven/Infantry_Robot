#include "module/gimbal.h"
#include <math.h>
#include <stddef.h>
#include <string.h>
#include "component/user_math.h"
#include "module/gimbal_governor.h"

/*
 * 参数沿用合并前值：周期上限 0.1 秒、PID 增益上限 1000；rpm 按每分钟 60 秒、每圈 2π 换算，完整一圈为 360 度
 * 电机转速按有符号 16 位协议校验；惯性角速度上限 40 rad/s，俯仰奇异点与相邻姿态跳变量各保留 1 度余量
 */
#define GIMBAL_MAX_PERIOD_S (0.1f)
#define GIMBAL_MAX_GAIN (1000.0f)
#define GIMBAL_RPM_TO_RAD_S (0.10471975512f)
#define GIMBAL_RAW_SPEED_MIN_RPM (-32768.0f)
#define GIMBAL_RAW_SPEED_MAX_RPM (32767.0f)
#define GIMBAL_IMU_MAX_RAD_S (40.0f)
#define GIMBAL_IMU_MAX_PITCH_DEG (89.0f)
#define GIMBAL_IMU_JUMP_MARGIN_DEG (1.0f)
#define GIMBAL_FULL_TURN_DEG (360.0f)

/*
 * 本分支首次滑模试验的参考约束：速度 60 度每秒，加速度 180 度每平方秒
 * 参考追踪增益 5 每秒用于平滑绝对目标跳变，均为未上板试验值，不修改原默认配置
 * 滑模参数 C=5、K=10 每秒，q/p=21/27，近零过渡尺度 1 度
 * J=0.001 安培乘平方秒每度，趋近项 0.5 度每平方秒，候选电流上限 0.5 安培
 * 单云台任务独占配置快照和运行上下文，与现有 governor 的单实例约束一致
 */
#define GIMBAL_SMC_MAX_SPEED_DEG_S (60.0f)
#define GIMBAL_SMC_MAX_ACCEL_DEG_S2 (180.0f)
#define GIMBAL_SMC_REFERENCE_GAIN (5.0f)

volatile SMC_t gimbal_yaw_smc_config =
{
    .C = 5.0f, .K = 10.0f,
    .error_eps = 1.0f,
    .q = 21.0f, .p = 27.0f,
    .u_max = 0.5f,
    .J = 0.001f,
    .epsilon = 0.5f,
};
SMC_t gimbal_yaw_smc;

static Gimbal_Config_t control_config;
static bool config_valid;
static bool initialized;
static bool smc_reset;
static Gimbal_Axis_t *yaw_axis;
static bool imu_ready;

static bool Gimbal_IsConfigValid(const Gimbal_Config_t *config);

static void Gimbal_UpdateImu(Gimbal_t *gimbal, const Gimbal_ImuFeedback_t *imu, uint64_t now_us);
static void Gimbal_UpdateAxisFeedback(Gimbal_Axis_t *axis, const Gimbal_AxisConfig_t *config,
                                      const MOTOR_RM_Feedback_t *motor, uint64_t now_us, uint64_t timeout_us,
                                      bool signed_angle);

static void Gimbal_ClearFeedback(Gimbal_Axis_t *axis);
static void Gimbal_StopAxis(Gimbal_Axis_t *axis, Gimbal_AxisState_t next_state);
static int8_t Gimbal_InitAxis(Gimbal_Axis_t *axis, const Gimbal_AxisConfig_t *config, float frequency_hz);
static void Gimbal_ApplyAxisConfig(Gimbal_Axis_t *axis, const Gimbal_AxisConfig_t *config);
static void Gimbal_ControlAxis(Gimbal_Axis_t *axis, const Gimbal_AxisConfig_t *config, Gimbal_Mode_t mode,
                               bool mode_changed, const Gimbal_AxisSetpoint_t *command, float period_s,
                               bool motion_governed);

int8_t Gimbal_Init(Gimbal_t *gimbal, const Gimbal_Config_t *config, float frequency_hz)
{
    initialized = false;
    config_valid = false;
    imu_ready = false;
    yaw_axis = &gimbal->yaw;
    // 先复制配置并清除全部运行数据，任一失败路径都保持两轴零输出
    const Gimbal_Config_t saved_config = *config;
    memset(gimbal, 0, sizeof(*gimbal));
    gimbal->config = saved_config;
    Gimbal_ClearFeedback(&gimbal->yaw);
    Gimbal_ClearFeedback(&gimbal->pitch);
    gimbal->yaw.angle_deg = gimbal->pitch.angle_deg = NAN;
    gimbal->yaw.speed_rad_s = gimbal->pitch.speed_rad_s = NAN;
    gimbal->yaw.base_speed_rad_s = gimbal->pitch.base_speed_rad_s = NAN;
    const SMC_t smc_config = gimbal_yaw_smc_config;
    config_valid = Gimbal_IsConfigValid(&saved_config);
    if (!config_valid || !SMC_IsConfigValid(&smc_config) ||
        !isfinite(frequency_hz) || frequency_hz < 10.0f || frequency_hz > 2000.0f)
    {
        return -1;
    }
    control_config = saved_config;
    SMC_Init(&gimbal_yaw_smc, smc_config.C, smc_config.K, 0.0f, smc_config.error_eps,
             smc_config.q, smc_config.p, smc_config.u_max, smc_config.J, smc_config.epsilon);
    smc_reset = true;
    // 参数已集中校验，两轴只建立指向持久副本的 PID 实例
    if (Gimbal_InitAxis(&gimbal->yaw, &control_config.yaw, frequency_hz) != 0 ||
        Gimbal_InitAxis(&gimbal->pitch, &control_config.pitch, frequency_hz) != 0)
    {
        return -1;
    }
    GimbalGovernor_InitYaw(&gimbal->yaw);
    initialized = true;
    return 0;
}
/**
 * @brief 将只读单轴命令解析到持久设定值，增量以上周期实际限幅目标为基准
 *
 * @param[in,out] axis 单轴状态及模块持有的设定值
 * @param[in] command 只读单轴命令
 * @param[in] mode 当前控制模式
 * @param[in] mode_changed 本周期是否切换模式，切入角度模式时捕获当前姿态
 * @param[in] period_s 控制周期，单位秒
 * @return 命令数值和来源有效时为真，非法时为假，由控制入口进入故障
 */
static bool Gimbal_ResolveSetpoint(Gimbal_Axis_t *axis, const Gimbal_AxisCommand_t *command,
                                   Gimbal_Mode_t mode, bool mode_changed, float period_s)
{
    // 命令边界先拒绝非法数值，起控捕获不得掩盖输入错误
    axis->setpoint.target_speed_rad_s = command->rate_rad_s;
    if (mode != GIMBAL_MODE_RELAX && !isfinite(command->rate_rad_s))
    {
        return false;
    }
    if (mode == GIMBAL_MODE_ANGLE)
    {
        switch (command->source)
        {
            case GIMBAL_TARGET_INCREMENT:
                axis->setpoint.target_angle_deg =
                    (axis->state == GIMBAL_AXIS_RUNNING && !mode_changed ? axis->target_angle_deg : 0.0f) +
                    command->rate_rad_s * M_RAD2DEG_MULT * period_s;
                break;
            case GIMBAL_TARGET_ABSOLUTE:
                axis->setpoint.target_angle_deg = command->angle_deg;
                break;
            default:
                return false;
        }
        // 未运行或切换模式时不沿用旧目标，先校验增量换算再由下方捕获当前姿态
        // 解析出口统一拒绝非法绝对角及增量换算溢出，起控捕获不能覆盖计算异常
        if (!isfinite(axis->setpoint.target_angle_deg))
        {
            return false;
        }
        // 首次进入角度模式及失联恢复均捕获当前姿态，不追赶旧目标
        if (imu_ready && axis->feedback_time_us != UINT64_MAX &&
            (axis->state == GIMBAL_AXIS_RELAX || axis->state == GIMBAL_AXIS_OFFLINE ||
             mode_changed))
        {
            axis->setpoint.target_angle_deg = axis->angle_deg;
            axis->setpoint.target_speed_rad_s = 0.0f;
        }
    }
    return true;
}

void Gimbal_ControlWithChassisFeedback(Gimbal_t *gimbal, const Gimbal_Command_t *command,
                                       const Gimbal_ChassisFeedback_t *chassis, uint64_t now_us, float period_s)
{
    // 在线调参每周期只取一次快照，任何非法配置都不得参与本周期控制
    const Gimbal_Config_t snapshot = *(const volatile Gimbal_Config_t *)&gimbal->config;
    const SMC_t smc_config = gimbal_yaw_smc_config;
    const Gimbal_Mode_t mode = command->mode;
    const bool mode_changed = mode != gimbal->mode;
    config_valid = Gimbal_IsConfigValid(&snapshot);
    if (!initialized || !config_valid || !isfinite(period_s) || period_s <= 0.0f ||
        period_s > GIMBAL_MAX_PERIOD_S || (unsigned)mode > GIMBAL_MODE_ANGLE)
    {
        Gimbal_StopAxis(&gimbal->yaw, GIMBAL_AXIS_FAULT);
        Gimbal_StopAxis(&gimbal->pitch, GIMBAL_AXIS_FAULT);
        return;
    }
    control_config = snapshot;
    imu_ready = gimbal->imu_state == GIMBAL_IMU_READY;
    Gimbal_ApplyAxisConfig(&gimbal->yaw, &control_config.yaw);
    Gimbal_ApplyAxisConfig(&gimbal->pitch, &control_config.pitch);
    // 参数变更仅在本入口采纳，算法内部及输出函数均不读取可写配置
    const bool smc_valid = SMC_IsConfigValid(&smc_config);
    if (smc_valid)
    {
        gimbal_yaw_smc.C = smc_config.C;
        gimbal_yaw_smc.K = smc_config.K;
        gimbal_yaw_smc.error_eps = smc_config.error_eps;
        gimbal_yaw_smc.q = smc_config.q;
        gimbal_yaw_smc.p = smc_config.p;
        gimbal_yaw_smc.u_max = fminf(smc_config.u_max, control_config.yaw.max_current_a);
        gimbal_yaw_smc.J = smc_config.J;
        gimbal_yaw_smc.epsilon = smc_config.epsilon;
    }
    // 两轴命令分别校验，单轴非法命令不掩盖另一轴的运行状态
    const bool yaw_valid = Gimbal_ResolveSetpoint(&gimbal->yaw, &command->yaw, mode, mode_changed, period_s);
    const bool pitch_valid = Gimbal_ResolveSetpoint(&gimbal->pitch, &command->pitch, mode, mode_changed, period_s);
    if (!pitch_valid)
    {
        Gimbal_StopAxis(&gimbal->pitch, GIMBAL_AXIS_FAULT);
    }
    // 共用源故障只在人工回待机后解除，下一轮等待新的有效快照
    if (mode == GIMBAL_MODE_RELAX && gimbal->imu_state == GIMBAL_IMU_FAULT)
    {
        gimbal->imu_state = GIMBAL_IMU_WAITING;
        gimbal->imu.valid = false;
    }
    // yaw 保护先跟随当前姿态再捕获新目标，校验失败仍走原故障停机路径，pitch 独立控制
    if (yaw_valid && (mode != GIMBAL_MODE_ANGLE || smc_valid) &&
        GimbalGovernor_PrepareYawWithChassis(gimbal, mode, &gimbal->yaw.setpoint, chassis, now_us, period_s,
                                            control_config.yaw.enabled))
    {
        Gimbal_ControlAxis(&gimbal->yaw, &control_config.yaw, mode, mode_changed,
                           &gimbal->yaw.setpoint, period_s, true);
    }
    else
    {
        Gimbal_StopAxis(&gimbal->yaw, GIMBAL_AXIS_FAULT);
    }
    Gimbal_ControlAxis(&gimbal->pitch, &control_config.pitch, mode, mode_changed, &gimbal->pitch.setpoint,
                       period_s, false);
    gimbal->mode = mode;
}

void Gimbal_DumpOutput(const Gimbal_t *gimbal, Gimbal_Output_t *output)
{
    // 唯一输出闸门只依赖已确定的状态，禁止重新读取在线配置或重复检查中间量
    const bool running = initialized && config_valid && gimbal->imu_state == GIMBAL_IMU_READY;
    const float yaw_current_a = running && gimbal->yaw.state == GIMBAL_AXIS_RUNNING ?
                                gimbal->yaw.output_current_a : 0.0f;
    const float pitch_current_a = running && gimbal->pitch.state == GIMBAL_AXIS_RUNNING ?
                                  gimbal->pitch.output_current_a : 0.0f;
    output->yaw_current_a = gimbal->yaw.applied_reversed ? -yaw_current_a : yaw_current_a;
    output->pitch_current_a = gimbal->pitch.applied_reversed ? -pitch_current_a : pitch_current_a;
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
 * @brief 在初始化及控制入口集中校验完整云台参数快照
 *
 * @param[in] config 本周期参数快照，调用方保证非空
 * @return 两轴参数和反馈超时范围均有效时为真
 */
static bool Gimbal_IsConfigValid(const Gimbal_Config_t *config)
{
    return Gimbal_IsAxisConfigValid(&config->yaw) && Gimbal_IsAxisConfigValid(&config->pitch) &&
           isfinite(config->feedback_timeout_s) && config->feedback_timeout_s > 0.0f &&
           config->feedback_timeout_s <= 1.0f && isfinite(config->imu_timeout_s) &&
           config->imu_timeout_s > 0.0f && config->imu_timeout_s <= GIMBAL_MAX_PERIOD_S;
}
/**
 * @brief 使用入口已校验的单轴配置初始化串级 PID
 *
 * @param[out] axis 单轴运行数据
 * @param[in] config 单轴配置
 * @param[in] frequency_hz 控制频率，单位 Hz
 * @return 成功返回 0，PID 初始化失败返回 -1
 */
static int8_t Gimbal_InitAxis(Gimbal_Axis_t *axis, const Gimbal_AxisConfig_t *config, float frequency_hz)
{
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
    return 0;
}
/**
 * @brief 在控制入口同步有效配置，零点或方向变化使运行轴故障，PID 参数变化清空对应积分
 *
 * @param[in,out] axis 单轴运行数据
 * @param[in] config 单轴配置，调用方保证已在控制入口集中校验
 * @return 无返回值
 */
static void Gimbal_ApplyAxisConfig(Gimbal_Axis_t *axis, const Gimbal_AxisConfig_t *config)
{
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
    if (memcmp(&config->angle_pid, &axis->applied_angle_pid, sizeof(config->angle_pid)) != 0)
    {
        axis->applied_angle_pid = config->angle_pid;
        PID_ResetIntegral(&axis->angle_pid);
    }
    if (memcmp(&config->speed_pid, &axis->applied_speed_pid, sizeof(config->speed_pid)) != 0)
    {
        axis->applied_speed_pid = config->speed_pid;
        PID_ResetIntegral(&axis->speed_pid);
    }
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
    if (axis == yaw_axis)
    {
        smc_reset = true;
    }
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
 * @return 计算成功返回 true，浮点计算链出口异常时返回 false
 */
static bool Gimbal_RunAxis(Gimbal_Axis_t *axis, const Gimbal_AxisConfig_t *config, Gimbal_Mode_t mode,
                           const Gimbal_AxisSetpoint_t *command, float period_s, bool motion_governed)
{
    const bool angle_needed = mode == GIMBAL_MODE_ANGLE;
    const bool use_smc = motion_governed && angle_needed;
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
        if (!use_smc)
        {
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
    // yaw 角度模式直接产生安培电流，其他路径继续使用原速度环
    const float previous_speed_integral = axis->speed_pid.i;
    if (use_smc)
    {
        // 起控或保护捕获时只重置参考，真实反馈速度继续参与制动
        if (smc_reset || GimbalGovernor_IsTracking())
        {
            gimbal_yaw_smc.target = axis->angle_deg;
            gimbal_yaw_smc.target_dot = gimbal_yaw_smc.target_ddot = 0.0f;
            smc_reset = false;
        }
        else
        {
            const float max_speed = fminf(GIMBAL_SMC_MAX_SPEED_DEG_S, max_speed_rad_s * M_RAD2DEG_MULT);
            SMC_AdvanceReference(&gimbal_yaw_smc, axis->target_angle_deg, target_speed_rad_s * M_RAD2DEG_MULT,
                                  max_speed, GIMBAL_SMC_MAX_ACCEL_DEG_S2, GIMBAL_SMC_REFERENCE_GAIN, period_s);
        }
        axis->target_angle_deg = gimbal_yaw_smc.target;
        axis->target_speed_rad_s = gimbal_yaw_smc.target_dot * M_DEG2RAD_MULT;
        if (!SMC_Tick(&gimbal_yaw_smc, axis->angle_deg, axis->speed_rad_s * M_RAD2DEG_MULT,
                      gimbal_yaw_smc.target_dot, gimbal_yaw_smc.target_ddot))
        {
            return false;
        }
        axis->output_current_a = gimbal_yaw_smc.u;
    }
    else
    {
        axis->target_speed_rad_s = target_speed_rad_s;
        const float current_a = PID_Calc(&axis->speed_pid, target_speed_rad_s, axis->speed_rad_s, 0.0f, period_s);
        axis->output_current_a = AbsClip(current_a, config->max_current_a);
    }
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
        if (!use_smc)
        {
            axis->speed_pid.i = previous_speed_integral;
        }
        mechanically_limited = true;
    }
    // 用机械边界处理后的实际电流反算，避免下游阻断制动后仍按未执行的电流调整积分
    if (motion_governed)
    {
        if (use_smc)
        {
            GimbalGovernor_CommitCurrent(axis->output_current_a);
        }
        else
        {
            GimbalGovernor_TrackCurrent(axis, candidate_a, previous_speed_integral, previous_angle_integral, period_s);
        }
    }
    if (mechanically_limited && !use_smc)
    {
        axis->angle_pid.i = previous_angle_integral;
    }
    return isfinite(axis->output_current_a);
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
    // 调用方已校验并采纳本周期配置，此处只推进轴状态机
    const bool run_requested = config->enabled && (mode == GIMBAL_MODE_SPEED || mode == GIMBAL_MODE_ANGLE);
    const bool feedback_ready = axis->feedback_time_us != UINT64_MAX && imu_ready;
    switch (axis->state)
    {
        // 待机与失联恢复共用起控路径，失联期间仍校验共用反馈和控制输入
        case GIMBAL_AXIS_RELAX:
        case GIMBAL_AXIS_OFFLINE:
            if (!run_requested || !feedback_ready)
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
            if (!feedback_ready)
            {
                Gimbal_StopAxis(axis, GIMBAL_AXIS_FAULT);
                break;
            }
            if (!run_requested)
            {
                Gimbal_StopAxis(axis, GIMBAL_AXIS_RELAX);
                break;
            }
            if (mode_changed)
            {
                Gimbal_EnterRunning(axis);
            }
            if (!Gimbal_RunAxis(axis, config, mode, command, period_s, motion_governed))
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

void Gimbal_UpdateFeedback(Gimbal_t *gimbal, const MOTOR_RM_Feedback_t *yaw_motor,
                           const MOTOR_RM_Feedback_t *pitch_motor, const Gimbal_ImuFeedback_t *imu, uint64_t now_us)
{
    // 反馈沿用最近一次有效快照，在线调参仅由稍后的控制入口统一校验与采纳
    const uint64_t timeout_us = (uint64_t)(control_config.feedback_timeout_s * 1.0e6f);
    Gimbal_UpdateAxisFeedback(&gimbal->yaw, &control_config.yaw, yaw_motor, now_us, timeout_us, false);
    Gimbal_UpdateAxisFeedback(&gimbal->pitch, &control_config.pitch, pitch_motor, now_us, timeout_us, true);
    // 惯性快照独立校验时效和连续性，控制量只由该快照更新
    Gimbal_UpdateImu(gimbal, imu, now_us);
    // 两侧反馈更新后估计底座大地系角速度，底座水平时与关节共轴，任一侧为非数值时结果同为非数值
    gimbal->yaw.base_speed_rad_s = gimbal->yaw.speed_rad_s - gimbal->yaw.motor_speed_rad_s;
    gimbal->pitch.base_speed_rad_s = gimbal->pitch.speed_rad_s - gimbal->pitch.motor_speed_rad_s;
}
/**
 * @brief 撤销机械反馈并断开连续角参考
 * @param[out] axis 单轴运行数据
 * @return 无返回值
 */
static void Gimbal_ClearFeedback(Gimbal_Axis_t *axis)
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
    const float timeout_s = control_config.imu_timeout_s;
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
 * @brief 换算单轴反馈，超时进入失联等待，非法帧、时间倒退或展开歧义锁存故障
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
/**
 * @brief 清空控制历史并进入停止状态，最终电流仅由输出闸门阻断
 *
 * @param[in,out] axis 单轴运行数据
 * @param[in] next_state RELAX、OFFLINE 或 FAULT，其他值按 FAULT 处理
 * @return 无返回值
 */
static void Gimbal_StopAxis(Gimbal_Axis_t *axis, Gimbal_AxisState_t next_state)
{
    PID_Reset(&axis->angle_pid);
    PID_Reset(&axis->speed_pid);
    axis->target_angle_deg = NAN;
    axis->target_speed_rad_s = 0.0f;
    if (axis == yaw_axis)
    {
        smc_reset = true;
    }
    axis->state = next_state == GIMBAL_AXIS_RELAX || next_state == GIMBAL_AXIS_OFFLINE ? next_state : GIMBAL_AXIS_FAULT;
}
