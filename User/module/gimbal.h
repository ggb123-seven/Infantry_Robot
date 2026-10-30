#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "component/pid.h"
#include "component/SMC/slidingmodec.h"
#include "device/motor_rm.h"

/*
 * yaw 单圈角度展开状态：
 * - has_reference：已记录上一帧单圈角度时为 true
 * - last_wrapped_angle：上一帧单圈角度，范围 [0, 360)
 * - continuous_angle：按相邻帧最短路径累计的连续角度，单位度
 */
typedef struct
{
    bool has_reference;
    float last_wrapped_angle;
    float continuous_angle;
} AngleUnwrap_t;

/**
 * @brief 将角度环绕到单圈区间
 *
 * @param[in] angle 有限角度
 * @param[in] period 单圈周期，调用方保证为有限正数
 * @return 单圈角度，范围为 [0, period)
 */
static inline float Gimbal_WrapAngle(float angle, float period)
{
    float wrapped = fmodf(angle, period);
    if (wrapped < 0.0f)
    {
        wrapped += period;
    }
    // 浮点舍入到整圈时统一归到零点
    return wrapped >= period ? 0.0f : wrapped;
}

/**
 * @brief 清除 yaw 连续角参考
 *
 * @param[out] unwrap yaw 连续角状态，调用方保证非空
 * @return 无返回值
 */
static inline void Gimbal_ResetAngleUnwrap(AngleUnwrap_t *unwrap)
{
    unwrap->has_reference = false;
    unwrap->last_wrapped_angle = 0.0f;
    unwrap->continuous_angle = 0.0f;
}

/**
 * @brief 按相邻帧最短路径展开 yaw 单圈角度
 *
 * @param[in,out] unwrap yaw 连续角状态，调用方保证非空
 * @param[in] wrapped_angle 本帧单圈角度，调用方保证范围为 [0, period)
 * @param[in] period 单圈周期，调用方保证为有限正数
 * @param[out] continuous_angle 展开后的连续角度，调用方保证非空
 * @return 成功返回 true，半圈歧义或累计结果异常时返回 false
 */
static inline bool Gimbal_UpdateAngleUnwrap(AngleUnwrap_t *unwrap, float wrapped_angle, float period,
                                            float *continuous_angle)
{
    // 首帧直接以单圈角度建立连续坐标
    if (!unwrap->has_reference)
    {
        unwrap->has_reference = true;
        unwrap->last_wrapped_angle = wrapped_angle;
        unwrap->continuous_angle = wrapped_angle;
        *continuous_angle = wrapped_angle;
        return true;
    }

    // 将相邻帧差值折算到半圈以内，跨越编码器零点时取最短路径
    const float half_period = period * 0.5f;
    float delta = wrapped_angle - unwrap->last_wrapped_angle;
    if (delta > half_period)
    {
        delta -= period;
    }
    else if (delta < -half_period)
    {
        delta += period;
    }

    // 半圈方向无法判定，累计链出口异常时同样丢弃参考
    const float next_angle = unwrap->continuous_angle + delta;
    if (fabsf(delta) >= half_period || !isfinite(next_angle))
    {
        Gimbal_ResetAngleUnwrap(unwrap);
        return false;
    }

    unwrap->last_wrapped_angle = wrapped_angle;
    unwrap->continuous_angle = next_angle;
    *continuous_angle = next_angle;
    return true;
}

/*
 * INS 任务发布的同帧惯性反馈，不改变当前 INS 安装坐标
 * - angle_deg：依次为 Roll、Pitch、累计 Yaw，单位度
 * - gyro_rad_s：当前机体系 X、Y、Z 轴角速度，单位 rad/s
 * - sample_time_us：本帧开始采集的单调时间，单位微秒，接收端不得改写
 * - valid：完整更新成功时为 true，初始化及故障通知为 false
 */
typedef struct
{
    float angle_deg[3];
    float gyro_rad_s[3];
    uint64_t sample_time_us;
    bool valid;
} Gimbal_ImuFeedback_t;

/*
 * 云台控制模式，由命令统一指定两轴：
 * - GIMBAL_MODE_RELAX：两轴零电流，同时解除故障锁存
 * - GIMBAL_MODE_SPEED：只运行速度环，用于单独整定速度环参数
 * - GIMBAL_MODE_ANGLE：yaw 使用滑模直接输出电流，pitch 使用角度与速度串级控制
 */
typedef enum
{
    GIMBAL_MODE_RELAX = 0,
    GIMBAL_MODE_SPEED,
    GIMBAL_MODE_ANGLE,
} Gimbal_Mode_t;

/*
 * 单轴运行状态：
 * - GIMBAL_AXIS_RELAX：待机零输出，反馈和配置有效且命令非 RELAX 时进入运行
 * - GIMBAL_AXIS_RUNNING：闭环运行
 * - GIMBAL_AXIS_FAULT：反馈非法、连续角歧义、零点改变或控制输入异常，零输出锁存至命令回到 RELAX
 * - GIMBAL_AXIS_OFFLINE：电机反馈超时，单轴零输出等待，有效反馈恢复且运行请求有效时自动起控
 */
typedef enum
{
    GIMBAL_AXIS_RELAX = 0,
    GIMBAL_AXIS_RUNNING,
    GIMBAL_AXIS_FAULT,
    GIMBAL_AXIS_OFFLINE,
} Gimbal_AxisState_t;

/*
 * 共用惯性反馈状态：等待有效首帧、可用于闭环、故障锁存
 * 收到有效快照即可就绪，故障后须先回 RELAX，再等待新的有效快照
 * 保留就绪值 2 和故障值 3，兼容现有调试器观测，不再使用原静止检查值 1
 */
typedef enum
{
    GIMBAL_IMU_WAITING = 0,
    GIMBAL_IMU_READY = 2,
    GIMBAL_IMU_FAULT,
} Gimbal_ImuState_t;

/*
 * 单轴配置，运行中可由调试器修改，模块每周期校验并采纳：
 * - enabled：false 时该轴始终零输出
 * - reversed：true 时反转编码器角度、转速和输出电流方向，不改变 INS 朝向
 * - zero_encoder_deg：yaw 正前方或 pitch 水平对应的方向修正后编码器角度，单位度，范围 [0, 360)
 *   修改零点或 reversed 后运行中的该轴闭环进入故障
 * - mechanical_limit_enabled：是否启用编码器机械行程约束
 * - min_angle_deg、max_angle_deg：相对编码器零点的机械角范围，单位度，仅启用机械约束时使用
 * - max_speed_rad_s：速度目标绝对值上限，单位 rad/s
 * - max_current_a：转子侧输出电流绝对值上限，单位 A
 * - angle_pid：角度外环参数，输入为弧度，输出为 rad/s，range 须为 0
 * - speed_pid：速度内环参数，输入为 rad/s，输出为 A
 * PID 增益、积分限幅或输出限幅变化时清空对应积分，d_cutoff_freq 只在初始化时生效
 */
typedef struct
{
    bool enabled;
    bool reversed;
    bool mechanical_limit_enabled;
    float zero_encoder_deg;
    float min_angle_deg;
    float max_angle_deg;
    float max_speed_rad_s;
    float max_current_a;
    KPID_Params_t angle_pid;
    KPID_Params_t speed_pid;
} Gimbal_AxisConfig_t;

/*
 * 云台整体配置：
 * - yaw、pitch：两轴各自配置
 * - feedback_timeout_s：反馈帧最大间隔，单位 s，超过时视为失联并断开连续角
 *   须保证最高转速下该时间内转动小于半圈，才能唯一展开编码器角度
 * - imu_timeout_s：惯性反馈源采样时刻的最大帧龄及相邻帧间隔，单位 s，范围 (0, 0.1]
 */
typedef struct
{
    Gimbal_AxisConfig_t yaw;
    Gimbal_AxisConfig_t pitch;
    float feedback_timeout_s;
    float imu_timeout_s;
} Gimbal_Config_t;

// 两轴独立选择目标来源：遥控使用增量，自瞄及调试轨迹使用绝对角度
typedef enum
{
    GIMBAL_TARGET_INCREMENT = 0,
    GIMBAL_TARGET_ABSOLUTE,
} Gimbal_TargetSource_t;

/*
 * 单轴命令：source 选择目标来源，angle_deg 仅 ABSOLUTE 使用，为当前 INS 坐标下的角度，yaw 连续累计
 * rate_rad_s 在 INCREMENT 下为累加速度兼前馈，在 ABSOLUTE 下为前馈，在 SPEED 模式下为速度目标
 * 模块只读取命令，起控、恢复和模式切换时在内部捕获姿态并清零前馈，不回写调用方
 */
typedef struct
{
    Gimbal_TargetSource_t source;
    float rate_rad_s;
    float angle_deg;
} Gimbal_AxisCommand_t;

/*
 * 模块内部持久设定值，供捕获和保护改写：target_angle_deg 为姿态角，单位度
 * target_speed_rad_s 为速度目标或前馈，单位 rad/s，外部调用方不得写入
 */
typedef struct
{
    float target_angle_deg;
    float target_speed_rad_s;
} Gimbal_AxisSetpoint_t;

/*
 * 云台命令：mode 为两轴共用模式，yaw、pitch 为两轴各自目标
 */
typedef struct
{
    Gimbal_Mode_t mode;
    Gimbal_AxisCommand_t yaw;
    Gimbal_AxisCommand_t pitch;
} Gimbal_Command_t;

/*
 * 单轴运行数据，调试器可直接观察：
 * - state：单轴运行状态
 * - setpoint：模块持有的原始目标与前馈，起控和恢复时可捕获当前姿态，区别于下方实际限幅目标
 * - encoder_deg：方向修正后的编码器单圈角度，单位度，范围 [0, 360)，失联时为非数值
 * - mechanical_angle_deg：编码器相对零点的机械角，单位度，yaw 连续展开，pitch 为 [-180, 180)
 * - motor_speed_rad_s：编码器换算的角速度，单位 rad/s，用于机械观测
 * - angle_deg：当前 INS 坐标下的姿态角，单位度，yaw 为累计 Yaw，pitch 为 Pitch
 * - speed_rad_s：大地系欧拉角速度，单位 rad/s，由机体系陀螺仪按 ZYX 欧拉运动学换算
 *   yaw 为 (ωy·sinRoll + ωz·cosRoll) / cosPitch，pitch 为 ωy·cosRoll − ωz·sinRoll，姿态未就绪时与角度均为非数值
 * - base_speed_rad_s：底座大地系角速度估计，单位 rad/s，等于 speed_rad_s − motor_speed_rad_s
 *   底座水平时精确，倾斜时含投影误差，仅用于观测，任一输入失效时为非数值
 * - target_angle_deg：本周期实际参考角度，单位度，yaw 滑模路径包含参考轨迹约束
 * - target_speed_rad_s：本周期实际参考速度，单位 rad/s，yaw 滑模路径不包含旧 PID 纠偏量
 * - output_current_a：本周期输出电流，单位 A，正方向与角度增大方向一致
 * - feedback_time_us：最近反馈帧时刻，单位微秒，无反馈时为 UINT64_MAX
 * - applied_zero_encoder_deg：当前连续角坐标采用的零点，用于识别零点修改
 * - applied_reversed：当前机械反馈采用的方向，用于识别运行中方向修改
 * - applied_angle_pid、applied_speed_pid：PID 实际使用的参数副本，与配置不同时同步并清空积分
 * - unwrap：yaw 连续角展开器，pitch 不使用
 * - angle_pid、speed_pid：角度外环和速度内环运行状态
 */
typedef struct
{
    Gimbal_AxisState_t state;
    Gimbal_AxisSetpoint_t setpoint;
    float encoder_deg;
    float mechanical_angle_deg;
    float motor_speed_rad_s;
    float angle_deg;
    float speed_rad_s;
    float base_speed_rad_s;
    float target_angle_deg;
    float target_speed_rad_s;
    float output_current_a;
    uint64_t feedback_time_us;
    float applied_zero_encoder_deg;
    bool applied_reversed;
    KPID_Params_t applied_angle_pid;
    KPID_Params_t applied_speed_pid;
    AngleUnwrap_t unwrap;
    KPID_t angle_pid;
    KPID_t speed_pid;
} Gimbal_Axis_t;

/*
 * 云台实例：
 * - config：初始化时复制的配置副本，调试器可在运行中修改
 * - mode：上一周期采用的控制模式
 * - yaw、pitch：两轴运行数据
 * - imu：INS 发布的完整原始快照，角度单位度，角速度单位 rad/s，源时间单位微秒
 * - imu_state：惯性反馈就绪及故障状态
 * 实例内 PID 持有指向自身参数副本的指针，初始化后不得整体复制实例
 */
typedef struct
{
    Gimbal_Config_t config;
    Gimbal_Mode_t mode;
    Gimbal_Axis_t yaw;
    Gimbal_Axis_t pitch;
    Gimbal_ImuFeedback_t imu;
    Gimbal_ImuState_t imu_state;
} Gimbal_t;

/*
 * 云台输出：两轴转子侧电流，单位 A，已按 reversed 换算到电机方向
 */
typedef struct
{
    float yaw_current_a;
    float pitch_current_a;
} Gimbal_Output_t;

/*
 * 单云台实例的滑模调参与观测，复用原有结构体，不改变字段布局
 * gimbal_yaw_smc_config 只允许修改 C、K、error_eps、q、p、u_max、J、epsilon 参数字段
 * Control 每周期复制参数并校验，运行计算仅使用 gimbal_yaw_smc，后者只供观测
 * target、target_dot、target_ddot 为内部参考角度、速度和加速度，单位度、度每秒、度每平方秒
 * u 为滑模候选电流，单位安培，机械边界与输出闸门处理后的实际值观察 gimbal.yaw.output_current_a
 */
extern volatile SMC_t gimbal_yaw_smc_config;
extern SMC_t gimbal_yaw_smc;

/**
 * @brief 复制配置并初始化两轴控制器，初始化后两轴处于 RELAX，仅支持本工程单一云台实例
 *
 * @param[out] gimbal 云台实例
 * @param[in] config 云台配置，内容被复制到实例
 * @param[in] frequency_hz 控制频率，单位 Hz，范围 10 至 2000
 * @return 成功返回 0，参数非法或 PID 初始化失败返回 -1
 */
int8_t Gimbal_Init(Gimbal_t *gimbal, const Gimbal_Config_t *config, float frequency_hz);

/**
 * @brief 接收两轴电机帧及同帧惯性快照，更新机械反馈、惯性反馈和时效
 *
 * @param[in,out] gimbal 云台实例
 * @param[in] yaw_motor yaw 新反馈帧，本周期没有新帧时传 NULL
 * @param[in] pitch_motor pitch 新反馈帧，本周期没有新帧时传 NULL
 * @param[in] imu INS 新快照，本周期没有新帧时传 NULL，不会刷新缓存的源采样时刻
 * @param[in] now_us 当前单调时刻，单位微秒
 * @return 无返回值
 */
void Gimbal_UpdateFeedback(Gimbal_t *gimbal, const MOTOR_RM_Feedback_t *yaw_motor,
                           const MOTOR_RM_Feedback_t *pitch_motor, const Gimbal_ImuFeedback_t *imu, uint64_t now_us);

/**
 * @brief 根据命令更新两轴状态机并计算输出电流
 *
 * ANGLE 的 yaw 使用滑模，pitch 和 SPEED 使用 PID
 * 机械限位、反馈失联、惯性故障和最终零输出闸门仍按原状态机处理
 *
 * @param[in,out] gimbal 云台实例
 * @param[in] command 只读云台命令，调用方保证非空，起控及恢复时内部捕获姿态并清零前馈
 * @param[in] period_s 实际控制间隔，单位 s，范围 (0, 0.1]，超出时运行中的轴进入故障
 * @return 无返回值
 */
void Gimbal_Control(Gimbal_t *gimbal, const Gimbal_Command_t *command, float period_s);

/**
 * @brief 查询任一轴或共用惯性源是否存在锁存故障
 *
 * @param[in] gimbal 云台实例，调用方保证非空
 * @return 存在锁存故障时为 true，电机单轴超时不算锁存故障
 */
static inline bool Gimbal_HasLatchedFault(const Gimbal_t *gimbal)
{
    return gimbal->imu_state == GIMBAL_IMU_FAULT ||
           gimbal->yaw.state == GIMBAL_AXIS_FAULT || gimbal->pitch.state == GIMBAL_AXIS_FAULT;
}

/**
 * @brief 导出两轴电机方向的输出电流
 *
 * @param[in] gimbal 云台实例
 * @param[out] output 云台输出
 * @return 无返回值
 */
void Gimbal_DumpOutput(const Gimbal_t *gimbal, Gimbal_Output_t *output);

#ifdef __cplusplus
}
#endif
