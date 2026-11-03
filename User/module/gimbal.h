#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "component/pid.h"
#include "device/motor_rm.h"

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
 * - GIMBAL_MODE_ANGLE：角度与速度串级运行，命令角速度作为前馈叠加
 */
typedef enum
{
    GIMBAL_MODE_RELAX = 0,
    GIMBAL_MODE_SPEED,
    GIMBAL_MODE_ANGLE,
} Gimbal_Mode_t;

/*
 * 底盘任务发布的运行状态，在线状态与已采纳的自旋命令共同决定：
 * - GIMBAL_CHASSIS_OFFLINE：四个底盘电机全部无有效在线反馈，单独调试云台时也使用此值
 * - GIMBAL_CHASSIS_ONLINE：至少一个底盘电机在线，未进入小陀螺
 * - GIMBAL_CHASSIS_SPINNING：至少一个底盘电机在线，且底盘已采纳非零自旋命令
 * 状态通过最新值邮箱传递，云台不直接读取底盘或遥控任务的全局变量
 */
typedef enum
{
    GIMBAL_CHASSIS_OFFLINE = 0,
    GIMBAL_CHASSIS_ONLINE,
    GIMBAL_CHASSIS_SPINNING,
} Gimbal_ChassisState_t;

/*
 * 单轴运行状态：
 * - GIMBAL_AXIS_RELAX：待机零输出，反馈和配置有效且命令非 RELAX 时进入运行
 * - GIMBAL_AXIS_RUNNING：闭环运行
 * - GIMBAL_AXIS_FAULT：反馈非法、连续角歧义、零点改变或控制输入异常，零输出锁存至命令回到 RELAX
 * - GIMBAL_AXIS_OFFLINE：电机反馈超时，单轴零输出等待，有效反馈恢复且运行请求有效时自动起控
 * - GIMBAL_AXIS_TRIPPED：仅 yaw 使用，悬空带转保护撤驱，零输出锁存至命令回到 RELAX
 *   不计入锁存故障，遥控保持使能，pitch 继续闭环
 */
typedef enum
{
    GIMBAL_AXIS_RELAX = 0,
    GIMBAL_AXIS_RUNNING,
    GIMBAL_AXIS_FAULT,
    GIMBAL_AXIS_OFFLINE,
    GIMBAL_AXIS_TRIPPED,
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
 * 云台命令：mode 为两轴共用模式，yaw、pitch 为两轴各自目标
 */
typedef struct
{
    Gimbal_Mode_t mode;
    Gimbal_AxisCommand_t yaw;
    Gimbal_AxisCommand_t pitch;
} Gimbal_Command_t;

/*
 * 单轴公开观测：
 * - state：单轴运行状态
 * - encoder_deg：方向修正后的编码器单圈角度，单位度，范围 [0, 360)，失联时为非数值
 * - mechanical_angle_deg：编码器相对零点的机械角，单位度，yaw 连续展开，pitch 为 [-180, 180)
 * - motor_speed_rad_s：编码器换算的角速度，单位 rad/s，用于机械观测及 yaw 机械运动保护
 * - angle_deg：当前 INS 坐标下的姿态角，单位度，yaw 为累计 Yaw，pitch 为 Pitch
 * - speed_rad_s：大地系欧拉角速度，单位 rad/s，由机体系陀螺仪按 ZYX 欧拉运动学换算
 *   yaw 为 (ωy·sinRoll + ωz·cosRoll) / cosPitch，pitch 为 ωy·cosRoll − ωz·sinRoll，姿态未就绪时与角度均为非数值
 * - target_angle_deg：限幅后实际采用的目标角度，单位度
 * - target_speed_rad_s：限幅后实际送入速度环的目标，单位 rad/s
 * - output_current_a：本周期输出电流，单位 A，正方向与角度增大方向一致
 */
typedef struct
{
    Gimbal_AxisState_t state;
    float encoder_deg;
    float mechanical_angle_deg;
    float motor_speed_rad_s;
    float angle_deg;
    float speed_rad_s;
    float target_angle_deg;
    float target_speed_rad_s;
    float output_current_a;
} Gimbal_Axis_t;

/*
 * 云台实例：
 * - config：初始化时复制的配置副本，EmberProbe 可在运行中修改
 * - config_valid：本周期 Control 的快照校验结果，非法时两轴进入故障并由输出闸门归零
 * - mode：上一周期采用的控制模式
 * - yaw、pitch：两轴状态和必要的机械/惯性观测
 * - imu_state：惯性反馈就绪及故障状态
 * 本工程使用单一云台实例，初始化后不得整体复制实例
 */
typedef struct
{
    Gimbal_Config_t config;
    bool config_valid;
    Gimbal_Mode_t mode;
    Gimbal_Axis_t yaw;
    Gimbal_Axis_t pitch;
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
 * @brief 根据命令更新两轴状态机并计算输出电流，yaw 悬空带转时单独撤驱
 *
 * 底盘四个电机持续全部离线时底座转速视为零，此时编码器与惯性转速持续不一致视为悬空带转并撤驱
 * 电机在线且自旋命令已采纳时明确按小陀螺处理；在线未自旋单独标记，两种在线状态均不检测悬空带转
 * 不按编码器相对转速单独撤驱
 * 撤驱只作用于 yaw，pitch 继续闭环，命令回到 RELAX 后解除并在下次起控时捕获当前姿态
 *
 * @param[in,out] gimbal 云台实例
 * @param[in] command 只读云台命令，为 NULL 时按 RELAX 处理，起控及恢复时内部捕获姿态并清零前馈
 * @param[in] chassis_state 底盘任务发布的离线、在线未自旋或在线小陀螺状态
 * @param[in] period_s 实际控制间隔，单位 s，范围 (0, 0.1]，超出时运行中的轴进入故障
 * @return 无返回值
 */
void Gimbal_Control(Gimbal_t *gimbal, const Gimbal_Command_t *command,
                    Gimbal_ChassisState_t chassis_state, float period_s);

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
