#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

#include "component/angle_unwrap.h"
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
 * 单轴运行状态：
 * - GIMBAL_AXIS_RELAX：待机零输出，反馈和配置有效且命令非 RELAX 时进入运行
 * - GIMBAL_AXIS_RUNNING：闭环运行
 * - GIMBAL_AXIS_FAULT：运行中反馈超时、连续角断开、零点改变或输入非法，零输出锁存至命令回到 RELAX
 */
typedef enum
{
    GIMBAL_AXIS_RELAX = 0,
    GIMBAL_AXIS_RUNNING,
    GIMBAL_AXIS_FAULT,
} Gimbal_AxisState_t;

/*
 * 单轴配置，运行中可由调试器修改，模块每周期校验并采纳：
 * - enabled：false 时该轴始终零输出
 * - reversed：true 时反转编码器角度、转速和输出电流方向
 * - zero_encoder_deg：yaw 正前方或 pitch 水平对应的方向修正后编码器角度，单位度，范围 [0, 360)
 *   修改零点后运行中的角度闭环进入故障
 * - min_angle_deg、max_angle_deg：相对零点的目标角度范围，单位度，目标超出时限幅
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
 */
typedef struct
{
    Gimbal_AxisConfig_t yaw;
    Gimbal_AxisConfig_t pitch;
    float feedback_timeout_s;
} Gimbal_Config_t;

/*
 * 单轴命令：
 * - target_angle_deg：相对零点的目标角度，单位度，仅 ANGLE 模式使用
 * - target_speed_rad_s：SPEED 模式下为速度目标，ANGLE 模式下为角速度前馈，单位 rad/s
 */
typedef struct
{
    float target_angle_deg;
    float target_speed_rad_s;
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
 * 单轴运行数据，调试器可直接观察：
 * - state：单轴运行状态
 * - encoder_deg：方向修正后的编码器单圈角度，单位度，范围 [0, 360)，失联时为非数值
 * - angle_deg：相对零点的角度，单位度，yaw 为连续角且首帧落在 [0, 360)，pitch 为 [-180, 180)
 *   pitch 首帧和失联恢复均直接确定正负角，失联或角度无效时为非数值
 * - speed_rad_s：编码器换算角速度，单位 rad/s，失联时为非数值
 * - target_angle_deg：限幅后实际采用的目标角度，单位度
 * - target_speed_rad_s：限幅后实际送入速度环的目标，单位 rad/s
 * - output_current_a：本周期输出电流，单位 A，正方向与角度增大方向一致
 * - feedback_time_us：最近反馈帧时刻，单位微秒，无反馈时为 UINT64_MAX
 * - applied_zero_encoder_deg：当前连续角坐标采用的零点，用于识别零点修改
 * - applied_angle_pid、applied_speed_pid：PID 实际使用的参数副本，与配置不同时同步并清空积分
 * - unwrap：yaw 连续角展开器，pitch 不使用
 * - angle_pid、speed_pid：角度外环和速度内环运行状态
 */
typedef struct
{
    Gimbal_AxisState_t state;
    float encoder_deg;
    float angle_deg;
    float speed_rad_s;
    float target_angle_deg;
    float target_speed_rad_s;
    float output_current_a;
    uint64_t feedback_time_us;
    float applied_zero_encoder_deg;
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
 * 实例内 PID 持有指向自身参数副本的指针，初始化后不得整体复制实例
 */
typedef struct
{
    Gimbal_Config_t config;
    Gimbal_Mode_t mode;
    Gimbal_Axis_t yaw;
    Gimbal_Axis_t pitch;
    Gimbal_ImuFeedback_t imu;
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
 * @brief 复制配置并初始化两轴控制器，初始化后两轴处于 RELAX
 *
 * @param[out] gimbal 云台实例
 * @param[in] config 云台配置，内容被复制到实例
 * @param[in] frequency_hz 控制频率，单位 Hz，范围 10 至 2000
 * @return 成功返回 0，参数非法或 PID 初始化失败返回 -1
 */
int8_t Gimbal_Init(Gimbal_t *gimbal, const Gimbal_Config_t *config, float frequency_hz);

/**
 * @brief 接收两轴最新反馈帧，更新角度、转速和反馈时效
 *
 * @param[in,out] gimbal 云台实例
 * @param[in] yaw_motor yaw 新反馈帧，本周期没有新帧时传 NULL
 * @param[in] pitch_motor pitch 新反馈帧，本周期没有新帧时传 NULL
 * @param[in] now_us 当前单调时刻，单位微秒
 * @return 无返回值
 */
void Gimbal_UpdateFeedback(Gimbal_t *gimbal, const MOTOR_RM_Feedback_t *yaw_motor,
                           const MOTOR_RM_Feedback_t *pitch_motor, uint64_t now_us);

/**
 * @brief 根据命令更新两轴状态机并计算输出电流
 *
 * @param[in,out] gimbal 云台实例
 * @param[in] command 云台命令，为 NULL 时按 RELAX 处理
 * @param[in] period_s 实际控制间隔，单位 s，范围 (0, 0.1]，超出时运行中的轴进入故障
 * @return 无返回值
 */
void Gimbal_Control(Gimbal_t *gimbal, const Gimbal_Command_t *command, float period_s);

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
