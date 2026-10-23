#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <cmsis_os2.h>

#include "component/pid.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*
 * yaw 速度环调试参数与反馈集中区，初值见模块定义
 * kp：比例增益
 * ki：积分增益
 * kd：反馈微分增益
 * target_rad_s：速度目标
 * actual_rad_s：实际角速度
 */
typedef struct gimbal_yaw_speed
{
    float kp;
    float ki;
    float kd;
    float target_rad_s;
    float actual_rad_s;
} GimbalYawSpeed_t;

extern volatile GimbalYawSpeed_t gimbal_yaw_speed;

/*
 * yaw 角度环调试参数与反馈集中区，默认禁用，三个增益及目标初值均为零
 * kp：比例增益，单位 1/s；ki：积分增益，单位 1/s²；kd：反馈角速度增益，无量纲
 * 三个增益由调用方保证为有限非负数，零增益初值供后续调参
 * target_deg：相对机械正前方的目标角度，单位度，有限值按 360 度环绕到 [0, 360)
 * actual_deg：相对机械正前方的实际角度，单位度，范围 [0, 360)，未标定或反馈无效时为非数值
 * enabled：串级控制使能，标定完成后手动置 true；零点改变或失效时自动置 false
 * 角度环输出写入 gimbal_yaw_speed.target_rad_s，单位 rad/s，不重复保存输出字段
 * 调试器可修改增益、目标和使能，实际角度只读；同一任务串行调用，volatile 不提供互斥
 */
typedef struct gimbal_yaw_angle
{
    float kp;
    float ki;
    float kd;
    float target_deg;
    float actual_deg;
    bool enabled;
} GimbalYawAngle_t;

extern volatile GimbalYawAngle_t gimbal_yaw_angle;

/*
 * yaw 机械零点标定独立变量，沿用现有调试结构体，不增加字段
 * gimbal_yaw_zero_deg：正前方对应的编码器角度，单位度，须在 [0, 360)，初值为非数值
 * 可手动填入已记录的固定零点；非数值或越界值表示未标定，零点改变时停用两环并清空历史
 * gimbal_yaw_zero_capture：停用串级并等待零输出后，对准正前方写 true，模块单次采样后自动清除
 * 只有反馈有效且两环均不在运行时才接受采样，拒绝的请求也会清除，不延迟到后续周期执行
 * gimbal_yaw_encoder_deg：只读编码器单圈角度，单位度，范围 [0, 360)，反馈无效时为非数值
 * 标定不驱动电机寻找位置，不写入闪存；记录零点数值，重启后需重新填入或人工标定
 * 模块由同一任务串行调用，调试器修改时先停用控制；volatile 不保证多个字段的一致快照
 */
extern volatile float gimbal_yaw_zero_deg;
extern volatile bool gimbal_yaw_zero_capture;
extern volatile float gimbal_yaw_encoder_deg;

/*
 * yaw 目标电流双向限幅为 0.5 A，低于手册所列 1.62 A 额定电流
 * 目标速度限幅依据 GM6020 使用说明 20231013 的 320 rpm 空载转速，以 2π/60 转为 rad/s
 * 反馈超过 100 ms 时禁止计算，控制周期最大 100 ms，均为当前模块的软件保护配置
 */
#define GIMBAL_YAW_CURRENT_LIMIT_A 0.5f
#define GIMBAL_YAW_SPEED_LIMIT_RAD_S (320.0f * 0.10471975512f)
#define GIMBAL_YAW_FEEDBACK_TIMEOUT_MS 100U
#define GIMBAL_YAW_MAX_PERIOD_S 0.1f

/**
 * @brief 初始化 yaw 角度与速度控制器，保留参数和零点配置，强制禁用并丢弃采样请求
 *
 * @param[in] sample_frequency_hz 控制频率，单位 Hz，范围为 10 至 2000
 * @return 成功返回 0，频率非法或控制器初始化失败返回 -1
 */
int8_t Gimbal_Init(float sample_frequency_hz);

/**
 * @brief 绑定已创建的云台反馈与电流邮箱，并使旧反馈失效
 *
 * @param[in] feedback_queue 容量为 1 的 MOTOR_RM_Feedback_t 邮箱，句柄须有效
 * @param[in] current_queue 容量为 1 的 float 安培值邮箱，句柄须有效
 * @return 无返回值，邮箱由系统初始化入口创建和销毁，模块只使用句柄
 */
void Gimbal_BindQueues(osMessageQueueId_t feedback_queue, osMessageQueueId_t current_queue);

/**
 * @brief 读取编码器反馈、执行串级控制并发布最新电流命令
 *
 * @param[in] period_s 实际控制间隔，单位秒，由任务周期调度提供
 * @return 无返回值，须先初始化控制器并绑定邮箱，由同一任务周期调用
 */
void Gimbal_Update(float period_s);

/**
 * @brief 根据 yaw 转子反馈计算单周期电流命令，不直接访问总线
 *
 * @param[in] feedback_rpm 驱动原始转子速度，单位 rpm，有效范围为有符号 16 位反馈范围
 * @param[in] feedback_age_ms 距最近有效帧的时间，单位 ms，尚无反馈时传入 UINT32_MAX
 * @param[in] enabled 是否允许速度控制，false 时清空控制器并输出零电流
 * @param[in] period_s 控制间隔，单位 s，须不小于初始化周期且不超过 0.1 s
 * @return 转子侧目标电流，单位 A，范围为正负 0.5 A，未初始化或输入无效时返回 0
 */
float Gimbal_YawSpeedRun(float feedback_rpm, uint32_t feedback_age_ms, bool enabled, float period_s);

/**
 * @brief 根据完整微秒时间戳检查反馈时效并执行速度闭环
 *
 * @param[in] feedback_rpm 驱动反馈转速，单位 rpm，无有效反馈时传入非数值
 * @param[in] feedback_time_us 最近反馈时刻，单位微秒，未收到反馈时传入 UINT64_MAX
 * @param[in] now_us 当前单调时间，单位微秒
 * @param[in] enabled 是否允许控制，false 时清空历史并输出零电流
 * @param[in] period_s 实际控制间隔，单位秒，须在初始化周期至 0.1 秒之间
 * @return 转子侧目标电流，单位 A，范围为正负 0.5 A，反馈过期或时间异常时返回零
 */
float Gimbal_YawSpeedRunAt(float feedback_rpm, uint64_t feedback_time_us, uint64_t now_us,
                         bool enabled, float period_s);

/**
 * @brief 将编码器角度转换为机械相对角，执行标定及角度与速度串级控制
 *
 * @param[in] feedback_deg 原始编码器单圈角度，单位度，范围 [0, 360)，不得预先减去机械零点
 * @param[in] feedback_rpm 编码器转子转速，单位 rpm，无有效反馈时传入非数值
 * @param[in] feedback_time_us 最近反馈的完整微秒时刻，无反馈时传入 UINT64_MAX
 * @param[in] now_us 当前完整单调时刻，单位微秒
 * @param[in] enabled 本次调用是否允许控制，与全局角度环使能共同决定运行状态
 * @param[in] period_s 实际控制间隔，单位秒，须在初始化周期至 0.1 秒之间
 * @return 转子侧目标电流，单位 A，未标定、禁用或输入无效时为零，并清空两环历史
 */
float Gimbal_YawRunAt(float feedback_deg, float feedback_rpm, uint64_t feedback_time_us,
                     uint64_t now_us, bool enabled, float period_s);

/**
 * @brief 独立初始化 GM6020 偏航角度外环，初始化后保持禁用，由调用方接入速度环
 *
 * @param[in] sample_frequency_hz 控制频率，单位 Hz，须为有限正数
 * @param[in] params 持续有效的现有 PID 参数对象，调用期间不得并发修改
 * k 通常取 1，p、i、d 分别为比例、积分和反馈角速度增益，单位依次为 1/s、1/s² 和无量纲
 * i_limit 为积分状态绝对值上限，单位 rad·s，out_limit 为输出速度限幅，单位 rad/s
 * range 为角度环绕周期，单圈反馈取 2π，多圈连续角度取 0，环绕时目标与反馈须在同一周期区间
 * d_cutoff_freq 不参与外部角速度微分，所有参数和反馈的有效性由调用方保证
 * @return 成功返回 0，底层 PID 初始化失败返回 -1
 */
int8_t Gimbal_YawAngleInit(float sample_frequency_hz, const KPID_Params_t *params);

/**
 * @brief 设置偏航角度外环使能，切换时清空控制历史，不操作速度环和电机输出
 *
 * @param[in] enabled true 为使能，false 为禁用，初始化前调用不生效
 * @return 无返回值
 */
void Gimbal_YawAngleSetEnabled(bool enabled);

/**
 * @brief 计算偏航角度外环速度目标，返回值由调用方自行接入速度环
 *
 * @param[in] target_deg 目标角度，单位度，单圈模式下有限值按 360 度环绕，与反馈采用相同零点和方向
 * @param[in] feedback_deg 实际角度，单位度，单圈模式下按 360 度环绕，串级调用传入机械相对角
 * @param[in] feedback_rad_s 实际角速度，单位 rad/s，当前可由编码器转速换算，后续可使用 INS 陀螺仪读数
 * @param[in] period_s 控制间隔，单位 s，须为有限正数且不小于初始化周期
 * @return 速度目标，单位 rad/s，未初始化、禁用或输入无效时返回零，不执行机械零点换算
 */
float Gimbal_YawAngleRun(float target_deg, float feedback_deg, float feedback_rad_s, float period_s);

#ifdef __cplusplus
}
#endif
