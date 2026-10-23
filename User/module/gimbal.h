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
 * target_deg：以机械正前方为 0° 的线性目标，单位度，范围 [0, 360]，0°与 360° 作为不同目标
 * actual_deg：相对机械正前方的连续位置，单位度，启动时在 [0, 360)，跨编码器零点后保持连续
 * 未标定、反馈无效或连续位置丢失时 actual_deg 为非数值
 * enabled：串级控制使能，标定完成后手动置 true；零点改变或失效时自动置 false
 * 角度环输出与目标角速度前馈相加并限幅后写入 gimbal_yaw_speed.target_rad_s，单位 rad/s
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
 * 正弦跟随测试使用独立调试变量，沿用现有结构体布局
 * gimbal_yaw_sine_test_enabled：默认 false，true 时以 180° 为中心、30° 为振幅、6 rad/s 为相位角速度
 * 测试只在串级控制有效时推进，停止或反馈异常后相位归零，重新使能时从 180° 开始
 * gimbal_yaw_sine_feedforward_enabled：默认 true，仅控制正弦测试的前馈，可在同一轨迹中切换比较
 * gimbal_yaw_feedforward_rad_s：只读本周期采用的前馈，单位 rad/s，合成限幅前的值，停机时为零
 * 正弦测试覆盖 target_deg 和接口前馈输入，关闭测试后保持最后目标并恢复接口前馈输入
 * 参数由同一云台任务消费，volatile 仅保证调试可见性，不提供互斥或多字段原子快照
 */
extern volatile bool gimbal_yaw_sine_test_enabled;
extern volatile bool gimbal_yaw_sine_feedforward_enabled;
extern volatile float gimbal_yaw_feedforward_rad_s;

/*
 * yaw 目标电流双向限幅为 0.5 A，低于手册所列 1.62 A 额定电流
 * 目标速度限幅依据 GM6020 使用说明 20231013 的 320 rpm 空载转速，以 2π/60 转为 rad/s
 * 反馈超过 100 ms 时禁止速度环计算；角度位置样本超过 90 ms 时清除零点并要求重新标定
 * 控制周期最大 100 ms，均为当前模块的软件保护配置
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
 * @param[in] yaw_rate_feedback_rad_s 外部偏航角速度，单位 rad/s，正方向须与角度增大方向一致
 * @param[in] yaw_rate_feedback_valid 外部角速度有效且未过期时为 true，角速度须为有限值；false 时使用编码器换算值
 * @param[in] yaw_rate_feedforward_rad_s 目标角速度前馈，单位 rad/s，须为有限值，零表示关闭，正方向与目标角度一致
 * @return 无返回值，须先初始化控制器并绑定邮箱，由同一任务周期调用
 */
void Gimbal_Update(float period_s, float yaw_rate_feedback_rad_s, bool yaw_rate_feedback_valid,
                   float yaw_rate_feedforward_rad_s);

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
 * @brief 将编码器角度展开为机械相对位置，执行标定及角度与速度串级控制
 *
 * @param[in] feedback_deg 原始编码器单圈角度，单位度，范围 [0, 360)，不得预先减去机械零点
 * @param[in] angle_valid 电机反馈中的角度跟踪有效标志，false 时清除控制使能
 * @param[in] feedback_rpm 编码器转子转速，单位 rpm，无有效反馈时传入非数值
 * @param[in] yaw_rate_feedback_rad_s 外部偏航角速度，单位 rad/s，正方向须与角度增大方向一致
 * @param[in] yaw_rate_feedback_valid 外部角速度有效且未过期时为 true，角速度须为有限值；false 时由反馈转速换算
 * @param[in] feedback_time_us 最近反馈的完整微秒时刻，无反馈时传入 UINT64_MAX
 * @param[in] now_us 当前完整单调时刻，单位微秒
 * @param[in] enabled 本次调用是否允许控制，与全局角度环使能共同决定运行状态
 * @param[in] period_s 实际控制间隔，单位秒，须在初始化周期至 0.1 秒之间
 * @param[in] yaw_rate_feedforward_rad_s 目标角速度前馈，单位 rad/s，须为有限值，零表示关闭，正方向与目标角度一致
 * @return 转子侧目标电流，单位 A，未标定、禁用或输入无效时为零，并清空两环历史
 */
float Gimbal_YawRunAt(float feedback_deg, bool angle_valid, float feedback_rpm, float yaw_rate_feedback_rad_s,
                     bool yaw_rate_feedback_valid, uint64_t feedback_time_us, uint64_t now_us, bool enabled,
                     float period_s, float yaw_rate_feedforward_rad_s);

/**
 * @brief 独立初始化 GM6020 偏航角度外环，初始化后保持禁用，由调用方接入速度环
 *
 * @param[in] sample_frequency_hz 控制频率，单位 Hz，须为有限正数
 * @param[in] params 持续有效的现有 PID 参数对象，调用期间不得并发修改
 * k 通常取 1，p、i、d 分别为比例、积分和反馈角速度增益，单位依次为 1/s、1/s² 和无量纲
 * i_limit 为积分状态绝对值上限，单位 rad·s，out_limit 为输出速度限幅，单位 rad/s
 * range 为角度环绕周期，单圈角度取 2π，连续线性位置取 0，环绕时目标与反馈须在同一周期区间
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
 * @param[in] target_deg 目标角度，单位度；range 为 0 时不环绕，为正数时按周期环绕
 * @param[in] feedback_deg 实际角度，单位度，与目标采用相同零点和方向；连续位置须传入未折返角度
 * @param[in] feedback_rad_s 实际角速度，单位 rad/s，正方向须与角度增大方向一致，可使用 INS 陀螺仪读数
 * @param[in] period_s 控制间隔，单位 s，须为有限正数且不小于初始化周期
 * @return 速度目标，单位 rad/s，未初始化、禁用或输入无效时返回零，不执行机械零点换算
 */
float Gimbal_YawAngleRun(float target_deg, float feedback_deg, float feedback_rad_s, float period_s);

#ifdef __cplusplus
}
#endif
