#pragma once

#include "component/pid.h"
#include "device/dr16.h"
#include "device/motor_rm.h"

#ifdef __cplusplus
extern "C"
{
#endif

// 待机撤驱，运行速度调试，离线等待新反馈，异常等待停机档解除
typedef enum
{
    FEEDER_RELAX = 0,
    FEEDER_RUNNING,
    FEEDER_OFFLINE,
    FEEDER_FAULT,
} Feeder_State_t;

/*
 * 拨弹可调配置，初始化复制到实例，运行中通过 EmberProbe 修改实例副本
 * 当前旁路位置外环，tooth_count、feed_hz、direction 和 position_pid 保留但不参与速度目标生成
 * tooth_count 为一圈齿数，须为有限值且至少为 2；feed_hz 为每秒进给齿数，须大于零且不超过采样频率
 * direction 为角度递增方向，只接受 -1 或 +1；默认 -1 沿用参考工程正常拨弹方向
 * max_speed_rpm 为输出轴速度目标上限，单位 rpm；max_current_a 为转子电流上限，单位 A，范围 (0,10]
 * position_pid 输入输出轴弧度，输出 rad/s，range 固定为 2π，输出限幅不得超过速度上限换算值
 * speed_pid 输入输出轴 rpm，输出 A，range 固定为零，输出限幅不得超过电流上限
 * speed_feedforward_gain_a_per_rpm 为目标逐周期差分的电流前馈系数，单位 A/rpm，非负，零表示关闭
 * 两环的 k 须为正，p/i/d/i_limit 非负，out_limit 大于 0.000001，微分截止频率小于采样频率一半
 */
typedef struct
{
    float tooth_count;
    float feed_hz;
    float direction;
    float max_speed_rpm;
    float max_current_a;
    KPID_Params_t position_pid;
    KPID_Params_t speed_pid;
    float speed_feedforward_gain_a_per_rpm;
} Feeder_Config_t;

/*
 * 拨弹实例由拨弹任务独占，初始化后不得整体复制
 * config 为可调副本，cycle_config 为本周期持久快照，PID 长期引用快照内参数
 * initialized 与 config_valid 分别表示初始化结果与本周期配置结果
 * state 为唯一运行状态，feedback 为输出轴弧度和 rpm，feedback_valid 表示最近完整帧有效且未超过 10 ms
 * target_angle_rad 为环绕位置目标，phase_s 为距下一齿的累计秒数，output_current_a 为转子侧安培
 * 速度调试期间 target_angle_rad 只在起控时捕获实际位置，phase_s 保持为零
 * speed_rad_s 为最近有效反馈的输出轴实测角速度，单位 rad/s
 * speed_pid 集中保存速度请求、限幅后目标、实际转速和速度控制器，字段说明见成员定义上方
 * measured_angle_deg、target_angle_deg 为只读角度镜像，单位度，不参与位置控制
 * measured_speed_deg_s、target_speed_deg_s 为只读角速度镜像，单位度每秒，速度环调参仍使用 rpm
 * position_pid 保存外环历史，frequency_hz 为初始化采样频率
 */
typedef struct
{
    Feeder_Config_t config;
    Feeder_Config_t cycle_config;
    bool initialized;
    bool config_valid;
    Feeder_State_t state;
    MOTOR_RM_Feedback_t feedback;
    bool feedback_valid;
    float target_angle_rad;
    float speed_rad_s;
    float measured_angle_deg;
    float target_angle_deg;
    float measured_speed_deg_s;
    float target_speed_deg_s;
    double phase_s;
    float output_current_a;
    KPID_t position_pid;
    /*
     * 速度环运行数据，三个转速字段均为减速后输出轴 rpm
     * requested_speed_rpm 为可写请求，上电为零，每周期控制入口只读取一次
     * target_speed_rpm 为限幅后目标，状态切换或改参时重置，同时保存前馈差分历史
     * measured_speed_rpm 为实测转速，反馈无效时为 NaN
     * controller 保存速度 PID 历史，参数指针长期引用实例内 cycle_config.speed_pid
     */
    struct
    {
        volatile float requested_speed_rpm;
        float target_speed_rpm;
        float measured_speed_rpm;
        KPID_t controller;
    } speed_pid;
    float frequency_hz;
} Feeder_t;

/**
 * @brief 复制默认拨弹配置并初始化位置外环和速度内环
 *
 * @param[out] feeder 拨弹实例，调用方保证非空
 * @param[in] frequency_hz 调度频率，单位 Hz，须为有限正值
 * @return 成功返回 0，失败返回 -1 并保持安全状态
 */
int8_t Feeder_Init(Feeder_t *feeder, float frequency_hz);

/**
 * @brief 在输入边界接收新反馈并按源时间检查缓存帧龄
 *
 * @param[in,out] feeder 拨弹实例，调用方保证非空
 * @param[in] feedback 驱动反馈，调用方保证非空，转速为输出轴 rpm，角度为 rad
 * @param[in] received 驱动取得完整新反馈时为 true
 * @param[in] now_us 读取反馈后的完整单调微秒时间
 * @return 无返回值
 */
void Feeder_UpdateFeedback(Feeder_t *feeder, const MOTOR_RM_Feedback_t *feedback, bool received, uint64_t now_us);

/**
 * @brief 读取实例速度请求，校验周期配置与遥控源时间并执行速度控制
 *
 * @param[in,out] feeder 拨弹实例，调用方保证非空
 * @param[in] remote 最近遥控快照，调用方保证非空
 * @param[in] remote_valid 已收到遥控且邮箱读取无异常时为 true
 * @param[in] now_us 读取遥控后的完整单调微秒时间
 * @param[in] period_s 实测控制周期，单位秒，合法范围 (0,0.1]
 * @return 无返回值
 */
void Feeder_Control(Feeder_t *feeder, const DR16_State_t *remote, bool remote_valid, uint64_t now_us, float period_s);

/**
 * @brief 根据唯一运行状态导出电流，其他状态统一输出零
 *
 * @param[in] feeder 拨弹实例，调用方保证非空
 * @return 转子侧目标电流，单位 A
 */
float Feeder_DumpOutput(const Feeder_t *feeder);

#ifdef __cplusplus
}
#endif
