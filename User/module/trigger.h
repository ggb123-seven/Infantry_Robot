#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "component/pid.h"
#include "device/motor_rm.h"

// 拨弹状态：待机、速度闭环、反馈离线、故障锁存；故障恢复须先发送失能命令
typedef enum
{
    TRIGGER_RELAX = 0,
    TRIGGER_RUNNING,
    TRIGGER_OFFLINE,
    TRIGGER_FAULT,
} Trigger_State_t;

/*
 * 拨弹配置：max_speed_rpm 为减速后输出轴转速上限，单位 rpm，须为有限正值
 * max_current_a 为转子电流上限，单位 A，范围 (0, 10]
 * speed_pid 为速度 PID，输入输出轴 rpm、输出 A，线性误差 range=0
 * PID 的 k 须为正值，p/i/d/i_limit 须非负，out_limit 范围为 (0, max_current_a]
 * d_cutoff_freq 非正时直通，正值须小于控制频率一半；全部数值须有限
 */
typedef struct
{
    float max_speed_rpm;
    float max_current_a;
    KPID_Params_t speed_pid;
} Trigger_Config_t;

/*
 * 拨弹连续速度命令：enabled 为闭环使能，失能时零电流
 * target_speed_rpm 为减速后输出轴目标转速，单位 rpm，正负方向沿用设备反馈
 * 命令缺失时按失能处理，当前不表达单发、连发或定位事件
 */
typedef struct
{
    bool enabled;
    float target_speed_rpm;
} Trigger_Command_t;

// 拨弹输出：current_a 为 M2006 转子电流，单位 A，零表示撤驱
typedef struct
{
    float current_a;
} Trigger_Output_t;

/*
 * 拨弹实例，初始化后不得整体复制，PID 参数指针引用实例内部副本
 * config 为可调配置，initialized 为初始化结果，config_valid 为本周期配置校验结果
 * frequency_hz 为初始化时控制频率，单位 Hz，applied_speed_pid 为控制器实际引用的参数
 * state 为唯一业务状态，feedback 为最近有效设备反馈，转速为输出轴 rpm
 * feedback_valid 表示最近有效反馈尚未超时，feedback_time_us 为恢复后的完整微秒源时间
 * feedback_time_us 为 UINT64_MAX 时表示尚无反馈；反馈超时上限为 100 ms
 * target_speed_rpm 为限幅后的输出轴目标 rpm，current_a 为候选转子电流 A，非运行时由 DumpOutput 输出零
 * speed_pid 保存速度控制器历史，不由任务或其他模块直接执行
 */
typedef struct
{
    Trigger_Config_t config;
    bool initialized;
    bool config_valid;
    float frequency_hz;
    KPID_Params_t applied_speed_pid;
    Trigger_State_t state;
    MOTOR_RM_Feedback_t feedback;
    bool feedback_valid;
    uint64_t feedback_time_us;
    float target_speed_rpm;
    float current_a;
    KPID_t speed_pid;
} Trigger_t;

/**
 * @brief 复制拨弹配置并初始化速度控制器
 *
 * @param[out] trigger 拨弹实例，初始化后不得整体复制
 * @param[in] config 默认配置，调用方保证非空
 * @param[in] frequency_hz 控制频率，单位 Hz，须为有限正值
 * @return 成功返回 0，失败返回 -1 并保留零输出
 */
int8_t Trigger_Init(Trigger_t *trigger, const Trigger_Config_t *config, float frequency_hz);

/**
 * @brief 接纳拨弹新反馈并检查缓存时效
 *
 * @param[in,out] trigger 拨弹实例
 * @param[in] feedback 本周期新反馈，空指针时仅检查缓存是否超过 100 ms
 * @param[in] now_us 读取反馈后的完整单调时刻，单位微秒
 * @return 无返回值
 */
void Trigger_UpdateFeedback(Trigger_t *trigger, const MOTOR_RM_Feedback_t *feedback, uint64_t now_us);

/**
 * @brief 推进拨弹状态机并执行速度闭环
 *
 * @param[in,out] trigger 拨弹实例
 * @param[in] command 连续速度命令，空指针按失能处理
 * @param[in] period_s 实测控制周期，单位 s，须在 (0, 0.1] 内
 * @return 无返回值
 */
void Trigger_Control(Trigger_t *trigger, const Trigger_Command_t *command, float period_s);

/**
 * @brief 按拨弹运行状态导出本周期电流或零
 *
 * @param[in] trigger 已调用 Init 或 Control 的拨弹实例，须非空
 * @param[out] output 拨弹电流输出，单位 A，须非空
 * @return 无返回值
 */
void Trigger_DumpOutput(const Trigger_t *trigger, Trigger_Output_t *output);

#ifdef __cplusplus
}
#endif
