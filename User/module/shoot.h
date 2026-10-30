#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "component/pid.h"
#include "device/motor_rm.h"

// 两路摩擦轮按索引 0、1 对应 CAN2 上 M3508 的 ID5、ID6
#define SHOOT_MOTOR_COUNT (2U)

// 双轮状态：待机、速度闭环、任一路反馈离线、故障锁存；故障恢复须先失能
typedef enum
{
    SHOOT_RELAX = 0,
    SHOOT_RUNNING,
    SHOOT_OFFLINE,
    SHOOT_FAULT,
} Shoot_State_t;

/*
 * 摩擦轮配置：max_speed_rpm 为转子目标转速绝对值上限，单位 rpm，须为有限正值
 * max_current_a 为每轮转子电流上限，单位 A，范围 (0, 20]
 * speed_pid 为两轮共用的速度环参数，输入转子 rpm、输出 A，线性误差 range=0
 * PID 的 k 须为正值，p/i/d/i_limit 须非负，out_limit 范围为 (0, max_current_a]
 * d_cutoff_freq 非正时直通，正值须小于控制频率一半；全部数值须有限
 */
typedef struct
{
    float max_speed_rpm;
    float max_current_a;
    KPID_Params_t speed_pid;
} Shoot_Config_t;

/*
 * 摩擦轮连续速度命令：enabled 为两轮共同使能，失能时两轮零电流
 * target_speed_rpm[0~1] 为 ID5、ID6 的转子目标 rpm，方向独立指定，不隐含正反转
 * 命令缺失时按失能处理，当前不承担弹速、热量或单发连发策略
 */
typedef struct
{
    bool enabled;
    float target_speed_rpm[SHOOT_MOTOR_COUNT];
} Shoot_Command_t;

// 摩擦轮输出：current_a[0~1] 为 ID5、ID6 的转子电流，单位 A，零表示撤驱
typedef struct
{
    float current_a[SHOOT_MOTOR_COUNT];
} Shoot_Output_t;

/*
 * 双摩擦轮实例，初始化后不得整体复制，两个 PID 均引用实例内参数副本
 * config 为可调配置，initialized 为初始化结果，config_valid 为本周期配置校验结果
 * frequency_hz 为初始化时控制频率，单位 Hz，applied_speed_pid 为两轮实际采用的公共参数
 * state 为唯一业务状态，任一路反馈离线时两轮一起撤驱
 * feedback[0~1] 为最近有效反馈，速度为转子 rpm，feedback_valid 为各轮反馈时效标志
 * feedback_time_us[0~1] 为完整微秒源时间，UINT64_MAX 表示尚无反馈，超时上限为 100 ms
 * target_speed_rpm[0~1] 为限幅后的转子目标 rpm，current_a[0~1] 为最终转子电流 A
 * speed_pid[0~1] 保存两个独立速度控制器的积分、微分与滤波历史
 */
typedef struct
{
    Shoot_Config_t config;
    bool initialized;
    bool config_valid;
    float frequency_hz;
    KPID_Params_t applied_speed_pid;
    Shoot_State_t state;
    MOTOR_RM_Feedback_t feedback[SHOOT_MOTOR_COUNT];
    bool feedback_valid[SHOOT_MOTOR_COUNT];
    uint64_t feedback_time_us[SHOOT_MOTOR_COUNT];
    float target_speed_rpm[SHOOT_MOTOR_COUNT];
    float current_a[SHOOT_MOTOR_COUNT];
    KPID_t speed_pid[SHOOT_MOTOR_COUNT];
} Shoot_t;

/**
 * @brief 复制摩擦轮配置并初始化两个速度控制器
 *
 * @param[out] shoot 双摩擦轮实例，初始化后不得整体复制
 * @param[in] config 默认配置，空指针时初始化失败
 * @param[in] frequency_hz 控制频率，单位 Hz，须为有限正值
 * @return 成功返回 0，失败返回 -1 并保留两轮零输出
 */
int8_t Shoot_Init(Shoot_t *shoot, const Shoot_Config_t *config, float frequency_hz);

/**
 * @brief 独立接纳两轮新反馈并检查各自的缓存时效
 *
 * @param[in,out] shoot 双摩擦轮实例
 * @param[in] fric1 第一轮新反馈，空指针时仅检查该轮缓存是否超过 100 ms
 * @param[in] fric2 第二轮新反馈，空指针时仅检查该轮缓存是否超过 100 ms
 * @param[in] now_us 读取两路反馈后的完整单调时刻，单位微秒
 * @return 无返回值
 */
void Shoot_UpdateFeedback(Shoot_t *shoot, const MOTOR_RM_Feedback_t *fric1,
                          const MOTOR_RM_Feedback_t *fric2, uint64_t now_us);

/**
 * @brief 推进双轮状态机并执行两个独立的速度闭环
 *
 * @param[in,out] shoot 双摩擦轮实例
 * @param[in] command 双轮连续速度命令，空指针按失能处理
 * @param[in] period_s 实测控制周期，单位 s，须在 (0, 0.1] 内
 * @return 无返回值
 */
void Shoot_Control(Shoot_t *shoot, const Shoot_Command_t *command, float period_s);

/**
 * @brief 导出两轮电流，未初始化或非运行状态时两轮输出零
 *
 * @param[in] shoot 双摩擦轮实例，允许为空
 * @param[out] output 两轮电流，单位 A，允许为空
 * @return 无返回值
 */
void Shoot_DumpOutput(const Shoot_t *shoot, Shoot_Output_t *output);

#ifdef __cplusplus
}
#endif
