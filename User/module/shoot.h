/*
 * 摩擦轮射击模块对外接口
 *
 * 两路摩擦轮按索引对应 CAN2 上 M3508 的 ID5、ID6，输入连续速度命令，输出转子电流
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ----------------------------------------------------------------- */
#include "component/pid.h"
#include "device/motor_rm.h"

/* Exported constants ------------------------------------------------------- */
/* 两路摩擦轮按索引 0、1 对应 CAN2 上 M3508 的 ID5、ID6 */
#define SHOOT_MOTOR_COUNT (2U) /* 摩擦轮数量 */

/* 双轮共用的一阶目标差分前馈系数，写入 0 关闭前馈 */
extern volatile float shoot_speed_feedforward_gain_a_per_rpm; /* 前馈系数，单位：A/rpm */

/* Exported types ----------------------------------------------------------- */
/* 双轮状态，故障恢复须先失能 */
typedef enum
{
    SHOOT_RELAX = 0, /* 待机，两轮零电流 */
    SHOOT_RUNNING,   /* 恒速闭环 */
    SHOOT_OFFLINE,   /* 任一路反馈离线，两轮撤驱 */
    SHOOT_FAULT,     /* 故障锁存，须先失能解除 */
    SHOOT_RAMPING,   /* 一秒目标过渡 */
} Shoot_State_t;

/* 摩擦轮配置 */
typedef struct
{
    float max_speed_rpm;     /* 转子目标转速上限，单位：rpm */
    float max_current_a;     /* 每轮转子电流上限，单位：A */
    KPID_Params_t speed_pid; /* 两轮共用的速度环参数 */
} Shoot_Config_t;

/* 摩擦轮连续速度命令，命令缺失时按失能处理 */
typedef struct
{
    bool enabled;                              /* 两轮共同使能 */
    float target_speed_rpm[SHOOT_MOTOR_COUNT]; /* 各轮转子目标，方向独立指定，单位：rpm */
} Shoot_Command_t;

/* 摩擦轮输出 */
typedef struct
{
    float current_a[SHOOT_MOTOR_COUNT]; /* 各轮转子电流，单位：A */
} Shoot_Output_t;

/* 双摩擦轮实例，初始化后不得整体复制，两个 PID 均引用实例内参数副本 */
typedef struct
{
    Shoot_Config_t config;                           /* 可调配置副本 */
    bool initialized;                                /* 初始化结果 */
    bool config_valid;                               /* 本周期配置校验结果 */
    float frequency_hz;                              /* 初始化控制频率，单位：Hz */
    KPID_Params_t applied_speed_pid;                 /* 两轮实际采用的公共参数 */
    Shoot_State_t state;                             /* 唯一业务状态 */
    MOTOR_RM_Feedback_t feedback[SHOOT_MOTOR_COUNT]; /* 各轮最近有效反馈 */
    bool feedback_valid[SHOOT_MOTOR_COUNT];          /* 各轮反馈时效标志 */
    uint64_t feedback_time_us[SHOOT_MOTOR_COUNT];    /* 各轮源时间，单位：us，UINT64_MAX 为无反馈 */
    float target_speed_rpm[SHOOT_MOTOR_COUNT];       /* 斜坡后的转子目标，单位：rpm */
    float current_a[SHOOT_MOTOR_COUNT];              /* 叠加前馈后的候选电流，单位：A */
    KPID_t speed_pid[SHOOT_MOTOR_COUNT];             /* 两个独立速度环历史 */
    float ramp_target_rpm[SHOOT_MOTOR_COUNT];        /* 各轮限幅后的斜坡终点，单位：rpm */
    float ramp_rate_rpm_s[SHOOT_MOTOR_COUNT];        /* 各轮本次过渡变化率，单位：rpm/s */
    uint32_t command_miss_count;                     /* 读命令落空的累计周期数，仅供诊断 */
    uint32_t active_drop_count;                      /* 运行被打断的累计次数，仅供诊断 */
} Shoot_t;

/* Exported functions ------------------------------------------------------- */
/**
 * @brief 复制摩擦轮配置并初始化两个速度控制器
 *
 * @param shoot 双摩擦轮实例，初始化后不得整体复制
 * @param config 默认配置
 * @param frequency_hz 控制频率，单位：Hz
 * @return int8_t 成功返回 0，失败返回 -1 并保留两轮零输出
 */
int8_t Shoot_Init(Shoot_t *shoot, const Shoot_Config_t *config, float frequency_hz);

/**
 * @brief 独立接纳两轮新反馈并检查各自的缓存时效
 *
 * @param shoot 双摩擦轮实例
 * @param fric1 第一轮新反馈，空指针时仅检查该轮缓存是否超时
 * @param fric2 第二轮新反馈，空指针时仅检查该轮缓存是否超时
 * @param now_us 读取两路反馈后的完整单调时刻，单位：us
 * @return 无返回值
 */
void Shoot_UpdateFeedback(Shoot_t *shoot, const MOTOR_RM_Feedback_t *fric1,
                          const MOTOR_RM_Feedback_t *fric2, uint64_t now_us);

/**
 * @brief 推进双轮状态机，经一秒目标斜坡执行两个独立速度闭环
 *
 * @param shoot 双摩擦轮实例
 * @param command 双轮连续速度命令，失能或空指针立即撤驱
 * @param period_s 实测控制周期，单位：s
 * @return 无返回值
 */
void Shoot_Control(Shoot_t *shoot, const Shoot_Command_t *command, float period_s);

/**
 * @brief 按双摩擦轮运行状态导出本周期电流或零
 *
 * @param shoot 已初始化或已运行的双摩擦轮实例
 * @param output 两轮电流，单位：A
 * @return 无返回值
 */
void Shoot_DumpOutput(const Shoot_t *shoot, Shoot_Output_t *output);

#ifdef __cplusplus
}
#endif
