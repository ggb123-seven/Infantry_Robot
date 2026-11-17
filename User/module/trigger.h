/*
 * 拨弹速度模块对外接口
 *
 * 单路 M2006 拨弹按连续速度命令闭环输出转子电流，故障恢复须先失能
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ----------------------------------------------------------------- */
#include "component/pid.h"
#include "device/motor_rm.h"

/* Exported types ----------------------------------------------------------- */
/* 拨弹状态，故障恢复须先发送失能命令 */
typedef enum
{
    TRIGGER_RELAX = 0, /* 待机，零电流 */
    TRIGGER_RUNNING,   /* 速度闭环 */
    TRIGGER_OFFLINE,   /* 反馈离线，零电流 */
    TRIGGER_FAULT,     /* 故障锁存，须先失能解除 */
} Trigger_State_t;

/* 拨弹配置 */
typedef struct
{
    float max_speed_rpm;     /* 减速后输出轴转速上限，单位：rpm */
    float max_current_a;     /* 转子电流上限，单位：A */
    KPID_Params_t speed_pid; /* 速度环参数，输入输出轴 rpm，输出 A */
} Trigger_Config_t;

/* 拨弹连续速度命令，命令缺失时按失能处理 */
typedef struct
{
    bool enabled;           /* 闭环使能 */
    float target_speed_rpm; /* 输出轴目标转速，正负沿用设备反馈，单位：rpm */
} Trigger_Command_t;

/* 拨弹输出 */
typedef struct
{
    float current_a; /* M2006 转子电流，单位：A */
} Trigger_Output_t;

/* 拨弹实例，初始化后不得整体复制，PID 参数指针引用实例内部副本 */
typedef struct
{
    Trigger_Config_t config;         /* 可调配置副本 */
    bool initialized;                /* 初始化结果 */
    bool config_valid;               /* 本周期配置校验结果 */
    float frequency_hz;              /* 初始化控制频率，单位：Hz */
    KPID_Params_t applied_speed_pid; /* 控制器实际引用的参数 */
    Trigger_State_t state;           /* 唯一业务状态 */
    MOTOR_RM_Feedback_t feedback;    /* 最近有效设备反馈 */
    bool feedback_valid;             /* 最近有效反馈尚未超时 */
    uint64_t feedback_time_us;       /* 源时间，单位：us，UINT64_MAX 为无反馈 */
    float target_speed_rpm;          /* 限幅后的输出轴目标，单位：rpm */
    float current_a;                 /* 候选转子电流，单位：A */
    KPID_t speed_pid;                /* 速度控制器历史 */
} Trigger_t;

/* Exported functions ------------------------------------------------------- */
/**
 * @brief 复制拨弹配置并初始化速度控制器
 *
 * @param trigger 拨弹实例，初始化后不得整体复制
 * @param config 默认配置
 * @param frequency_hz 控制频率，单位：Hz
 * @return int8_t 成功返回 0，失败返回 -1 并保留零输出
 */
int8_t Trigger_Init(Trigger_t *trigger, const Trigger_Config_t *config, float frequency_hz);

/**
 * @brief 接纳拨弹新反馈并检查缓存时效
 *
 * @param trigger 拨弹实例
 * @param feedback 本周期新反馈，空指针时仅检查缓存是否超时
 * @param now_us 读取反馈后的完整单调时刻，单位：us
 * @return 无返回值
 */
void Trigger_UpdateFeedback(Trigger_t *trigger, const MOTOR_RM_Feedback_t *feedback, uint64_t now_us);

/**
 * @brief 推进拨弹状态机并执行速度闭环
 *
 * @param trigger 拨弹实例
 * @param command 连续速度命令，空指针按失能处理
 * @param period_s 实测控制周期，单位：s
 * @return 无返回值
 */
void Trigger_Control(Trigger_t *trigger, const Trigger_Command_t *command, float period_s);

/**
 * @brief 按拨弹运行状态导出本周期电流或零
 *
 * @param trigger 已初始化或已运行的拨弹实例
 * @param output 拨弹电流输出，单位：A
 * @return 无返回值
 */
void Trigger_DumpOutput(const Trigger_t *trigger, Trigger_Output_t *output);

#ifdef __cplusplus
}
#endif
