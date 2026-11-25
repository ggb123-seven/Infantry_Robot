/*
 * 拨弹模块对外接口
 *
 * 命令层每周期生成拨弹命令，模块按环路使能选择控制结构，最终由输出闸门给出转子电流
 */

#pragma once

/* Includes ----------------------------------------------------------------- */
#include "component/pid.h"
#include "device/motor_rm.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* Exported types ----------------------------------------------------------- */
/* 拨弹运行状态，非运行状态由输出闸门统一输出零电流 */
typedef enum
{
    FEEDER_RELAX = 0, /* 待机，回中释放或定量组完成 */
    FEEDER_RUNNING,   /* 运行，按环路结构输出电流 */
    FEEDER_OFFLINE,   /* 反馈超时或角度不连续，零输出待恢复 */
    FEEDER_FAULT,     /* 故障锁存，命令回到待机档才解除 */
} Feeder_State_t;

/* 拨弹命令档位，离线档不解除故障锁存，待机档可解除 */
typedef enum
{
    FEEDER_CMD_OFFLINE = 0, /* 离线，保持零输出 */
    FEEDER_CMD_RELAX,       /* 待机，可解除故障锁存 */
    FEEDER_CMD_RUN,         /* 运行，允许当前环路结构输出 */
} Feeder_CmdMode_t;

/* 进给模式，命令层唯一选择来源，遥控默认五连发 */
#define FEEDER_FIRE_MODE_AUTO (0U)   /* 连发，按倍率进给 */
#define FEEDER_FIRE_MODE_SINGLE (1U) /* 单发，逐齿角度进给 */
#define FEEDER_FIRE_MODE_BURST (2U)  /* 五连发，逐齿计数进给 */

extern volatile uint8_t feeder_fire_mode;      /* 实际进给模式，仅供观测 */
extern volatile float feeder_angle_target_deg; /* 单发输出轴目标角，单位：度 */

/*
 * 环路在线使能，每周期在入口各读取一次，默认只启用角度环
 * 仅角度环时直接输出电流 A，串级时角度环输出 rad/s 并接速度环
 * 两环都关闭时全部撤驱，结构改变会取消本组目标并清除控制历史
 */
extern volatile bool feeder_angle_loop_enabled; /* 角度环使能 */
extern volatile bool feeder_speed_loop_enabled; /* 速度环使能 */

/* 拨弹命令，由命令层每周期生成，模块只读 */
typedef struct
{
    Feeder_CmdMode_t mode; /* 命令档位 */
    float speed_ratio;     /* 连发归一化转速倍率 */
    uint8_t fire_mode;     /* 进给模式，0 连发、1 单发、2 五连发 */
    bool single_fire;      /* 单发或五连发持续开火请求 */
} Feeder_Command_t;

/* 拨弹可调配置，初始化和在线调参均通过实例副本生效 */
typedef struct
{
    float tooth_count;                      /* 一圈齿数，单发步距为 360 度除齿数 */
    float feed_hz;                          /* 单发重复进给齿数上限，单位：Hz */
    float direction;                        /* 角度递增方向，取 -1 或 +1 */
    float max_speed_rpm;                    /* 输出轴速度目标上限，单位：rpm */
    float max_current_a;                    /* 转子电流上限，单位：A */
    KPID_Params_t position_pid;             /* 角度环参数，输出单位随环路结构切换 */
    KPID_Params_t speed_pid;                /* 速度内环参数，输入输出轴 rpm，输出 A */
    float speed_feedforward_gain_a_per_rpm; /* 目标差分电流前馈系数，单位：A/rpm */
} Feeder_Config_t;

/* 唯一拨弹实例，保存配置副本、运行状态和控制观测 */
typedef struct
{
    Feeder_Config_t config;   /* EmberProbe 可写配置副本 */
    bool config_valid;        /* 最近一次 Control 周期的配置校验结果 */
    Feeder_State_t state;     /* 唯一运行状态，输出闸门依据 */
    float measured_angle_deg; /* 输出轴单圈角度，单位：度 */
    float measured_speed_rpm; /* 输出轴实测转速，单位：rpm */
    float target_angle_deg;   /* 角度模式采用的目标，单位：度 */
    float target_speed_rpm;   /* 速度内环目标，单位：rpm */
    float output_current_a;   /* 限幅后的候选转子电流，单位：A */
} Feeder_t;

/* Exported functions ------------------------------------------------------- */
/**
 * @brief 复制默认拨弹配置并初始化角度外环和速度内环
 *
 * @param feeder 拨弹实例
 * @param frequency_hz 调度频率，单位：Hz
 * @return int8_t 成功返回 0，失败返回 -1
 */
int8_t Feeder_Init(Feeder_t *feeder, float frequency_hz);

/**
 * @brief 在输入边界接收新反馈并按源时间检查缓存帧龄
 *
 * @param feeder 拨弹实例
 * @param feedback 输出轴反馈，转速单位：rpm，单圈角度单位：rad
 * @param received 驱动取得完整新反馈时为 true
 * @param now_us 读取反馈后的完整单调微秒时间
 * @return 无返回值
 */
void Feeder_UpdateFeedback(Feeder_t *feeder, const MOTOR_RM_Feedback_t *feedback, bool received, uint64_t now_us);

/**
 * @brief 校验周期配置，按环路使能执行速度环、单角度环或逐发串级控制
 *
 * 单发及五连发收到开火请求时捕获当前位置并逐齿执行固定发数
 * 未完成的组在反馈恢复后从当前位置重新计发，已完成的组保留完成状态，不重复开火
 * 到位判据为输出轴误差不超过 0.5 度、绝对转速不超过 1 rpm 并持续 20 毫秒
 *
 * @param feeder 拨弹实例
 * @param command 本周期拨弹命令，无命令时传 NULL，按离线档处理
 * @param period_s 实测控制周期，单位：s
 * @return 无返回值
 */
void Feeder_Control(Feeder_t *feeder, const Feeder_Command_t *command, float period_s);

/**
 * @brief 根据唯一运行状态导出电流，其他状态统一输出零
 *
 * @param feeder 拨弹实例
 * @return float 转子侧目标电流，单位：A
 */
float Feeder_DumpOutput(const Feeder_t *feeder);

#ifdef __cplusplus
}
#endif
