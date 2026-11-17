/*
 * 遥控命令解析接口
 *
 * 把 DR16 快照转换为底盘、云台和拨弹命令，异常输入统一输出待机与离线档
 */

#pragma once

/* Includes ----------------------------------------------------------------- */
#include "device/dr16.h"
#include "module/chassis.h"
#include "module/feeder.h"
#include "module/gimbal.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* Exported types ----------------------------------------------------------- */
/* 云台使能阶段，失联或故障撤销运行阶段并清除平滑历史 */
typedef enum
{
    CMD_GIMBAL_INACTIVE = 0, /* 未使能，等待首次上线 */
    CMD_GIMBAL_ACTIVE,       /* 运行，允许遥控速度过渡 */
} Cmd_GimbalState_t;

/* 云台任务独占的命令上下文 */
typedef struct
{
    Cmd_GimbalState_t state; /* 使能阶段 */
    float yaw_rate_deg_s;    /* 上周期平滑 yaw 角速度，单位：度/s */
    float pitch_rate_deg_s;  /* 上周期平滑 pitch 角速度，单位：度/s */
} Cmd_Gimbal_t;

/* 拨弹命令阶段，上位安全、中位备弹、下位开火，定量组执行由拨弹模块独占 */
typedef enum
{
    CMD_FEEDER_DISABLED = 0,  /* 安全，禁止进给 */
    CMD_FEEDER_AUTO_READY,    /* 连发备弹 */
    CMD_FEEDER_AUTO_ACTIVE,   /* 连发开火 */
    CMD_FEEDER_SINGLE_READY,  /* 单发备弹 */
    CMD_FEEDER_SINGLE_ACTIVE, /* 单发开火 */
    CMD_FEEDER_BURST_READY,   /* 五连发备弹 */
    CMD_FEEDER_BURST_ACTIVE,  /* 五连发开火 */
} Cmd_FeederState_t;

/* 拨弹任务独占的命令上下文，state 保存射击模式及拨杆备弹阶段 */
typedef struct
{
    Cmd_FeederState_t state; /* 射击模式及备弹阶段 */
} Cmd_Feeder_t;

/* Exported functions ------------------------------------------------------- */
/**
 * @brief 将遥控快照转换为底盘命令，异常输入输出待机
 *
 * @param rc 最近遥控快照
 * @param rc_valid 已取得有效缓存时为 true
 * @param now_us 读取快照后的当前时间，单位：us
 * @param out 底盘命令
 * @return bool 遥控快照在线且时效有效时返回 true
 */
bool Cmd_ParseChassis(const DR16_State_t *rc, bool rc_valid, uint64_t now_us, Chassis_Command_t *out);

/**
 * @brief 将遥控快照转换为拨弹命令，异常输入输出离线档
 *
 * 右拨杆上位安全、中位备弹、下位直接开火，左拨杆与右摇杆不参与射击选择
 * 固定发数的执行与释放重触发由拨弹模块负责
 *
 * @param ctx 拨弹任务独占命令上下文，初始为零
 * @param rc 最近遥控快照
 * @param rc_valid 已取得有效缓存时为 true
 * @param now_us 读取快照后的当前时间，单位：us
 * @param fire_mode 遥控进给模式，0 连发、1 单发、2 五连发
 * @param out 拨弹命令
 * @return bool 遥控、模式和右拨杆有效时返回 true
 */
bool Cmd_ParseFeeder(Cmd_Feeder_t *ctx, const DR16_State_t *rc, bool rc_valid, uint64_t now_us,
                     uint8_t fire_mode, Feeder_Command_t *out);

/**
 * @brief 解析在线遥控的云台平滑速度，异常输入撤销运行请求
 *
 * @param ctx 云台任务独占上下文
 * @param rc 最近遥控快照
 * @param rc_valid 已取得有效缓存时为 true
 * @param now_us 读取快照后的当前时间，单位：us
 * @param period_s 控制周期，单位：s
 * @param gimbal_fault 云台任一轴或共用惯性源故障锁存时为 true
 * @param out 云台命令
 * @return bool 快照与周期有效时为 true，调用方据此撤销缓存有效性
 */
bool Cmd_ParseGimbal(Cmd_Gimbal_t *ctx, const DR16_State_t *rc, bool rc_valid, uint64_t now_us,
                     float period_s, bool gimbal_fault, Gimbal_Command_t *out);

/**
 * @brief 根据遥控环路开关选择两轴共用模式
 *
 * @param command 已解析的遥控命令
 * @param speed_enabled 速度环使能
 * @param angle_enabled 角度环使能
 * @return 无返回值
 */
void Cmd_SelectGimbalLoops(Gimbal_Command_t *command, bool speed_enabled, bool angle_enabled);

#if MOTOR_GIMBAL_ENABLE_SINE_TEST
/**
 * @brief 根据云台状态生成正弦角度目标与速度前馈
 *
 * @param gimbal 云台实例
 * @param command 本周期命令
 * @param period_s 实际控制间隔，单位：s
 * @param debug_enabled 手动调试开关
 * @param enabled 正弦测试开关
 * @param feedforward_enabled 速度前馈开关
 * @return 无返回值
 */
void Cmd_UpdateGimbalSineTest(const Gimbal_t *gimbal, Gimbal_Command_t *command, float period_s,
                              bool debug_enabled, bool enabled, bool feedforward_enabled);
#endif

/**
 * @brief 撤销遥控运行阶段与速度历史，恢复后重新捕获姿态
 *
 * @param ctx 云台命令上下文
 * @return 无返回值
 */
void Cmd_ResetGimbal(Cmd_Gimbal_t *ctx);

#ifdef __cplusplus
}
#endif
