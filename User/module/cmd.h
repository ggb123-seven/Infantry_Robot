#pragma once

#include "device/dr16.h"
#include "module/chassis.h"
#include "module/feeder.h"
#include "module/gimbal.h"

#ifdef __cplusplus
extern "C"
{
#endif

// 在线后进入运行，失联或故障撤销运行阶段并清除平滑历史
typedef enum
{
    CMD_GIMBAL_INACTIVE = 0,
    CMD_GIMBAL_ACTIVE,
} Cmd_GimbalState_t;

/*
 * 云台任务独占的命令上下文：state 为使能阶段
 * yaw_rate_deg_s、pitch_rate_deg_s 为上周期平滑角速度，单位度每秒
 */
typedef struct
{
    Cmd_GimbalState_t state;
    float yaw_rate_deg_s;
    float pitch_rate_deg_s;
} Cmd_Gimbal_t;

/*
 * 拨弹命令阶段保存已备弹或已开火的模式，上位安全、中位备弹、下位开火
 * 初次下位、失联恢复仍下位及运行中切换模式都须先回中位，不使用右摇杆触发
 */
typedef enum
{
    CMD_FEEDER_DISABLED = 0,
    CMD_FEEDER_AUTO_READY,
    CMD_FEEDER_AUTO_ACTIVE,
    CMD_FEEDER_SINGLE_READY,
    CMD_FEEDER_SINGLE_ACTIVE,
    CMD_FEEDER_BURST_READY,
    CMD_FEEDER_BURST_ACTIVE,
} Cmd_FeederState_t;

// 拨弹任务独占的命令上下文，state 保存射击模式及拨杆备弹阶段
typedef struct
{
    Cmd_FeederState_t state;
} Cmd_Feeder_t;

/**
 * @brief 将遥控快照转换为底盘命令，异常输入输出待机
 *
 * @param[in] rc 最近遥控快照，调用方保证非空
 * @param[in] rc_valid 已取得有效缓存时为 true
 * @param[in] now_us 读取快照后的当前时间，单位微秒
 * @param[out] out 底盘命令，调用方保证非空
 * @return 遥控快照在线且时效有效时返回 true，否则返回 false
 */
bool Cmd_ParseChassis(const DR16_State_t *rc, bool rc_valid, uint64_t now_us, Chassis_Command_t *out);

/**
 * @brief 将遥控快照转换为拨弹命令，异常输入输出离线档
 *
 * 右拨杆上位安全、中位备弹、下位开火，左拨杆与右摇杆不参与射击选择
 * 启动及模式切换先等待右拨杆回中，固定发数的执行与释放重触发由拨弹模块负责
 *
 * @param[in,out] ctx 拨弹任务独占命令上下文，调用方保证非空且初始为零
 * @param[in] rc 最近遥控快照，调用方保证非空
 * @param[in] rc_valid 已取得有效缓存时为 true
 * @param[in] now_us 读取快照后的当前时间，单位微秒
 * @param[in] fire_mode 遥控进给模式，0 连发、1 单发、2 五连发，非法值输出离线档
 * @param[out] out 拨弹命令，调用方保证非空
 * @return 遥控、模式和右拨杆有效时返回 true，否则返回 false
 */
bool Cmd_ParseFeeder(Cmd_Feeder_t *ctx, const DR16_State_t *rc, bool rc_valid, uint64_t now_us,
                     uint8_t fire_mode, Feeder_Command_t *out);

/**
 * @brief 解析在线遥控的云台平滑速度，异常输入撤销运行请求
 *
 * @param[in,out] ctx 云台任务独占上下文，调用方保证非空
 * @param[in] rc 最近遥控快照，调用方保证非空
 * @param[in] rc_valid 已取得有效缓存时为 true
 * @param[in] now_us 读取快照后的当前时间，单位微秒
 * @param[in] period_s 控制周期，单位秒，合法范围 (0, 0.1]
 * @param[in] gimbal_fault 云台任一轴或共用惯性源故障锁存时为 true
 * @param[out] out 云台命令，调用方保证非空
 * @return 快照与周期有效时为 true，否则为 false，调用方据此撤销缓存有效性
 */
bool Cmd_ParseGimbal(Cmd_Gimbal_t *ctx, const DR16_State_t *rc, bool rc_valid, uint64_t now_us,
                     float period_s, bool gimbal_fault, Gimbal_Command_t *out);

/**
 * @brief 根据遥控环路开关选择两轴共用模式
 *
 * @param[in,out] command 已解析的遥控命令，调用方保证非空
 * @param[in] speed_enabled 速度环使能
 * @param[in] angle_enabled 角度环使能
 * @return 无返回值
 */
void Cmd_SelectGimbalLoops(Gimbal_Command_t *command, bool speed_enabled, bool angle_enabled);

#if MOTOR_GIMBAL_ENABLE_SINE_TEST
/**
 * @brief 根据云台状态生成正弦角度目标与速度前馈
 *
 * @param[in] gimbal 云台实例，调用方保证非空
 * @param[in,out] command 本周期命令，调用方保证非空
 * @param[in] period_s 实际控制间隔，单位秒
 * @param[in] debug_enabled 手动调试开关
 * @param[in] enabled 正弦测试开关
 * @param[in] feedforward_enabled 速度前馈开关
 * @return 无返回值
 */
void Cmd_UpdateGimbalSineTest(const Gimbal_t *gimbal, Gimbal_Command_t *command, float period_s,
                              bool debug_enabled, bool enabled, bool feedforward_enabled);
#endif

/**
 * @brief 撤销遥控运行阶段与速度历史，恢复后重新捕获姿态
 *
 * @param[out] ctx 云台命令上下文，调用方保证非空
 * @return 无返回值
 */
void Cmd_ResetGimbal(Cmd_Gimbal_t *ctx);

#ifdef __cplusplus
}
#endif
