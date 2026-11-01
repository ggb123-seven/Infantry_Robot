#pragma once

#include "device/dr16.h"
#include "module/chassis.h"
#include "module/feeder.h"
#include "module/gimbal.h"

#ifdef __cplusplus
extern "C"
{
#endif

// 启动及异常后等待中位确认，中位就绪后才允许上位运行
typedef enum
{
    CMD_GIMBAL_WAIT_NEUTRAL = 0,
    CMD_GIMBAL_READY,
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
 * 右拨杆下位为运行，右摇杆上下双向映射到归一化速度比例，上位和中位为待机
 *
 * @param[in] rc 最近遥控快照，调用方保证非空
 * @param[in] rc_valid 已取得有效缓存时为 true
 * @param[in] now_us 读取快照后的当前时间，单位微秒
 * @param[out] out 拨弹命令，调用方保证非空
 * @return 遥控快照在线且时效有效时返回 true，否则返回 false
 */
bool Cmd_ParseFeeder(const DR16_State_t *rc, bool rc_valid, uint64_t now_us, Feeder_Command_t *out);

/**
 * @brief 解析云台遥控使能顺序与平滑速度，异常输入要求重新确认中位
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
 * @brief 撤销遥控使能与速度历史，要求重新确认中位
 *
 * @param[out] ctx 云台命令上下文，调用方保证非空
 * @return 无返回值
 */
void Cmd_ResetGimbal(Cmd_Gimbal_t *ctx);

#ifdef __cplusplus
}
#endif
