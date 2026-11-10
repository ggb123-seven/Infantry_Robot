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
 * 拨弹命令阶段：中位停机，上位选择单发，下位选择连发，与左拨杆完全解耦
 * 启动和单连发切换均先等待右摇杆回中，再允许对应模式运行
 * 单发就绪后保持向上端部电平请求，执行和到位判定由拨弹模块负责
 */
typedef enum
{
    CMD_FEEDER_DISABLED = 0,
    CMD_FEEDER_AUTO_WAIT_CENTER,
    CMD_FEEDER_AUTO_READY,
    CMD_FEEDER_SINGLE_WAIT_CENTER,
    CMD_FEEDER_SINGLE_READY,
} Cmd_FeederState_t;

// 拨弹任务独占的命令上下文，state 保存射击模式及摇杆回中阶段
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
 * 右拨杆上位选择单发，下位选择连发，中位待机，左拨杆不参与射击模式选择
 * 启动及模式切换先等待右摇杆回中；连发保留双向速度比例，单发仅向上端部产生持续请求
 *
 * @param[in,out] ctx 拨弹任务独占命令上下文，调用方保证非空且初始为零
 * @param[in] rc 最近遥控快照，调用方保证非空
 * @param[in] rc_valid 已取得有效缓存时为 true
 * @param[in] now_us 读取快照后的当前时间，单位微秒
 * @param[out] out 拨弹命令，调用方保证非空
 * @return 遥控快照在线且时效有效时返回 true，否则返回 false
 */
bool Cmd_ParseFeeder(Cmd_Feeder_t *ctx, const DR16_State_t *rc, bool rc_valid, uint64_t now_us,
                     Feeder_Command_t *out);

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
 * @brief 撤销遥控运行阶段与速度历史，恢复后重新捕获姿态
 *
 * @param[out] ctx 云台命令上下文，调用方保证非空
 * @return 无返回值
 */
void Cmd_ResetGimbal(Cmd_Gimbal_t *ctx);

#ifdef __cplusplus
}
#endif
