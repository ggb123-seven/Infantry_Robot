#pragma once

#include "device/dr16.h"
#include "module/chassis.h"
#include "module/feeder.h"
#include "module/gimbal.h"

#ifdef __cplusplus
extern "C"
{
#endif

// 遥控在线即允许云台运行，两侧拨杆不参与使能，失联或故障时回到未使能
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
 * 拨弹命令阶段，上位安全、中位备弹、下位开火，定量组的执行与释放重触发由拨弹模块独占
 * 状态同时记录备弹时选定的射击模式，模式切换后要求重新备弹
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

// 拨弹任务独占的命令上下文，state 保存射击模式及拨杆备弹阶段，初始为零
typedef struct
{
    Cmd_FeederState_t state;
} Cmd_Feeder_t;

/**
 * @brief 将遥控快照转换为底盘命令，异常输入输出待机
 *
 * @param[in] rc 最近遥控快照，可为 NULL
 * @param[in] rc_valid 已取得有效缓存时为 true
 * @param[in] now_us 读取快照后的当前时间，单位微秒
 * @param[out] out 底盘命令，可为 NULL
 * @return 无返回值
 */
void Cmd_ParseChassis(const DR16_State_t *rc, bool rc_valid, uint64_t now_us, Chassis_Command_t *out);

/**
 * @brief 按右拨杆生成安全、备弹和开火命令，模式切换后要求重新备弹
 *
 * 右拨杆上位安全、中位备弹、下位直接开火，左拨杆与右摇杆不参与射击选择
 * 固定发数的执行与释放重触发由拨弹模块负责
 *
 * @param[in,out] ctx 拨弹任务独占命令上下文，可为 NULL
 * @param[in] rc 最近遥控快照，可为 NULL
 * @param[in] rc_valid 已取得有效缓存时为 true
 * @param[in] now_us 读取快照后的当前时间，单位微秒
 * @param[in] fire_mode 遥控进给模式，0 连发、1 单发、2 五连发
 * @param[out] out 本周期拨弹命令，可为 NULL
 * @return 遥控、模式和右拨杆有效时为 true，否则为 false
 */
bool Cmd_ParseFeeder(Cmd_Feeder_t *ctx, const DR16_State_t *rc, bool rc_valid, uint64_t now_us,
                     uint8_t fire_mode, Feeder_Command_t *out);

/**
 * @brief 解析云台遥控平滑速度，遥控在线即进入角度模式
 *
 * @param[in,out] ctx 云台任务独占上下文，可为 NULL
 * @param[in] rc 最近遥控快照，可为 NULL
 * @param[in] rc_valid 已取得有效缓存时为 true
 * @param[in] now_us 读取快照后的当前时间，单位微秒
 * @param[in] period_s 控制周期，单位秒，合法范围 (0, 0.1]
 * @param[in] gimbal_fault 云台任一轴或共用惯性源故障锁存时为 true
 * @param[out] out 云台命令，可为 NULL
 * @return 快照与周期有效时为 true，否则为 false，调用方据此撤销缓存有效性
 */
bool Cmd_ParseGimbal(Cmd_Gimbal_t *ctx, const DR16_State_t *rc, bool rc_valid, uint64_t now_us,
                     float period_s, bool gimbal_fault, Gimbal_Command_t *out);

/**
 * @brief 根据环路开关选择两轴共用模式，失联和故障产生的待机不受影响
 *
 * @param[in,out] command 已解析的遥控命令，可为 NULL
 * @param[in] speed_enabled 速度环使能，关闭时整体待机
 * @param[in] angle_enabled 角度环使能，关闭时角度模式降为速度模式
 * @return 无返回值
 */
void Cmd_SelectGimbalLoops(Gimbal_Command_t *command, bool speed_enabled, bool angle_enabled);

/**
 * @brief 撤销遥控运行阶段与速度历史，恢复后重新捕获姿态
 *
 * @param[out] ctx 云台命令上下文，可为 NULL
 * @return 无返回值
 */
void Cmd_ResetGimbal(Cmd_Gimbal_t *ctx);

#ifdef __cplusplus
}
#endif
