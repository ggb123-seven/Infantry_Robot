#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

/*
 * CAN 设备聚合参数：底盘固定注册四路 M3508，云台固定注册一路 GM6020
 * GM6020 电流环给定范围为 -3~3 A，对应设备协议原始范围 -16384~16384
 */
#define CAN_DEVICES_CHASSIS_MOTOR_COUNT (4U)
#define CAN_DEVICES_GIMBAL_MOTOR_COUNT (1U)
#define CAN_DEVICES_GM6020_CURRENT_LIMIT_A (3.0F)

/**
 * @brief 整车 CAN 设备集合状态
 */
typedef enum
{
    CAN_DEVICES_OK = 0,
    CAN_DEVICES_ERROR = -1,
    CAN_DEVICES_NULL_ERROR = -2,
    CAN_DEVICES_NOT_INITIALIZED = -3,
    CAN_DEVICES_CAN_INIT_ERROR = -4,
    CAN_DEVICES_DEVICE_UNAVAILABLE = -5,
    CAN_DEVICES_INVALID_CURRENT = -6,
} CANDevices_Status_t;

/*
 * 四个底盘 M3508 的本周期电流命令：
 * - sequence：命令快照序号，用于后续任务邮箱诊断，不参与电机协议计算
 * - current_a[0~3]：依次对应 C620 电调 ID 1~4 的转子侧电流命令，单位 A
 */
typedef struct
{
    uint32_t sequence;
    float current_a[CAN_DEVICES_CHASSIS_MOTOR_COUNT];
} CANDevices_Command_t;

/*
 * 云台 GM6020 的独立电流命令快照
 * - sequence：命令快照序号，仅用于任务间诊断，不参与电机协议计算
 * - current_a：GM6020 转子侧目标电流，单位 A，安全范围由设备层校验
 */
typedef struct
{
    uint32_t sequence;
    float current_a;
} CANDevices_GimbalCommand_t;

/*
 * 整车 CAN 设备集合快照：
 * - initialized：CAN1 可运行时为 true，允许单个电机注册失败后隔离运行
 * - init_status：CAN1 初始化、四路 M3508 和一路 GM6020 注册的聚合结果
 * - feedback_status：四路 M3508 和一路 GM6020 本周期反馈更新的聚合结果
 * - output_status：四路 M3508 本周期电流提交的聚合结果
 * - tx_status：四路 M3508 槽位处理完成后的 0x200 控制帧发送结果
 * - register_status[0~3]：四个 M3508 的注册结果
 * - feedback_update_status[0~3]：四个 M3508 的本周期反馈更新结果
 * - current_set_status[0~3]：四路 M3508 电流命令的校验与缓存写入结果
 * - motor_online[0~3]：对应 M3508 最近 100 ms 内收到反馈时为 true
 * - actual_speed_rpm[0~3]、temperature_c[0~3]：M3508 输出轴转速和温度，单位分别为 rpm 和摄氏度
 * - applied_current_a[0~3]：经过在线校验和故障归零后写入的 M3508 转子侧电流，单位 A
 * - gm6020_register_status、gm6020_feedback_update_status：GM6020 的注册和本周期反馈更新结果
 * - gm6020_online：GM6020 最近 100 ms 内收到反馈时为 true
 * - gm6020_angle_rad：GM6020 转子单圈角度，范围 [0, 2π)，单位 rad
 * - gm6020_speed_rpm、gm6020_raw_current_lsb、gm6020_torque_current_a、gm6020_temperature_c：
 *   GM6020 转速、原始电流、换算电流和温度，单位分别为 rpm、LSB、A 和摄氏度
 * - sequence：设备反馈周期序号，每次反馈更新调用递增
 * - applied_command_sequence：最近一次电流提交采用的命令快照序号
 */
typedef struct
{
    bool initialized;
    int8_t init_status;
    int8_t feedback_status;
    int8_t output_status;
    int8_t tx_status;
    int8_t register_status[CAN_DEVICES_CHASSIS_MOTOR_COUNT];
    int8_t feedback_update_status[CAN_DEVICES_CHASSIS_MOTOR_COUNT];
    int8_t current_set_status[CAN_DEVICES_CHASSIS_MOTOR_COUNT];
    bool motor_online[CAN_DEVICES_CHASSIS_MOTOR_COUNT];
    float actual_speed_rpm[CAN_DEVICES_CHASSIS_MOTOR_COUNT];
    float temperature_c[CAN_DEVICES_CHASSIS_MOTOR_COUNT];
    float applied_current_a[CAN_DEVICES_CHASSIS_MOTOR_COUNT];
    int8_t gm6020_register_status;
    int8_t gm6020_feedback_update_status;
    int8_t gm6020_current_set_status;
    int8_t gm6020_tx_status;
    bool gm6020_online;
    float gm6020_angle_rad;
    float gm6020_speed_rpm;
    int16_t gm6020_raw_current_lsb;
    float gm6020_torque_current_a;
    float gm6020_temperature_c;
    float gm6020_applied_current_a;
    uint32_t sequence;
    uint32_t applied_command_sequence;
    uint32_t gm6020_applied_command_sequence;
} CANDevices_Snapshot_t;

/**
 * @brief 初始化 CAN1 并注册四个底盘 M3508 和一个 GM6020
 *
 * 单个电机注册失败时保留其他电机运行能力，并在快照中记录对应注册状态
 *
 * @param[out] snapshot CAN 设备集合初始化结果快照
 * @return 全部设备注册成功返回 CAN_DEVICES_OK，部分失败或总线不可用时返回对应状态码
 */
int8_t CANDevices_Init(CANDevices_Snapshot_t *snapshot);

/**
 * @brief 更新四个底盘 M3508 和一个 GM6020 的最新反馈
 *
 * @param[out] snapshot 本周期 CAN 设备反馈快照
 * @return 五路均取得新反馈返回 CAN_DEVICES_OK，否则返回 CAN_DEVICES_ERROR
 */
int8_t CANDevices_UpdateFeedback(CANDevices_Snapshot_t *snapshot);

/**
 * @brief 校验并写入四路电流后统一发送一次 RM 控制帧
 *
 * 非法电流、反馈未更新或电机离线时对应槽位写零；命令为空时四路全部按零电流处理
 *
 * @param[in] command 本周期四路电流命令快照，允许为 NULL
 * @param[in,out] snapshot 本周期 CAN 设备反馈与输出结果快照
 * @return 四路命令均有效且统一发送成功返回 CAN_DEVICES_OK，否则返回 CAN_DEVICES_ERROR
 */
int8_t CANDevices_ApplyCurrent(const CANDevices_Command_t *command, CANDevices_Snapshot_t *snapshot);

/**
 * @brief 校验并提交 GM6020 独立电流命令后发送对应控制帧
 *
 * 空命令、非法电流、反馈离线或写入失败时均向 GM6020 写入零电流
 * 该接口只刷新 GM6020 控制组，不会发送底盘四路控制帧
 *
 * @param[in] command 云台电流命令快照，允许为 NULL
 * @param[in,out] snapshot CAN 设备反馈与云台输出结果快照
 * @return 命令有效且 GM6020 控制帧发送成功时返回 CAN_DEVICES_OK，否则返回对应状态码
 */
int8_t CANDevices_ApplyGimbalCurrent(const CANDevices_GimbalCommand_t *command,
                                     CANDevices_Snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif
