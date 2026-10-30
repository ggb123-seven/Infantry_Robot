#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

#include "device/motor_rm.h"

/*
 * CAN 设备聚合参数：CAN1 注册四路 M3508，CAN2 注册两路 GM6020、两路 M3508 和一路 M2006
 * 云台索引 0、1 分别对应 ID5 yaw 和 ID6 pitch，反馈标准帧为 0x209、0x20A
 * 发射索引 0、1、2 分别对应摩擦轮 ID5、ID6 与拨弹 ID7，反馈为 0x205、0x206、0x207
 */
#define CAN_DEVICES_CHASSIS_MOTOR_COUNT (4U)
#define CAN_DEVICES_GIMBAL_MOTOR_COUNT (2U)
#define CAN_DEVICES_GIMBAL_YAW (0U)
#define CAN_DEVICES_GIMBAL_PITCH (1U)
#define CAN_DEVICES_SHOOT_MOTOR_COUNT (3U)
#define CAN_DEVICES_SHOOT_FRIC1 (0U)
#define CAN_DEVICES_SHOOT_FRIC2 (1U)
#define CAN_DEVICES_SHOOT_TRIGGER (2U)

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
 * 整车 CAN 设备集合快照：
 * - initialized：CAN 总线初始化成功时为 true，允许单个电机注册失败后隔离运行
 * - init_status：总线初始化、六路 M3508、两路 GM6020 与一路 M2006 注册的聚合结果
 * - feedback_status：四路 M3508 本周期反馈更新的聚合结果
 * - output_status：四路 M3508 本周期电流提交的聚合结果
 * - tx_status：四路 M3508 槽位处理完成后的 0x200 控制帧发送结果
 * - register_status[0~3]：四个 M3508 的注册结果
 * - feedback_update_status[0~3]：四个 M3508 的本周期反馈更新结果
 * - current_set_status[0~3]：四路 M3508 电流命令的校验与缓存写入结果
 * - motor_online[0~3]：对应 M3508 最近 100 ms 内收到反馈时为 true
 * - actual_speed_rpm[0~3]、temperature_c[0~3]：M3508 输出轴转速和温度，单位分别为 rpm 和摄氏度
 * - feedback_time_us[0~3]：各轮源反馈解码时间低 32 位，单位微秒，接收端按帧龄恢复完整时刻
 * - applied_current_a[0~3]：经过在线校验和故障归零后写入的 M3508 转子侧电流，单位 A
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
    uint32_t feedback_time_us[CAN_DEVICES_CHASSIS_MOTOR_COUNT];
    float temperature_c[CAN_DEVICES_CHASSIS_MOTOR_COUNT];
    float applied_current_a[CAN_DEVICES_CHASSIS_MOTOR_COUNT];
    uint32_t sequence;
    uint32_t applied_command_sequence;
} CANDevices_Snapshot_t;

/**
 * @brief 初始化 CAN 总线并注册底盘、云台、双摩擦轮与拨弹电机
 *
 * 单个电机注册失败时保留其他电机运行能力，并在快照中记录对应注册状态
 *
 * @param[out] snapshot CAN 设备集合初始化结果快照
 * @return 全部设备注册成功返回 CAN_DEVICES_OK，部分失败或总线不可用时返回对应状态码
 */
int8_t CANDevices_Init(CANDevices_Snapshot_t *snapshot);

/**
 * @brief 更新四个底盘 M3508 的最新反馈
 *
 * @param[out] snapshot 本周期 CAN 设备反馈快照
 * @return 四路均取得新反馈返回 CAN_DEVICES_OK，否则返回 CAN_DEVICES_ERROR
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
 * @brief 更新单路云台 GM6020 反馈，供 CAN 任务发布至独立邮箱
 *
 * @param[in] axis 云台轴索引，yaw 为 CAN_DEVICES_GIMBAL_YAW，pitch 为 CAN_DEVICES_GIMBAL_PITCH
 * @param[out] feedback 电机最新有效反馈，转速单位 rpm，last_update_time 单位为微秒，失败时清零
 * @return 取得新反馈返回 CAN_DEVICES_OK，否则返回设备错误码且不得发布本次结果
 */
int8_t CANDevices_UpdateGimbalFeedback(uint8_t axis, MOTOR_RM_Feedback_t *feedback);

/**
 * @brief 分别写入 yaw 和 pitch 电流，统一发送一次 0x2FE 帧
 *
 * @param[in] yaw_current_a yaw 转子侧电流，单位 A，传入零表示停机，非有限值按零处理
 * @param[in] pitch_current_a pitch 转子侧电流，单位 A，传入零表示停机，非有限值按零处理
 * @return 两路缓存更新且发送成功返回 CAN_DEVICES_OK，非法输入、设备不可用或发送失败返回错误码
 */
int8_t CANDevices_ApplyGimbalCurrent(float yaw_current_a, float pitch_current_a);

/**
 * @brief 更新单路发射机构电机的新反馈
 *
 * @param[in] index 电机索引，0、1 为摩擦轮，2 为拨弹
 * @param[out] feedback 新反馈，摩擦轮为转子 rpm、拨弹为输出轴 rpm，时间戳为微秒
 * @return 取得新反馈返回 CAN_DEVICES_OK，否则返回设备错误码，输出清零
 */
int8_t CANDevices_UpdateShootFeedback(uint8_t index, MOTOR_RM_Feedback_t *feedback);

/**
 * @brief 合并两只摩擦轮与拨弹电流并统一发送一次 CAN2 的 0x1FF 帧
 *
 * @param[in] fric1_a 第一只摩擦轮的转子电流，单位 A，非有限值按零处理
 * @param[in] fric2_a 第二只摩擦轮的转子电流，单位 A，非有限值按零处理
 * @param[in] trigger_a 拨弹电机的转子电流，单位 A，非有限值按零处理
 * @return 全部写入并发送成功返回 CAN_DEVICES_OK，否则返回 CAN_DEVICES_ERROR
 */
int8_t CANDevices_ApplyShootCurrent(float fric1_a, float fric2_a, float trigger_a);

#ifdef __cplusplus
}
#endif
