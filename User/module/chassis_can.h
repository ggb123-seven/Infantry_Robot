#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

#define CHASSIS_CAN_MOTOR_COUNT (4U)

typedef enum
{
    CHASSIS_CAN_OK = 0,
    CHASSIS_CAN_ERROR = -1,
    CHASSIS_CAN_NULL_ERROR = -2,
    CHASSIS_CAN_NOT_INITIALIZED = -3,
    CHASSIS_CAN_DEVICE_UNAVAILABLE = -4,
    CHASSIS_CAN_INVALID_CURRENT = -5,
} ChassisCAN_Status_t;

/**
 * @brief 四电机 CAN 边界的单实例运行上下文
 *
 * 字段说明：
 * - initialized：CAN 总线初始化成功后为 true，允许周期读写设备。
 * - register_status[0~3]：C620 电调 ID 1~4 对应 M3508 的注册结果，0 表示成功。
 */
typedef struct
{
    bool initialized;
    int8_t register_status[CHASSIS_CAN_MOTOR_COUNT];
} ChassisCAN_t;

/**
 * @brief 四电机 CAN 反馈的一致快照
 *
 * 字段说明：
 * - actual_speed_rpm[0~3]：C620 电调 ID 1~4 对应电机输出轴转速，单位 rpm。
 * - motor_online[0~3]：对应电机最近 100 毫秒内存在有效反馈时为 true。
 * - temperature_c[0~3]：对应电机反馈温度，单位摄氏度。
 * - feedback_update_status[0~3]：本周期对应设备反馈更新结果，0 表示收到新反馈。
 * - register_status[0~3]：对应设备注册结果，0 表示成功。
 */
typedef struct
{
    float actual_speed_rpm[CHASSIS_CAN_MOTOR_COUNT];
    bool motor_online[CHASSIS_CAN_MOTOR_COUNT];
    float temperature_c[CHASSIS_CAN_MOTOR_COUNT];
    int8_t feedback_update_status[CHASSIS_CAN_MOTOR_COUNT];
    int8_t register_status[CHASSIS_CAN_MOTOR_COUNT];
} ChassisCAN_Feedback_t;

/**
 * @brief 四电机 CAN 电流提交状态快照
 *
 * 字段说明：
 * - current_set_status[0~3]：对应电机电流槽位写入结果，0 表示成功。
 * - can_tx_status：四个槽位写入后统一发送控制帧的结果，0 表示成功。
 */
typedef struct
{
    int8_t current_set_status[CHASSIS_CAN_MOTOR_COUNT];
    int8_t can_tx_status;
} ChassisCAN_OutputStatus_t;

/**
 * @brief 初始化 CAN 总线并注册四个 M3508 设备
 *
 * @param[out] context 四电机 CAN 边界上下文
 * @return 全部设备注册成功返回 CHASSIS_CAN_OK，部分注册失败返回 CHASSIS_CAN_ERROR，参数或总线失败返回对应状态码
 */
int8_t ChassisCAN_Init(ChassisCAN_t *context);

/**
 * @brief 刷新四个 M3508 反馈并生成安全一致快照
 *
 * @param[in] context 已初始化的四电机 CAN 边界上下文
 * @param[out] feedback 本周期反馈快照
 * @return 四路均收到新反馈返回 CHASSIS_CAN_OK，否则返回对应状态码
 */
int8_t ChassisCAN_ReadFeedback(const ChassisCAN_t *context, ChassisCAN_Feedback_t *feedback);

/**
 * @brief 写入四路电流槽位并统一发送一次 CAN 控制帧
 *
 * 非有限电流和单路写入失败均使用零电流覆盖对应槽位，其他电机不受影响。
 *
 * @param[in] context 已初始化的四电机 CAN 边界上下文
 * @param[in] current_command_a 四路转子侧目标电流，单位 A
 * @param[out] status 本周期四路槽位写入和统一发送状态
 * @return 四路写入和统一发送均成功返回 CHASSIS_CAN_OK，否则返回对应状态码
 */
int8_t ChassisCAN_WriteCurrent(const ChassisCAN_t *context,
                               const float current_command_a[CHASSIS_CAN_MOTOR_COUNT],
                               ChassisCAN_OutputStatus_t *status);

#ifdef __cplusplus
}
#endif
