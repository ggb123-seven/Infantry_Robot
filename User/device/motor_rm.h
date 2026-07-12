#pragma once

#include "motor.h"
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */
#include "device/device.h"
#include "device/motor.h"
#include "bsp/can.h"

/* Exported constants ------------------------------------------------------- */
#define MOTOR_RM_MAX_MOTORS 11

/* Exported macro ----------------------------------------------------------- */
/* Exported types ----------------------------------------------------------- */
typedef enum {
    MOTOR_M2006,
    MOTOR_M3508,
    MOTOR_GM6020,
} MOTOR_RM_Module_t;

typedef enum {
    MOTOR_RM_C610_ERROR_NONE = 0,
    MOTOR_RM_C610_ERROR_SUPPLY_OVERVOLTAGE = 2,
    MOTOR_RM_C610_ERROR_PHASE_LOSS = 3,
    MOTOR_RM_C610_ERROR_SENSOR_LOST = 4,
    MOTOR_RM_C610_ERROR_STALL = 6,
    MOTOR_RM_C610_ERROR_CALIBRATION_FAILED = 7,
} MOTOR_RM_C610_ErrorCode_t;

/*一个can最多控制11个电机*/
typedef union {
  int16_t output[MOTOR_RM_MAX_MOTORS];
  struct {
    int16_t m3508_m2006_id201;
    int16_t m3508_m2006_id202;
    int16_t m3508_m2006_id203;
    int16_t m3508_m2006_id204;
    int16_t m3508_m2006_gm6020_id205;
    int16_t m3508_m2006_gm6020_id206;
    int16_t m3508_m2006_gm6020_id207;
    int16_t m3508_m2006_gm6020_id208;
    int16_t gm6020_id209;
    int16_t gm6020_id20A;
    int16_t gm6020_id20B;
  } named;
} MOTOR_RM_MsgOutput_t;

/*每个电机需要的参数*/
typedef struct {
    BSP_CAN_t can;
    uint16_t id;
    MOTOR_RM_Module_t module;
    bool reverse;
    bool gear;
} MOTOR_RM_Param_t;

typedef MOTOR_Feedback_t MOTOR_RM_Feedback_t;

typedef struct {
    MOTOR_RM_Param_t param;
    MOTOR_RM_Feedback_t feedback;
    MOTOR_t motor;
    // 多圈相关变量，仅gear模式下有效
    uint16_t last_raw_angle;
    bool angle_inited;
    int32_t gearbox_round_count;
    int32_t gearbox_total_raw_count;
    float gearbox_total_angle;
    uint32_t angle_lost_count;
    uint32_t last_feedback_time;
} MOTOR_RM_t;

/*CAN管理器，管理一个CAN总线上所有的电机*/
typedef struct {
    BSP_CAN_t can;
    MOTOR_RM_MsgOutput_t output_msg;
    /* 缓存后统一发送：记录哪些 RM 控制帧组已写入缓存。 */
    uint8_t pending_tx_groups;
    MOTOR_RM_t *motors[MOTOR_RM_MAX_MOTORS];
    uint8_t motor_count;
} MOTOR_RM_CANManager_t;

/* Exported functions prototypes -------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* 原生 C 驱动接口：注册、反馈更新、兼容输出、直接组帧发送与状态控制。 */
/* -------------------------------------------------------------------------- */

/**
 * @brief 注册一个RM电机
 * @param param 电机参数
 * @return 
 */
int8_t MOTOR_RM_Register(MOTOR_RM_Param_t *param);

/**
 * @brief 更新指定电机数据
 * @param param 电机参数
 * @return 
 */
int8_t MOTOR_RM_Update(MOTOR_RM_Param_t *param);

/**
 * @brief 设置一个电机的归一化输出（底层兼容接口）
 * @param param 电机参数
 * @param value 输出值，范围[-1.0, 1.0]
 * @note 该接口只写入 CAN 管理器的输出缓存；随后调用 MOTOR_RM_Ctrl()
 *       或 MOTOR_RM_FlushGroup()/MOTOR_RM_FlushCAN() 发送对应控制帧组。
 * @return 
 */
int8_t MOTOR_RM_SetOutput(MOTOR_RM_Param_t *param, float value);

/**
 * @brief 发送控制命令到电机，注意一个CAN可以控制多个电机，所以只需要发送一次即可
 * @param param 电机参数
 * @note 兼容旧代码的立即发送接口。多个 RM 电机同周期控制时，推荐先调用
 *       MOTOR_RM_SetOutput()/MOTOR_RM_SetTorqueCurrent() 写完同组缓存，再调用
 *       MOTOR_RM_FlushGroup()/MOTOR_RM_FlushCAN() 统一发送。
 * @return 
 */
int8_t MOTOR_RM_Ctrl(MOTOR_RM_Param_t *param);

/**
 * @brief 按 C 驱动缓存发送方式，发送指定电机所属 RM 控制帧组的缓存命令。
 * @param param 电机参数，用于定位 CAN 与控制帧组
 * @return 设备状态码
 */
int8_t MOTOR_RM_FlushGroup(MOTOR_RM_Param_t *param);

/**
 * @brief 按 C 驱动缓存发送方式，发送指定 CAN 上所有待发送的 RM 控制帧组。
 * @param can CAN 总线
 * @return 设备状态码
 */
int8_t MOTOR_RM_FlushCAN(BSP_CAN_t can);

/**
 * @brief 按 C 驱动缓存发送方式，发送所有 CAN 上所有待发送的 RM 控制帧组。
 * @return 设备状态码
 */
int8_t MOTOR_RM_FlushAll(void);

/**
 * @brief 获取指定电机的实例指针
 * @param param 电机参数
 * @return 
 */
MOTOR_RM_t* MOTOR_RM_GetMotor(MOTOR_RM_Param_t *param);

/**
 * @brief 使电机松弛（设置输出为0）
 * @param param 
 * @return 
 */
int8_t MOTOR_RM_Relax(MOTOR_RM_Param_t *param);

/**
 * @brief 使电机离线（设置在线状态为false）
 * @param param 
 * @return 
 */
int8_t MOTOR_RM_Offine(MOTOR_RM_Param_t *param);

/**
 * @brief 
 * @param  
 * @return 
 */
int8_t MOTOR_RM_UpdateAll(void);

/**
 * @brief 设置 RM 电机的目标电流
 *
 * @param[in] param 电机参数
 * @param[in] current_a 目标电流，单位 A
 * @return 成功返回 DEVICE_OK，参数或设备状态异常时返回对应错误码
 */
int8_t MOTOR_RM_SetTorqueCurrent(MOTOR_RM_Param_t *param, float current_a);

#ifdef __cplusplus
}
#endif
