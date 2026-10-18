#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

/* BMI088 加速度计寄存器与配置值，移植自本机参考工程的 BMI088reg.h */
#define BMI088_ACC_CHIP_ID (0x00U)
#define BMI088_ACC_CHIP_ID_VALUE (0x1EU)
#define BMI088_ACC_XOUT_L (0x12U)
#define BMI088_ACC_TEMP_M (0x22U)
#define BMI088_ACC_CONF (0x40U)
#define BMI088_ACC_CONF_VALUE (0xABU)
#define BMI088_ACC_RANGE (0x41U)
#define BMI088_ACC_RANGE_6G (0x01U)
#define BMI088_ACC_INT1_IO_CTRL (0x53U)
#define BMI088_ACC_INT1_IO_CTRL_VALUE (0x08U)
#define BMI088_ACC_INT_MAP_DATA (0x58U)
#define BMI088_ACC_INT_MAP_DATA_VALUE (0x04U)
#define BMI088_ACC_PWR_CONF (0x7CU)
#define BMI088_ACC_PWR_ACTIVE (0x00U)
#define BMI088_ACC_PWR_CTRL (0x7DU)
#define BMI088_ACC_ENABLE (0x04U)
#define BMI088_ACC_SOFTRESET (0x7EU)
#define BMI088_ACC_SOFTRESET_VALUE (0xB6U)

/* BMI088 陀螺仪寄存器与配置值，移植自本机参考工程的 BMI088reg.h */
#define BMI088_GYRO_CHIP_ID (0x00U)
#define BMI088_GYRO_CHIP_ID_VALUE (0x0FU)
#define BMI088_GYRO_XOUT_L (0x02U)
#define BMI088_GYRO_RANGE (0x0FU)
#define BMI088_GYRO_RANGE_2000DPS (0x00U)
#define BMI088_GYRO_BANDWIDTH (0x10U)
#define BMI088_GYRO_BANDWIDTH_VALUE (0x81U)
#define BMI088_GYRO_LPM1 (0x11U)
#define BMI088_GYRO_NORMAL_MODE (0x00U)
#define BMI088_GYRO_CTRL (0x15U)
#define BMI088_GYRO_DATA_READY_ENABLE (0x80U)
#define BMI088_GYRO_INT3_INT4_IO_CONF (0x16U)
#define BMI088_GYRO_INT3_PUSH_PULL_LOW (0x00U)
#define BMI088_GYRO_INT3_INT4_IO_MAP (0x18U)
#define BMI088_GYRO_INT3_DATA_READY (0x01U)
#define BMI088_GYRO_SOFTRESET (0x14U)
#define BMI088_GYRO_SOFTRESET_VALUE (0xB6U)

/* 量程灵敏度和温度参数来自本机参考工程的 BMI088driver.h */
#define BMI088_ACC_SENSITIVITY_MPS2_PER_LSB (0.00179443359375F)
#define BMI088_GYRO_SENSITIVITY_RAD_PER_S (0.0010652644360316953F)
#define BMI088_TEMPERATURE_SENSITIVITY_C (0.125F)
#define BMI088_TEMPERATURE_OFFSET_C (23.0F)

/**
 * @brief BMI088 设备驱动状态
 */
typedef enum
{
    BMI088_OK = 0,
    BMI088_ERR = -1,
    BMI088_ERR_NULL = -2,
    BMI088_ERR_NOT_INITIALIZED = -3,
    BMI088_ERR_NO_SENSOR = -4,
    BMI088_ERR_COMMUNICATION = -5,
    BMI088_ERR_CONFIGURATION = -6,
} BMI088_Status_t;

/*
 * BMI088 已换算数据：加速度单位为 m/s²，角速度单位为 rad/s，温度单位为摄氏度
 * valid 仅在最近一次完整读取成功时为 true，读取失败时三轴数据和温度均清零
 */
typedef struct
{
    float accel_mps2[3];
    float gyro_rad_s[3];
    float temperature_c;
    bool valid;
} BMI088_Data_t;

/**
 * @brief 初始化 BMI088 加速度计和陀螺仪
 *
 * @return 初始化成功返回 BMI088_OK，芯片 ID 不匹配返回 BMI088_ERR_NO_SENSOR，寄存器配置校验失败返回
 *         BMI088_ERR_CONFIGURATION，SPI 或延时失败返回 BMI088_ERR_COMMUNICATION
 */
BMI088_Status_t BMI088_Init(void);

/**
 * @brief 读取 BMI088 当前加速度、角速度和温度
 *
 * @param[out] data 输出数据，读取失败时由驱动清零并将 valid 置为 false
 * @return 读取成功返回 BMI088_OK，参数为空返回 BMI088_ERR_NULL，未初始化返回 BMI088_ERR_NOT_INITIALIZED，
 *         SPI 事务失败返回 BMI088_ERR_COMMUNICATION
 */
BMI088_Status_t BMI088_Read(BMI088_Data_t *data);

/**
 * @brief 获取 BMI088 最近一次操作状态
 *
 * @return 最近一次初始化或读取操作的状态
 */
BMI088_Status_t BMI088_GetStatus(void);

/**
 * @brief 查询 BMI088 是否完成初始化且最近一次读取有效
 *
 * @return 最近一次状态为 BMI088_OK 时返回 true，否则返回 false
 */
bool BMI088_IsReady(void);

#ifdef __cplusplus
}
#endif
