#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

#include "bsp/bsp.h"

/*
 * BMI088 板级资源：SPI1 使用软件片选，温控 PWM 使用 TIM10_CH1，周期为 2000 个计数
 * 占空比接口统一采用千分比，范围 0~1000，对应 0%~100%
 */
typedef enum
{
    BSP_IMU_ACCEL = 0,
    BSP_IMU_GYRO,
    BSP_IMU_DEVICE_NUM,
} BSP_IMU_Device_t;

/**
 * @brief 设置 BMI088 设备片选电平
 *
 * @param[in] device BMI088 加速度计或陀螺仪
 * @param[in] selected 为 true 时拉低片选，为 false 时释放片选
 * @return 成功返回 BSP_OK，设备参数非法返回 BSP_ERR
 */
int8_t BSP_IMU_SetSelected(BSP_IMU_Device_t device, bool selected);

/**
 * @brief 通过 SPI1 完成一个字节的全双工事务
 *
 * @param[in] tx_data 待发送字节
 * @param[out] rx_data 接收字节存储地址
 * @param[in] timeout_ms HAL 事务超时时间，单位毫秒
 * @return 成功返回 BSP_OK，参数为空返回 BSP_ERR_NULL，超时返回 BSP_ERR_TIMEOUT，其余 HAL 错误返回 BSP_ERR
 */
int8_t BSP_IMU_SPI_TransferByte(uint8_t tx_data, uint8_t *rx_data, uint32_t timeout_ms);

/**
 * @brief 提供 BMI088 初始化所需的毫秒级延时
 *
 * @param[in] delay_ms 延时时间，单位毫秒
 * @return 成功返回 BSP_OK，当前系统状态不允许阻塞延时返回 BSP_ERR
 */
int8_t BSP_IMU_DelayMs(uint32_t delay_ms);

/**
 * @brief 提供 BMI088 初始化所需的微秒级延时
 *
 * @param[in] delay_us 延时时间，单位微秒
 * @return 成功返回 BSP_OK
 */
int8_t BSP_IMU_DelayUs(uint32_t delay_us);

/**
 * @brief 启动 BMI088 加热 PWM
 *
 * @return 成功返回 BSP_OK，HAL 启动失败返回 BSP_ERR
 */
int8_t BSP_IMU_HeatStart(void);

/**
 * @brief 停止 BMI088 加热 PWM 并清零比较值
 *
 * @return 成功返回 BSP_OK，HAL 停止失败返回 BSP_ERR
 */
int8_t BSP_IMU_HeatStop(void);

/**
 * @brief 设置 BMI088 加热 PWM 占空比
 *
 * @param[in] duty_permille 占空比千分比，范围 0~1000
 * @return 成功返回 BSP_OK，越界时关闭 PWM 并返回 BSP_ERR
 */
int8_t BSP_IMU_HeatSetDuty(uint16_t duty_permille);

#ifdef __cplusplus
}
#endif
