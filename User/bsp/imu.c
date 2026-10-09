#include "bsp/imu.h"

#include <stddef.h>

#include "main.h"
#include "spi.h"
#include "tim.h"

#include "bsp/time.h"

/* BMI088 加速度计和陀螺仪片选均为低有效，释放状态必须保持高电平 */
static const uint16_t imu_cs_pins[BSP_IMU_DEVICE_NUM] =
{
    CS1_ACCEL_Pin,
    CS1_GYRO_Pin,
};

static GPIO_TypeDef *const imu_cs_ports[BSP_IMU_DEVICE_NUM] =
{
    CS1_ACCEL_GPIO_Port,
    CS1_GYRO_GPIO_Port,
};

static bool imu_heat_started;

static bool BSP_IMU_IsValidDevice(BSP_IMU_Device_t device)
{
    return device < BSP_IMU_DEVICE_NUM;
}

/**
 * @brief 设置 BMI088 设备片选电平
 *
 * @param[in] device BMI088 加速度计或陀螺仪
 * @param[in] selected 为 true 时拉低片选，为 false 时释放片选
 * @return 成功返回 BSP_OK，设备参数非法返回 BSP_ERR
 */
int8_t BSP_IMU_SetSelected(BSP_IMU_Device_t device, bool selected)
{
    if (!BSP_IMU_IsValidDevice(device))
    {
        return BSP_ERR;
    }

    HAL_GPIO_WritePin(imu_cs_ports[device], imu_cs_pins[device], selected ? GPIO_PIN_RESET : GPIO_PIN_SET);
    return BSP_OK;
}

/**
 * @brief 通过 SPI1 完成一个字节的全双工事务
 *
 * @param[in] tx_data 待发送字节
 * @param[out] rx_data 接收字节存储地址
 * @param[in] timeout_ms HAL 事务超时时间，单位毫秒
 * @return 成功返回 BSP_OK，参数为空返回 BSP_ERR_NULL，超时返回 BSP_ERR_TIMEOUT，其余 HAL 错误返回 BSP_ERR
 */
int8_t BSP_IMU_SPI_TransferByte(uint8_t tx_data, uint8_t *rx_data, uint32_t timeout_ms)
{
    if (rx_data == NULL)
    {
        return BSP_ERR_NULL;
    }

    const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, &tx_data, rx_data, 1U, timeout_ms);
    if (status == HAL_OK)
    {
        return BSP_OK;
    }
    if (status == HAL_TIMEOUT)
    {
        return BSP_ERR_TIMEOUT;
    }
    return BSP_ERR;
}

/**
 * @brief 提供 BMI088 初始化所需的毫秒级延时
 *
 * @param[in] delay_ms 延时时间，单位毫秒
 * @return 成功返回 BSP_OK，当前系统状态不允许阻塞延时返回 BSP_ERR
 */
int8_t BSP_IMU_DelayMs(uint32_t delay_ms)
{
    return BSP_TIME_Delay_ms(delay_ms);
}

/**
 * @brief 提供 BMI088 初始化所需的微秒级延时
 *
 * @param[in] delay_us 延时时间，单位微秒
 * @return 成功返回 BSP_OK
 */
int8_t BSP_IMU_DelayUs(uint32_t delay_us)
{
    return BSP_TIME_Delay_us(delay_us);
}

/**
 * @brief 启动 BMI088 加热 PWM
 *
 * @return 成功返回 BSP_OK，HAL 启动失败返回 BSP_ERR
 */
int8_t BSP_IMU_HeatStart(void)
{
    if (HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1) != HAL_OK)
    {
        imu_heat_started = false;
        return BSP_ERR;
    }

    imu_heat_started = true;
    return BSP_OK;
}

/**
 * @brief 停止 BMI088 加热 PWM 并清零比较值
 *
 * @return 成功返回 BSP_OK，HAL 停止失败返回 BSP_ERR
 */
int8_t BSP_IMU_HeatStop(void)
{
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0U);
    if (HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1) != HAL_OK)
    {
        imu_heat_started = false;
        return BSP_ERR;
    }

    imu_heat_started = false;
    return BSP_OK;
}

/**
 * @brief 设置 BMI088 加热 PWM 占空比
 *
 * @param[in] duty_permille 占空比千分比，范围 0~1000
 * @return 成功返回 BSP_OK，越界时关闭 PWM 并返回 BSP_ERR
 */
int8_t BSP_IMU_HeatSetDuty(uint16_t duty_permille)
{
    if (duty_permille > 1000U)
    {
        BSP_IMU_HeatStop();
        return BSP_ERR;
    }
    if (!imu_heat_started)
    {
        return BSP_ERR;
    }

    const uint32_t period_counts = (uint32_t)__HAL_TIM_GET_AUTORELOAD(&htim10) + 1U;
    uint32_t compare = (period_counts * duty_permille) / 1000U;
    if (compare > (uint32_t)__HAL_TIM_GET_AUTORELOAD(&htim10))
    {
        compare = (uint32_t)__HAL_TIM_GET_AUTORELOAD(&htim10);
    }
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, compare);
    return BSP_OK;
}
