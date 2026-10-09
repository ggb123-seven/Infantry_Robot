#include "device/bmi088.h"

#include <stddef.h>
#include <string.h>

#include "bsp/imu.h"

/* BMI088 SPI 读命令使用地址最高位，读事务的填充字节不参与数据解析 */
#define BMI088_SPI_TIMEOUT_MS (10U)
#define BMI088_RESET_DELAY_MS (80U)
#define BMI088_CONFIG_DELAY_MS (1U)
#define BMI088_ACCEL_READ_DUMMY (0x55U)
#define BMI088_GYRO_READ_DUMMY (0x55U)
#define BMI088_ACCEL_DATA_LENGTH (6U)
#define BMI088_GYRO_DATA_LENGTH (6U)
#define BMI088_TEMP_DATA_LENGTH (2U)

static BMI088_Status_t bmi088_status = BMI088_ERR_NOT_INITIALIZED;
static bool bmi088_initialized;

static BMI088_Status_t BMI088_MapBspStatus(int8_t status)
{
    if (status == BSP_OK)
    {
        return BMI088_OK;
    }
    return BMI088_ERR_COMMUNICATION;
}

static BMI088_Status_t BMI088_Transfer(uint8_t tx_data, uint8_t *rx_data)
{
    return BMI088_MapBspStatus(BSP_IMU_SPI_TransferByte(tx_data, rx_data, BMI088_SPI_TIMEOUT_MS));
}

static BMI088_Status_t BMI088_Select(BSP_IMU_Device_t device)
{
    return BMI088_MapBspStatus(BSP_IMU_SetSelected(device, true));
}

static BMI088_Status_t BMI088_Deselect(BSP_IMU_Device_t device)
{
    return BMI088_MapBspStatus(BSP_IMU_SetSelected(device, false));
}

static BMI088_Status_t BMI088_DelayMs(uint32_t delay_ms)
{
    return BMI088_MapBspStatus(BSP_IMU_DelayMs(delay_ms));
}

static BMI088_Status_t BMI088_WriteRegister(BSP_IMU_Device_t device, uint8_t reg, uint8_t value)
{
    uint8_t rx_data = 0U;
    BMI088_Status_t status = BMI088_Select(device);
    if (status != BMI088_OK)
    {
        return status;
    }

    status = BMI088_Transfer(reg, &rx_data);
    if (status == BMI088_OK)
    {
        status = BMI088_Transfer(value, &rx_data);
    }
    BMI088_Deselect(device);
    return status;
}

static BMI088_Status_t BMI088_ReadRegisters(BSP_IMU_Device_t device, uint8_t reg, uint8_t *data, uint8_t length)
{
    if (data == NULL || length == 0U)
    {
        return BMI088_ERR_NULL;
    }

    uint8_t rx_data = 0U;
    BMI088_Status_t status = BMI088_Select(device);
    if (status != BMI088_OK)
    {
        return status;
    }

    status = BMI088_Transfer(reg | 0x80U, &rx_data);
    if (status == BMI088_OK && device == BSP_IMU_ACCEL)
    {
        status = BMI088_Transfer(BMI088_ACCEL_READ_DUMMY, &rx_data);
    }
    for (uint8_t index = 0U; status == BMI088_OK && index < length; index++)
    {
        const uint8_t dummy = device == BSP_IMU_ACCEL ? BMI088_ACCEL_READ_DUMMY : BMI088_GYRO_READ_DUMMY;
        status = BMI088_Transfer(dummy, &data[index]);
    }
    BMI088_Deselect(device);
    return status;
}

static BMI088_Status_t BMI088_ReadRegister(BSP_IMU_Device_t device, uint8_t reg, uint8_t *data)
{
    return BMI088_ReadRegisters(device, reg, data, 1U);
}

static BMI088_Status_t BMI088_CheckRegister(BSP_IMU_Device_t device, uint8_t reg, uint8_t expected)
{
    uint8_t actual = 0U;
    const BMI088_Status_t status = BMI088_ReadRegister(device, reg, &actual);
    if (status != BMI088_OK)
    {
        return status;
    }
    return actual == expected ? BMI088_OK : BMI088_ERR_CONFIGURATION;
}

static BMI088_Status_t BMI088_ResetAccelerometer(void)
{
    BMI088_Status_t status = BMI088_WriteRegister(BSP_IMU_ACCEL, BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    if (status != BMI088_OK)
    {
        return status;
    }
    return BMI088_DelayMs(BMI088_RESET_DELAY_MS);
}

static BMI088_Status_t BMI088_ResetGyroscope(void)
{
    BMI088_Status_t status = BMI088_WriteRegister(BSP_IMU_GYRO, BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    if (status != BMI088_OK)
    {
        return status;
    }
    return BMI088_DelayMs(BMI088_RESET_DELAY_MS);
}

static BMI088_Status_t BMI088_InitAccelerometer(void)
{
    // 先确认加速度计芯片 ID，避免对不存在的设备执行复位
    BMI088_Status_t status = BMI088_CheckRegister(BSP_IMU_ACCEL, BMI088_ACC_CHIP_ID, BMI088_ACC_CHIP_ID_VALUE);
    if (status != BMI088_OK)
    {
        return status == BMI088_ERR_CONFIGURATION ? BMI088_ERR_NO_SENSOR : status;
    }

    // 复位后等待芯片完成内部启动，再次核对芯片 ID
    status = BMI088_ResetAccelerometer();
    if (status != BMI088_OK)
    {
        return status;
    }
    status = BMI088_CheckRegister(BSP_IMU_ACCEL, BMI088_ACC_CHIP_ID, BMI088_ACC_CHIP_ID_VALUE);
    if (status != BMI088_OK)
    {
        return status == BMI088_ERR_CONFIGURATION ? BMI088_ERR_NO_SENSOR : status;
    }

    // 按本机参考工程中的顺序写入量程、带宽和数据就绪中断配置
    const uint8_t registers[][2] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE},
        {BMI088_ACC_CONF, BMI088_ACC_CONF_VALUE},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G},
        {BMI088_ACC_INT1_IO_CTRL, BMI088_ACC_INT1_IO_CTRL_VALUE},
        {BMI088_ACC_INT_MAP_DATA, BMI088_ACC_INT_MAP_DATA_VALUE},
    };
    const uint8_t register_count = (uint8_t)(sizeof(registers) / sizeof(registers[0]));
    for (uint8_t index = 0U; index < register_count; index++)
    {
        status = BMI088_WriteRegister(BSP_IMU_ACCEL, registers[index][0], registers[index][1]);
        if (status != BMI088_OK)
        {
            return status;
        }
        status = BMI088_DelayMs(BMI088_CONFIG_DELAY_MS);
        if (status != BMI088_OK)
        {
            return status;
        }
        status = BMI088_CheckRegister(BSP_IMU_ACCEL, registers[index][0], registers[index][1]);
        if (status != BMI088_OK)
        {
            return status;
        }
        status = BMI088_DelayMs(BMI088_CONFIG_DELAY_MS);
        if (status != BMI088_OK)
        {
            return status;
        }
    }
    return BMI088_OK;
}

static BMI088_Status_t BMI088_InitGyroscope(void)
{
    // 先确认陀螺仪芯片 ID，避免对不存在的设备执行复位
    BMI088_Status_t status = BMI088_CheckRegister(BSP_IMU_GYRO, BMI088_GYRO_CHIP_ID, BMI088_GYRO_CHIP_ID_VALUE);
    if (status != BMI088_OK)
    {
        return status == BMI088_ERR_CONFIGURATION ? BMI088_ERR_NO_SENSOR : status;
    }

    // 复位后等待芯片完成内部启动，再次核对芯片 ID
    status = BMI088_ResetGyroscope();
    if (status != BMI088_OK)
    {
        return status;
    }
    status = BMI088_CheckRegister(BSP_IMU_GYRO, BMI088_GYRO_CHIP_ID, BMI088_GYRO_CHIP_ID_VALUE);
    if (status != BMI088_OK)
    {
        return status == BMI088_ERR_CONFIGURATION ? BMI088_ERR_NO_SENSOR : status;
    }

    // 按本机参考工程中的顺序写入量程、带宽和数据就绪中断配置
    const uint8_t registers[][2] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_RANGE_2000DPS},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_BANDWIDTH_VALUE},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE},
        {BMI088_GYRO_CTRL, BMI088_GYRO_DATA_READY_ENABLE},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_PUSH_PULL_LOW},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_INT3_DATA_READY},
    };
    const uint8_t register_count = (uint8_t)(sizeof(registers) / sizeof(registers[0]));
    for (uint8_t index = 0U; index < register_count; index++)
    {
        status = BMI088_WriteRegister(BSP_IMU_GYRO, registers[index][0], registers[index][1]);
        if (status != BMI088_OK)
        {
            return status;
        }
        status = BMI088_DelayMs(BMI088_CONFIG_DELAY_MS);
        if (status != BMI088_OK)
        {
            return status;
        }
        status = BMI088_CheckRegister(BSP_IMU_GYRO, registers[index][0], registers[index][1]);
        if (status != BMI088_OK)
        {
            return status;
        }
        status = BMI088_DelayMs(BMI088_CONFIG_DELAY_MS);
        if (status != BMI088_OK)
        {
            return status;
        }
    }
    return BMI088_OK;
}

static int16_t BMI088_CombineInt16(uint8_t low, uint8_t high)
{
    return (int16_t)(((uint16_t)high << 8U) | low);
}

static int16_t BMI088_ParseTemperature(const uint8_t data[BMI088_TEMP_DATA_LENGTH])
{
    int16_t raw = (int16_t)(((uint16_t)data[0] << 3U) | ((uint16_t)data[1] >> 5U));
    if (raw > 1023)
    {
        raw = (int16_t)(raw - 2048);
    }
    return raw;
}

BMI088_Status_t BMI088_Init(void)
{
    bmi088_initialized = false;
    bmi088_status = BMI088_ERR_NOT_INITIALIZED;

    BMI088_Status_t status = BMI088_InitAccelerometer();
    if (status == BMI088_OK)
    {
        status = BMI088_InitGyroscope();
    }
    if (status != BMI088_OK)
    {
        bmi088_status = status;
        return status;
    }

    bmi088_initialized = true;
    bmi088_status = BMI088_OK;
    return BMI088_OK;
}

BMI088_Status_t BMI088_Read(BMI088_Data_t *data)
{
    if (data == NULL)
    {
        bmi088_status = BMI088_ERR_NULL;
        return BMI088_ERR_NULL;
    }
    memset(data, 0, sizeof(*data));
    if (!bmi088_initialized)
    {
        bmi088_status = BMI088_ERR_NOT_INITIALIZED;
        return BMI088_ERR_NOT_INITIALIZED;
    }

    // 分别读取加速度、角速度和温度，任一 SPI 事务失败都不发布部分数据
    uint8_t accel_data[BMI088_ACCEL_DATA_LENGTH];
    uint8_t gyro_data[BMI088_GYRO_DATA_LENGTH];
    uint8_t temperature_data[BMI088_TEMP_DATA_LENGTH];
    BMI088_Status_t status = BMI088_ReadRegisters(BSP_IMU_ACCEL, BMI088_ACC_XOUT_L, accel_data, sizeof(accel_data));
    if (status == BMI088_OK)
    {
        status = BMI088_ReadRegisters(BSP_IMU_GYRO, BMI088_GYRO_XOUT_L, gyro_data, sizeof(gyro_data));
    }
    if (status == BMI088_OK)
    {
        status = BMI088_ReadRegisters(BSP_IMU_ACCEL, BMI088_ACC_TEMP_M, temperature_data, sizeof(temperature_data));
    }
    if (status != BMI088_OK)
    {
        bmi088_status = status;
        return status;
    }

    // 将原始二进制补码转换为 SI 单位，供上层 EKF 直接使用
    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
        const int16_t accel_raw = BMI088_CombineInt16(accel_data[axis * 2U], accel_data[axis * 2U + 1U]);
        const int16_t gyro_raw = BMI088_CombineInt16(gyro_data[axis * 2U], gyro_data[axis * 2U + 1U]);
        data->accel_mps2[axis] = (float)accel_raw * BMI088_ACC_SENSITIVITY_G_PER_LSB * BMI088_GRAVITY_MPS2;
        data->gyro_rad_s[axis] = (float)gyro_raw * BMI088_GYRO_SENSITIVITY_RAD_PER_S;
    }
    data->temperature_c = (float)BMI088_ParseTemperature(temperature_data) * BMI088_TEMPERATURE_SENSITIVITY_C +
                          BMI088_TEMPERATURE_OFFSET_C;
    data->valid = true;
    bmi088_status = BMI088_OK;
    return BMI088_OK;
}

BMI088_Status_t BMI088_GetStatus(void)
{
    return bmi088_status;
}

bool BMI088_IsReady(void)
{
    return bmi088_initialized && bmi088_status == BMI088_OK;
}
