#include "device/bmi088.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "bsp/imu.h"
#include "bsp/time.h"

/* BMI088 SPI 读命令使用地址最高位，读事务的填充字节不参与数据解析 */
#define BMI088_SPI_TIMEOUT_MS (10U)
#define BMI088_RESET_DELAY_MS (80U)
#define BMI088_CONFIG_DELAY_MS (1U)
#define BMI088_CHIP_ID_READ_DELAY_MS (1U)
#define BMI088_ACCEL_READ_DUMMY (0x55U)
#define BMI088_GYRO_READ_DUMMY (0x55U)
#define BMI088_ACCEL_DATA_LENGTH (6U)
#define BMI088_GYRO_DATA_LENGTH (8U)
#define BMI088_TEMP_DATA_LENGTH (2U)

/* 校准参数与超时回退值均来自参考工程 BMI088driver.c 和 BMI088driver.h 的默认机号配置 */
#define BMI088_CALIBRATION_SAMPLE_COUNT (6000U)
#define BMI088_CALIBRATION_START_DELAY_US (5000U)
#define BMI088_CALIBRATION_SAMPLE_DELAY_US (500U)
#define BMI088_CALIBRATION_TIMEOUT_US (10000000ULL)
#define BMI088_CALIBRATION_GRAVITY_DIFF_LIMIT (0.5F)
#define BMI088_CALIBRATION_GYRO_DIFF_LIMIT (0.15F)
#define BMI088_CALIBRATION_GRAVITY_ERROR_LIMIT (0.5F)
#define BMI088_CALIBRATION_GYRO_OFFSET_LIMIT (0.01F)
#define BMI088_CALIBRATION_GRAVITY_REFERENCE (9.8F)
#define BMI088_CALIBRATION_ACCEL_TARGET (9.81F)
#define BMI088_FALLBACK_GYRO_OFFSET_X (0.00247530174F)
#define BMI088_FALLBACK_GYRO_OFFSET_Y (0.000393082853F)
#define BMI088_FALLBACK_GYRO_OFFSET_Z (0.000393082853F)
#define BMI088_FALLBACK_GRAVITY_NORM (9.69293118F)
#define BMI088_FALLBACK_TEMPERATURE_C (40.0F)

static BMI088_Status_t bmi088_status = BMI088_ERR_NOT_INITIALIZED;
static bool bmi088_initialized;

/* 以下全局量对应参考工程的校准结果和运动检测量，可直接在 Ozone 中观察 */
float BMI088_GyroOffset[3];
float BMI088_GravityNorm;
float BMI088_AccelScale = 1.0F;
float BMI088_TemperatureWhenCalibratedC;
float BMI088_GyroDiff[3];
float BMI088_GravityNormDiff;
int16_t BMI088_CalibrationCount;

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

/**
 * @brief 连续读取两次芯片 ID，并使用第二次结果完成校验
 *
 * @param[in] device BMI088 加速度计或陀螺仪
 * @param[in] reg 芯片 ID 寄存器地址
 * @param[in] expected 期望的芯片 ID
 * @return 校验通过返回 BMI088_OK，读取失败或 ID 不匹配时返回对应错误状态
 */
static BMI088_Status_t BMI088_CheckChipId(BSP_IMU_Device_t device, uint8_t reg, uint8_t expected)
{
    // 首次读取用于建立稳定的 SPI 通信
    uint8_t actual = 0U;
    BMI088_Status_t status = BMI088_ReadRegister(device, reg, &actual);
    if (status != BMI088_OK)
    {
        return status;
    }
    status = BMI088_DelayMs(BMI088_CHIP_ID_READ_DELAY_MS);
    if (status != BMI088_OK)
    {
        return status;
    }

    // 第二次读取用于正式校验芯片 ID
    status = BMI088_ReadRegister(device, reg, &actual);
    if (status != BMI088_OK)
    {
        return status;
    }
    status = BMI088_DelayMs(BMI088_CHIP_ID_READ_DELAY_MS);
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
    BMI088_Status_t status = BMI088_CheckChipId(BSP_IMU_ACCEL, BMI088_ACC_CHIP_ID, BMI088_ACC_CHIP_ID_VALUE);
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
    status = BMI088_CheckChipId(BSP_IMU_ACCEL, BMI088_ACC_CHIP_ID, BMI088_ACC_CHIP_ID_VALUE);
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
    BMI088_Status_t status = BMI088_CheckChipId(BSP_IMU_GYRO, BMI088_GYRO_CHIP_ID, BMI088_GYRO_CHIP_ID_VALUE);
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
    status = BMI088_CheckChipId(BSP_IMU_GYRO, BMI088_GYRO_CHIP_ID, BMI088_GYRO_CHIP_ID_VALUE);
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

/**
 * @brief 阻塞采样 BMI088，计算三轴陀螺仪零偏和加速度计比例系数
 *
 * @return 校准或超时回退完成返回 BMI088_OK，通信失败或陀螺仪芯片 ID 异常时返回对应错误状态
 */
static BMI088_Status_t BMI088_Calibrate(void)
{
    uint8_t accel_data[BMI088_ACCEL_DATA_LENGTH];
    uint8_t gyro_data[BMI088_GYRO_DATA_LENGTH];
    uint8_t temperature_data[BMI088_TEMP_DATA_LENGTH];
    float accel_mps2[3];
    float gyro_rad_s[3];
    float gyro_max[3];
    float gyro_min[3];
    float gravity_norm_max = 0.0F;
    float gravity_norm_min = 0.0F;
    const uint64_t start_time_us = BSP_TIME_Get_us();

    BMI088_CalibrationCount = 0;
    do
    {
        // 超过参考工程规定的校准时间后使用其默认机号标定值
        if (BSP_TIME_Get_us() - start_time_us > BMI088_CALIBRATION_TIMEOUT_US)
        {
            BMI088_GyroOffset[0] = BMI088_FALLBACK_GYRO_OFFSET_X;
            BMI088_GyroOffset[1] = BMI088_FALLBACK_GYRO_OFFSET_Y;
            BMI088_GyroOffset[2] = BMI088_FALLBACK_GYRO_OFFSET_Z;
            BMI088_GravityNorm = BMI088_FALLBACK_GRAVITY_NORM;
            BMI088_TemperatureWhenCalibratedC = BMI088_FALLBACK_TEMPERATURE_C;
            break;
        }

        // 每次重新校准前按参考工程阻塞等待传感器数据稳定
        BMI088_Status_t status = BMI088_MapBspStatus(BSP_IMU_DelayUs(BMI088_CALIBRATION_START_DELAY_US));
        if (status != BMI088_OK)
        {
            return status;
        }
        BMI088_GravityNorm = 0.0F;
        BMI088_GyroOffset[0] = 0.0F;
        BMI088_GyroOffset[1] = 0.0F;
        BMI088_GyroOffset[2] = 0.0F;

        for (uint16_t sample = 0U; sample < BMI088_CALIBRATION_SAMPLE_COUNT; sample++)
        {
            // 采集加速度并累计重力模长
            status = BMI088_ReadRegisters(BSP_IMU_ACCEL, BMI088_ACC_XOUT_L, accel_data, sizeof(accel_data));
            if (status != BMI088_OK)
            {
                return status;
            }
            for (uint8_t axis = 0U; axis < 3U; axis++)
            {
                const int16_t raw = BMI088_CombineInt16(accel_data[axis * 2U], accel_data[axis * 2U + 1U]);
                accel_mps2[axis] = (float)raw * BMI088_ACC_SENSITIVITY_MPS2_PER_LSB;
            }
            const float gravity_norm = sqrtf(accel_mps2[0] * accel_mps2[0] + accel_mps2[1] * accel_mps2[1] +
                                             accel_mps2[2] * accel_mps2[2]);
            BMI088_GravityNorm += gravity_norm;

            // 从芯片 ID 开始读取陀螺仪帧，校验设备后累计三轴零偏
            status = BMI088_ReadRegisters(BSP_IMU_GYRO, BMI088_GYRO_CHIP_ID, gyro_data, sizeof(gyro_data));
            if (status != BMI088_OK)
            {
                return status;
            }
            if (gyro_data[0] != BMI088_GYRO_CHIP_ID_VALUE)
            {
                return BMI088_ERR_NO_SENSOR;
            }
            for (uint8_t axis = 0U; axis < 3U; axis++)
            {
                const int16_t raw = BMI088_CombineInt16(gyro_data[axis * 2U + 2U], gyro_data[axis * 2U + 3U]);
                gyro_rad_s[axis] = (float)raw * BMI088_GYRO_SENSITIVITY_RAD_PER_S;
                BMI088_GyroOffset[axis] += gyro_rad_s[axis];
            }

            // 记录采样极差，检测校准期间的外界运动
            if (sample == 0U)
            {
                gravity_norm_max = gravity_norm;
                gravity_norm_min = gravity_norm;
                for (uint8_t axis = 0U; axis < 3U; axis++)
                {
                    gyro_max[axis] = gyro_rad_s[axis];
                    gyro_min[axis] = gyro_rad_s[axis];
                }
            }
            else
            {
                if (gravity_norm > gravity_norm_max)
                {
                    gravity_norm_max = gravity_norm;
                }
                if (gravity_norm < gravity_norm_min)
                {
                    gravity_norm_min = gravity_norm;
                }
                for (uint8_t axis = 0U; axis < 3U; axis++)
                {
                    if (gyro_rad_s[axis] > gyro_max[axis])
                    {
                        gyro_max[axis] = gyro_rad_s[axis];
                    }
                    if (gyro_rad_s[axis] < gyro_min[axis])
                    {
                        gyro_min[axis] = gyro_rad_s[axis];
                    }
                }
            }

            BMI088_GravityNormDiff = gravity_norm_max - gravity_norm_min;
            for (uint8_t axis = 0U; axis < 3U; axis++)
            {
                BMI088_GyroDiff[axis] = gyro_max[axis] - gyro_min[axis];
            }
            if (BMI088_GravityNormDiff > BMI088_CALIBRATION_GRAVITY_DIFF_LIMIT ||
                BMI088_GyroDiff[0] > BMI088_CALIBRATION_GYRO_DIFF_LIMIT ||
                BMI088_GyroDiff[1] > BMI088_CALIBRATION_GYRO_DIFF_LIMIT ||
                BMI088_GyroDiff[2] > BMI088_CALIBRATION_GYRO_DIFF_LIMIT)
            {
                break;
            }

            // 按参考工程在相邻样本之间阻塞等待 500 微秒
            status = BMI088_MapBspStatus(BSP_IMU_DelayUs(BMI088_CALIBRATION_SAMPLE_DELAY_US));
            if (status != BMI088_OK)
            {
                return status;
            }
        }

        // 按固定采样数求平均，运动导致提前退出时通过校验条件触发整轮重采
        BMI088_GravityNorm /= (float)BMI088_CALIBRATION_SAMPLE_COUNT;
        for (uint8_t axis = 0U; axis < 3U; axis++)
        {
            BMI088_GyroOffset[axis] /= (float)BMI088_CALIBRATION_SAMPLE_COUNT;
        }

        // 记录本轮校准温度
        status = BMI088_ReadRegisters(BSP_IMU_ACCEL, BMI088_ACC_TEMP_M, temperature_data, sizeof(temperature_data));
        if (status != BMI088_OK)
        {
            return status;
        }
        BMI088_TemperatureWhenCalibratedC =
            (float)BMI088_ParseTemperature(temperature_data) * BMI088_TEMPERATURE_SENSITIVITY_C +
            BMI088_TEMPERATURE_OFFSET_C;
        BMI088_CalibrationCount++;
    } while (BMI088_GravityNormDiff > BMI088_CALIBRATION_GRAVITY_DIFF_LIMIT ||
             fabsf(BMI088_GravityNorm - BMI088_CALIBRATION_GRAVITY_REFERENCE) >
                 BMI088_CALIBRATION_GRAVITY_ERROR_LIMIT ||
             BMI088_GyroDiff[0] > BMI088_CALIBRATION_GYRO_DIFF_LIMIT ||
             BMI088_GyroDiff[1] > BMI088_CALIBRATION_GYRO_DIFF_LIMIT ||
             BMI088_GyroDiff[2] > BMI088_CALIBRATION_GYRO_DIFF_LIMIT ||
             fabsf(BMI088_GyroOffset[0]) > BMI088_CALIBRATION_GYRO_OFFSET_LIMIT ||
             fabsf(BMI088_GyroOffset[1]) > BMI088_CALIBRATION_GYRO_OFFSET_LIMIT ||
             fabsf(BMI088_GyroOffset[2]) > BMI088_CALIBRATION_GYRO_OFFSET_LIMIT);

    BMI088_AccelScale = BMI088_CALIBRATION_ACCEL_TARGET / BMI088_GravityNorm;
    return BMI088_OK;
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
    if (status == BMI088_OK)
    {
        // 按参考工程在传感器初始化完成后阻塞执行静止校准
        status = BMI088_Calibrate();
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
        status = BMI088_ReadRegisters(BSP_IMU_GYRO, BMI088_GYRO_CHIP_ID, gyro_data, sizeof(gyro_data));
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
    if (gyro_data[0] != BMI088_GYRO_CHIP_ID_VALUE)
    {
        bmi088_status = BMI088_ERR_NO_SENSOR;
        return BMI088_ERR_NO_SENSOR;
    }

    // 按参考工程应用加速度计比例系数和三轴陀螺仪零偏
    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
        const int16_t accel_raw = BMI088_CombineInt16(accel_data[axis * 2U], accel_data[axis * 2U + 1U]);
        const int16_t gyro_raw = BMI088_CombineInt16(gyro_data[axis * 2U + 2U], gyro_data[axis * 2U + 3U]);
        data->accel_mps2[axis] = (float)accel_raw * BMI088_ACC_SENSITIVITY_MPS2_PER_LSB * BMI088_AccelScale;
        data->gyro_rad_s[axis] = (float)gyro_raw * BMI088_GYRO_SENSITIVITY_RAD_PER_S - BMI088_GyroOffset[axis];
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
