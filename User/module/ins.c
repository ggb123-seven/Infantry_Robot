#include "module/ins.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "bsp/imu.h"

/* 温控参考 500 Hz 与 40 摄氏度；PID 系数按参考 2000 计数满量程换算为 BSP 的 1000 千分比满量程 */
#define INS_TEMPERATURE_CONTROL_HZ (500.0F)
#define INS_TEMPERATURE_SETPOINT_C (40.0F)
#define INS_TEMPERATURE_PID_OUTPUT_LIMIT (1000.0F)
#define INS_TEMPERATURE_PID_INTEGRAL_OUTPUT_LIMIT (150.0F)
#define INS_TEMPERATURE_PID_PROPORTIONAL_GAIN (500.0F)
#define INS_TEMPERATURE_PID_INTEGRAL_GAIN (10.0F)
#define INS_TEMPERATURE_PID_DERIVATIVE_CUTOFF_HZ (20.0F)

/* INS 安装角默认取零，姿态参数、运动加速度低通时间常数和重力值沿用参考 ins_task.c */
#define INS_MOTION_ACCEL_LPF_TIME_CONSTANT_S (0.0085F)
#define INS_REFERENCE_GRAVITY_MPS2 (9.81F)
#define INS_RAD_TO_DEG (57.295779513F)

static const QuaternionEKF_Config_t ins_quaternion_ekf_config =
{
    .process_noise_quaternion = 10.0F,
    .process_noise_gyro_bias = 0.001F,
    .measurement_noise_accel = 10000000.0F,
    .fading_factor = 1.0F,
    .accel_lpf_time_constant_s = 0.0F,
    .chi_square_test_threshold = 1.0e-8F,
};

static const KPID_Params_t ins_temperature_pid_params =
{
    .k = 1.0F,
    .p = INS_TEMPERATURE_PID_PROPORTIONAL_GAIN,
    .i = INS_TEMPERATURE_PID_INTEGRAL_GAIN,
    .d = 0.0F,
    .i_limit = INS_TEMPERATURE_PID_INTEGRAL_OUTPUT_LIMIT / INS_TEMPERATURE_PID_INTEGRAL_GAIN,
    .out_limit = INS_TEMPERATURE_PID_OUTPUT_LIMIT,
    .d_cutoff_freq = INS_TEMPERATURE_PID_DERIVATIVE_CUTOFF_HZ,
    .range = 0.0F,
};

static const float ins_body_axis_x[3] = {1.0F, 0.0F, 0.0F};
static const float ins_body_axis_y[3] = {0.0F, 1.0F, 0.0F};
static const float ins_body_axis_z[3] = {0.0F, 0.0F, 1.0F};
static const float ins_gravity_navigation[3] = {0.0F, 0.0F, INS_REFERENCE_GRAVITY_MPS2};

static bool INS_ValuesAreFinite(const float *values, uint8_t length);
static bool INS_UpdateCorrectionMatrix(INS_t *ins, bool force_update);
static bool INS_CorrectSensorData(INS_t *ins, const BMI088_Data_t *sample);
static void INS_BodyFrameToEarthFrame(const float vector_body[3], float vector_navigation[3], const float q[4]);
static void INS_EarthFrameToBodyFrame(const float vector_navigation[3], float vector_body[3], const float q[4]);
static INS_Status_t INS_EnterFault(INS_t *ins, INS_Status_t status);
static bool INS_UpdatePublishedAttitude(INS_t *ins);
static bool INS_UpdateMotionAcceleration(INS_t *ins, float dt_s);
static INS_Status_t INS_UpdateTemperatureControl(INS_t *ins, float temperature_c, float dt_s);

/**
 * @brief 检查浮点数组中的值是否均为有限数
 *
 * @param[in] values 待检查数组
 * @param[in] length 数组元素数量
 * @return 全部为有限数时返回 true
 */
static bool INS_ValuesAreFinite(const float *values, uint8_t length)
{
    if (values == NULL)
    {
        return false;
    }

    for (uint8_t index = 0U; index < length; index++)
    {
        if (!isfinite(values[index]))
        {
            return false;
        }
    }

    return true;
}

/**
 * @brief 根据安装修正角更新三轴坐标变换矩阵
 *
 * @param[in,out] ins INS 模块实例
 * @param[in] force_update 为 true 时强制重新生成矩阵
 * @return 修正角有效且矩阵生成成功时返回 true
 */
static bool INS_UpdateCorrectionMatrix(INS_t *ins, bool force_update)
{
    const float yaw_deg = ins->correction_yaw_deg;
    const float pitch_deg = ins->correction_pitch_deg;
    const float roll_deg = ins->correction_roll_deg;
    if (!isfinite(yaw_deg) || !isfinite(pitch_deg) || !isfinite(roll_deg))
    {
        return false;
    }
    if (!force_update && yaw_deg == ins->correction_angle_last[0] && pitch_deg == ins->correction_angle_last[1] &&
        roll_deg == ins->correction_angle_last[2])
    {
        return true;
    }

    // 按参考工程的偏航、俯仰、横滚顺序生成安装误差修正矩阵
    const float yaw_rad = yaw_deg / INS_RAD_TO_DEG;
    const float pitch_rad = pitch_deg / INS_RAD_TO_DEG;
    const float roll_rad = roll_deg / INS_RAD_TO_DEG;
    const float cos_yaw = cosf(yaw_rad);
    const float cos_pitch = cosf(pitch_rad);
    const float cos_roll = cosf(roll_rad);
    const float sin_yaw = sinf(yaw_rad);
    const float sin_pitch = sinf(pitch_rad);
    const float sin_roll = sinf(roll_rad);

    ins->correction_matrix[0] = cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll;
    ins->correction_matrix[1] = cos_pitch * sin_yaw;
    ins->correction_matrix[2] = cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch;
    ins->correction_matrix[3] = cos_yaw * sin_pitch * sin_roll - cos_roll * sin_yaw;
    ins->correction_matrix[4] = cos_yaw * cos_pitch;
    ins->correction_matrix[5] = -sin_yaw * sin_roll - cos_yaw * cos_roll * sin_pitch;
    ins->correction_matrix[6] = -cos_pitch * sin_roll;
    ins->correction_matrix[7] = sin_pitch;
    ins->correction_matrix[8] = cos_pitch * cos_roll;
    if (!INS_ValuesAreFinite(ins->correction_matrix, 9U))
    {
        return false;
    }

    ins->correction_angle_last[0] = yaw_deg;
    ins->correction_angle_last[1] = pitch_deg;
    ins->correction_angle_last[2] = roll_deg;
    return true;
}

/**
 * @brief 将 BMI088 原始换算数据应用比例系数和安装误差修正
 *
 * @param[in,out] ins INS 模块实例
 * @param[in] sample BMI088 最近一次完整读取数据
 * @return 修正后的角速度和加速度均为有限数时返回 true
 */
static bool INS_CorrectSensorData(INS_t *ins, const BMI088_Data_t *sample)
{
    if (!INS_UpdateCorrectionMatrix(ins, false) || !INS_ValuesAreFinite(ins->gyro_scale, 3U))
    {
        return false;
    }

    float gyro_scaled[3];
    float accel_raw[3];
    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
        gyro_scaled[axis] = sample->gyro_rad_s[axis] * ins->gyro_scale[axis];
        accel_raw[axis] = sample->accel_mps2[axis];
    }
    if (!INS_ValuesAreFinite(gyro_scaled, 3U) || !INS_ValuesAreFinite(accel_raw, 3U))
    {
        return false;
    }

    // 使用同一安装矩阵修正角速度和加速度，保持两类传感器轴向一致
    for (uint8_t row = 0U; row < 3U; row++)
    {
        const uint8_t matrix_index = (uint8_t)(row * 3U);
        ins->state.Gyro[row] = ins->correction_matrix[matrix_index] * gyro_scaled[0] +
                               ins->correction_matrix[matrix_index + 1U] * gyro_scaled[1] +
                               ins->correction_matrix[matrix_index + 2U] * gyro_scaled[2];
        ins->state.Accel[row] = ins->correction_matrix[matrix_index] * accel_raw[0] +
                                ins->correction_matrix[matrix_index + 1U] * accel_raw[1] +
                                ins->correction_matrix[matrix_index + 2U] * accel_raw[2];
    }

    return INS_ValuesAreFinite(ins->state.Gyro, 3U) && INS_ValuesAreFinite(ins->state.Accel, 3U);
}

/**
 * @brief 将机体系三维向量转换到导航坐标系
 *
 * @param[in] vector_body 机体系输入向量
 * @param[out] vector_navigation 导航系输出向量
 * @param[in] q 姿态四元数 [w,x,y,z]
 * @return 无
 */
static void INS_BodyFrameToEarthFrame(const float vector_body[3], float vector_navigation[3], const float q[4])
{
    vector_navigation[0] = 2.0F * ((0.5F - q[2] * q[2] - q[3] * q[3]) * vector_body[0] +
                                   (q[1] * q[2] - q[0] * q[3]) * vector_body[1] +
                                   (q[1] * q[3] + q[0] * q[2]) * vector_body[2]);
    vector_navigation[1] = 2.0F * ((q[1] * q[2] + q[0] * q[3]) * vector_body[0] +
                                   (0.5F - q[1] * q[1] - q[3] * q[3]) * vector_body[1] +
                                   (q[2] * q[3] - q[0] * q[1]) * vector_body[2]);
    vector_navigation[2] = 2.0F * ((q[1] * q[3] - q[0] * q[2]) * vector_body[0] +
                                   (q[2] * q[3] + q[0] * q[1]) * vector_body[1] +
                                   (0.5F - q[1] * q[1] - q[2] * q[2]) * vector_body[2]);
}

/**
 * @brief 将导航坐标系三维向量转换到机体系
 *
 * @param[in] vector_navigation 导航系输入向量
 * @param[out] vector_body 机体系输出向量
 * @param[in] q 姿态四元数 [w,x,y,z]
 * @return 无
 */
static void INS_EarthFrameToBodyFrame(const float vector_navigation[3], float vector_body[3], const float q[4])
{
    vector_body[0] = 2.0F * ((0.5F - q[2] * q[2] - q[3] * q[3]) * vector_navigation[0] +
                             (q[1] * q[2] + q[0] * q[3]) * vector_navigation[1] +
                             (q[1] * q[3] - q[0] * q[2]) * vector_navigation[2]);
    vector_body[1] = 2.0F * ((q[1] * q[2] - q[0] * q[3]) * vector_navigation[0] +
                             (0.5F - q[1] * q[1] - q[3] * q[3]) * vector_navigation[1] +
                             (q[2] * q[3] + q[0] * q[1]) * vector_navigation[2]);
    vector_body[2] = 2.0F * ((q[1] * q[3] + q[0] * q[2]) * vector_navigation[0] +
                             (q[2] * q[3] - q[0] * q[1]) * vector_navigation[1] +
                             (0.5F - q[1] * q[1] - q[2] * q[2]) * vector_navigation[2]);
}

/**
 * @brief 关闭加热输出并将 INS 模块锁定到故障状态
 *
 * @param[in,out] ins INS 模块实例
 * @param[in] status 触发故障的主状态
 * @return 保存的故障状态
 */
static INS_Status_t INS_EnterFault(INS_t *ins, INS_Status_t status)
{
    ins->module_state = INS_MODULE_STATE_FAULT;
    ins->status = status;
    ins->heat_status = BSP_IMU_HeatStop();
    memset(&ins->state, 0, sizeof(ins->state));
    return status;
}

/**
 * @brief 将 EKF 姿态及参考工程的角度定义发布到 INS 状态
 *
 * @param[in,out] ins INS 模块实例
 * @return 姿态输出均为有限数时返回 true
 */
static bool INS_UpdatePublishedAttitude(INS_t *ins)
{
    memcpy(ins->state.q, ins->quaternion_ekf.q, sizeof(ins->state.q));
    ins->state.Roll = ins->quaternion_ekf.Roll;
    ins->state.Pitch = ins->quaternion_ekf.Pitch;
    ins->state.Yaw = ins->quaternion_ekf.Yaw;
    ins->state.YawTotalAngle = ins->quaternion_ekf.YawTotalAngle;
    ins->state.atanxz = -atan2f(ins->state.Accel[0], ins->state.Accel[2]) * INS_RAD_TO_DEG;
    ins->state.atanyz = atan2f(ins->state.Accel[1], ins->state.Accel[2]) * INS_RAD_TO_DEG;

    // 使用当前四元数更新机体系三轴在导航系中的方向
    INS_BodyFrameToEarthFrame(ins_body_axis_x, ins->state.xn, ins->state.q);
    INS_BodyFrameToEarthFrame(ins_body_axis_y, ins->state.yn, ins->state.q);
    INS_BodyFrameToEarthFrame(ins_body_axis_z, ins->state.zn, ins->state.q);

    const float published_angles[6] =
    {
        ins->state.atanxz,
        ins->state.atanyz,
        ins->state.Roll,
        ins->state.Pitch,
        ins->state.Yaw,
        ins->state.YawTotalAngle,
    };
    return INS_ValuesAreFinite(ins->state.q, 4U) && INS_ValuesAreFinite(published_angles, 6U) &&
           INS_ValuesAreFinite(ins->state.xn, 3U) && INS_ValuesAreFinite(ins->state.yn, 3U) &&
           INS_ValuesAreFinite(ins->state.zn, 3U);
}

/**
 * @brief 分离重力并低通发布机体系与导航系运动加速度
 *
 * @param[in,out] ins INS 模块实例
 * @param[in] dt_s 本次姿态更新间隔，单位 s
 * @return 加速度输出均为有限数时返回 true
 */
static bool INS_UpdateMotionAcceleration(INS_t *ins, float dt_s)
{
    const float denominator = ins->MotionAccelLPF + dt_s;
    if (!isfinite(denominator) || denominator <= 0.0F)
    {
        return false;
    }

    // 将导航系重力转换到机体系并按参考一阶低通分离运动加速度
    float gravity_body[3];
    INS_EarthFrameToBodyFrame(ins_gravity_navigation, gravity_body, ins->state.q);
    const float input_weight = dt_s / denominator;
    const float previous_weight = ins->MotionAccelLPF / denominator;
    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
        const float motion_acceleration = ins->state.Accel[axis] - gravity_body[axis];
        ins->state.MotionAccel_b[axis] = motion_acceleration * input_weight +
                                         ins->state.MotionAccel_b[axis] * previous_weight;
    }

    // 将低通后的机体系运动加速度转换到导航系
    INS_BodyFrameToEarthFrame(ins->state.MotionAccel_b, ins->state.MotionAccel_n, ins->state.q);
    return INS_ValuesAreFinite(gravity_body, 3U) && INS_ValuesAreFinite(ins->state.MotionAccel_b, 3U) &&
           INS_ValuesAreFinite(ins->state.MotionAccel_n, 3U);
}

/**
 * @brief 按二分频周期更新温控 PID 并设置加热 PWM
 *
 * @param[in,out] ins INS 模块实例
 * @param[in] temperature_c BMI088 温度，单位摄氏度
 * @param[in] dt_s 本次采集间隔，单位 s
 * @return 温控成功返回 INS_STATUS_OK，数值或 PWM 错误时返回对应状态
 */
static INS_Status_t INS_UpdateTemperatureControl(INS_t *ins, float temperature_c, float dt_s)
{
    ins->temperature_elapsed_s += dt_s;
    if (!isfinite(ins->temperature_elapsed_s))
    {
        return INS_STATUS_NUMERIC;
    }

    if (ins->temperature_control_divider == 0U)
    {
        // 以实际累计周期计算温度控制输出并限制为加热方向
        const float output = PID_Calc(&ins->temperature_controller, INS_TEMPERATURE_SETPOINT_C, temperature_c,
                                      0.0F, ins->temperature_elapsed_s);
        if (!isfinite(output))
        {
            return INS_STATUS_NUMERIC;
        }
        const float limited_output = fmaxf(0.0F, fminf(INS_TEMPERATURE_PID_OUTPUT_LIMIT, output));
        const uint16_t duty_permille = (uint16_t)(limited_output + 0.5F);
        ins->heat_status = BSP_IMU_HeatSetDuty(duty_permille);
        if (ins->heat_status != BSP_OK)
        {
            return INS_STATUS_HEATER;
        }
        ins->temperature_elapsed_s = 0.0F;
    }

    ins->temperature_control_divider ^= 1U;
    return INS_STATUS_OK;
}

/**
 * @brief 初始化 BMI088、姿态滤波器和加热温控
 *
 * @param[in,out] ins INS 模块实例，不能为空
 * @return 初始化成功返回 INS_STATUS_OK，设备、滤波器、PID 或加热器初始化失败时进入故障状态
 */
INS_Status_t INS_Init(INS_t *ins)
{
    if (ins == NULL)
    {
        return INS_STATUS_NULL;
    }

    // 清空实例并先关闭加热输出，避免重初始化时沿用旧占空比
    memset(ins, 0, sizeof(*ins));
    ins->module_state = INS_MODULE_STATE_INITIALIZING;
    ins->status = INS_STATUS_NOT_INITIALIZED;
    ins->bmi088_status = BMI088_ERR_NOT_INITIALIZED;
    ins->quaternion_ekf_status = QUATERNION_EKF_STATUS_NOT_INITIALIZED;
    ins->heat_status = BSP_IMU_HeatStop();
    if (ins->heat_status != BSP_OK)
    {
        return INS_EnterFault(ins, INS_STATUS_HEATER);
    }

    // 初始化 BMI088 并保留设备层返回状态供故障定位
    ins->bmi088_status = BMI088_Init();
    if (ins->bmi088_status != BMI088_OK)
    {
        return INS_EnterFault(ins, INS_STATUS_BMI088);
    }

    // 初始化姿态 EKF 和参考工程的 1000 Hz 运动加速度低通参数
    ins->quaternion_ekf_status = QuaternionEKF_Init(&ins->quaternion_ekf, &ins_quaternion_ekf_config);
    if (ins->quaternion_ekf_status != QUATERNION_EKF_STATUS_OK)
    {
        return INS_EnterFault(ins, INS_STATUS_QUATERNION_EKF);
    }
    ins->MotionAccelLPF = INS_MOTION_ACCEL_LPF_TIME_CONSTANT_S;
    ins->gyro_scale[0] = 1.0F;
    ins->gyro_scale[1] = 1.0F;
    ins->gyro_scale[2] = 1.0F;
    if (!INS_UpdateCorrectionMatrix(ins, true))
    {
        return INS_EnterFault(ins, INS_STATUS_NUMERIC);
    }

    // 按参考 500 Hz 温控器参数初始化 PID，输出量纲与板级千分比接口一致
    ins->pid_status = PID_Init(&ins->temperature_controller, KPID_MODE_NO_D, INS_TEMPERATURE_CONTROL_HZ,
                               &ins_temperature_pid_params);
    if (ins->pid_status != 0)
    {
        return INS_EnterFault(ins, INS_STATUS_PID);
    }

    // 全部算法准备完成后再启动 PWM，避免初始化失败时留下加热输出
    ins->heat_status = BSP_IMU_HeatStart();
    if (ins->heat_status != BSP_OK)
    {
        return INS_EnterFault(ins, INS_STATUS_HEATER);
    }

    ins->module_state = INS_MODULE_STATE_RUNNING;
    ins->status = INS_STATUS_OK;
    return INS_STATUS_OK;
}

/**
 * @brief 读取 IMU 数据并更新姿态、坐标变换、运动加速度和温控
 *
 * @param[in,out] ins 已初始化的 INS 模块实例
 * @param[in] dt_s 本次采集间隔，单位 s，必须为有限正数
 * @return 更新成功返回 INS_STATUS_OK，设备或算法失败时关闭加热器并进入故障状态
 */
INS_Status_t INS_Update(INS_t *ins, float dt_s)
{
    if (ins == NULL)
    {
        return INS_STATUS_NULL;
    }

    // 集中检查模块运行状态，故障后必须通过重新初始化恢复
    switch (ins->module_state)
    {
        case INS_MODULE_STATE_UNINITIALIZED:
            return INS_STATUS_NOT_INITIALIZED;

        case INS_MODULE_STATE_INITIALIZING:
            return INS_STATUS_NOT_READY;

        case INS_MODULE_STATE_RUNNING:
            break;

        case INS_MODULE_STATE_FAULT:
            return ins->status;

        default:
            return INS_EnterFault(ins, INS_STATUS_ILLEGAL_STATE);
    }

    // 采样周期非法时进入故障安全状态，避免污染滤波器历史并关闭加热器
    if (!isfinite(dt_s) || dt_s <= 0.0F)
    {
        return INS_EnterFault(ins, INS_STATUS_INVALID_ARGUMENT);
    }

    ins->state.valid = false;

    // 读取完整 BMI088 样本，设备故障时清除公开姿态并关闭加热
    BMI088_Data_t sample = {0};
    ins->bmi088_status = BMI088_Read(&sample);
    if (ins->bmi088_status != BMI088_OK || !sample.valid)
    {
        return INS_EnterFault(ins, INS_STATUS_BMI088);
    }
    if (!isfinite(sample.temperature_c))
    {
        return INS_EnterFault(ins, INS_STATUS_NUMERIC);
    }

    // 应用安装修正并将同一组传感器数据送入姿态 EKF
    if (!INS_CorrectSensorData(ins, &sample))
    {
        return INS_EnterFault(ins, INS_STATUS_NUMERIC);
    }
    ins->quaternion_ekf_status = QuaternionEKF_Update(&ins->quaternion_ekf, ins->state.Gyro[0], ins->state.Gyro[1],
                                                      ins->state.Gyro[2], ins->state.Accel[0], ins->state.Accel[1],
                                                      ins->state.Accel[2], dt_s);
    if (ins->quaternion_ekf_status != QUATERNION_EKF_STATUS_OK)
    {
        return INS_EnterFault(ins, INS_STATUS_QUATERNION_EKF);
    }

    // 发布姿态、导航方向和扣除重力后的运动加速度
    if (!INS_UpdatePublishedAttitude(ins) || !INS_UpdateMotionAcceleration(ins, dt_s))
    {
        return INS_EnterFault(ins, INS_STATUS_NUMERIC);
    }
    ins->state.Temperature = sample.temperature_c;

    // 每两次 1000 Hz 采集执行一次 500 Hz 温控并检查 PWM 返回状态
    const INS_Status_t temperature_status = INS_UpdateTemperatureControl(ins, sample.temperature_c, dt_s);
    if (temperature_status != INS_STATUS_OK)
    {
        return INS_EnterFault(ins, temperature_status);
    }

    ins->state.valid = true;
    ins->UpdateCount++;
    ins->status = INS_STATUS_OK;
    return INS_STATUS_OK;
}

/**
 * @brief 获取最近一次有效的 INS 对外状态
 *
 * @param[in] ins 已初始化的 INS 模块实例
 * @param[out] state 状态输出地址，不能为空
 * @return 状态有效时返回 INS_STATUS_OK，尚无有效更新时返回 INS_STATUS_NOT_READY，故障时返回最近故障状态
 */
INS_Status_t INS_GetState(const INS_t *ins, INS_State_t *state)
{
    if (ins == NULL || state == NULL)
    {
        return INS_STATUS_NULL;
    }

    // 无论成功与否都返回当前可诊断快照，调用方必须同时检查状态码和 valid 标志
    *state = ins->state;
    switch (ins->module_state)
    {
        case INS_MODULE_STATE_UNINITIALIZED:
            return INS_STATUS_NOT_INITIALIZED;

        case INS_MODULE_STATE_INITIALIZING:
            return INS_STATUS_NOT_READY;

        case INS_MODULE_STATE_RUNNING:
            return state->valid ? INS_STATUS_OK : INS_STATUS_NOT_READY;

        case INS_MODULE_STATE_FAULT:
            return ins->status;

        default:
            return INS_STATUS_ILLEGAL_STATE;
    }
}
