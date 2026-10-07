#include "task/ozone_debug.h"

#include "module/chassis.h"

#include <stdbool.h>
#include <stddef.h>

_Static_assert(OZONE_MOTOR_CHASSIS_COUNT == CHASSIS_MOTOR_COUNT,
               "Ozone 底盘调试电机数量必须与底盘控制链一致");
_Static_assert(OZONE_MOTOR_CHASSIS_COUNT == FAULT_DETECT_MOTOR_COUNT,
               "Ozone 底盘调试电机数量必须与故障检测链一致");

/*
 * 独立故障检测 Ozone 监视数据初值：
 * - 任务首次发布前 evaluated 为 false，避免调试器把尚未诊断的零值误读为无故障。
 */
volatile FaultDetect_Snapshot_t g_fault_detect_monitor;

/*
 * 四个 M3508 的 Ozone 调试参数初值：
 * - 底盘运动使能和运动向量由 DR16 遥控输入生成
 * - 轮速尺度初值为 100 rpm，ID1~3 使用公共速度 PID 参数
 * - 独立 PID 覆盖默认关闭，电机选择为 ID4，并从 ID4 固化参数开始
 */
volatile MotorChassisTune_t g_motor_chassis_tune =
{
    .scale_rpm = 100.0F,
    .pid_kp = CHASSIS_PID_KP,
    .pid_ki = CHASSIS_PID_KI,
    .pid_kd = CHASSIS_PID_KD,
    .pid_override =
    {
        .pid_override_enable = false,
        .pid_override_motor_index = 4U,
        .pid_override_kp = CHASSIS_MOTOR4_PID_KP,
        .pid_override_ki = CHASSIS_MOTOR4_PID_KI,
        .pid_override_kd = CHASSIS_MOTOR4_PID_KD,
    },
};

/*
 * 四个 M3508 的 Ozone 运行监视数据初值：
 * - 任务启动前所有设备和控制器均标记为不可用
 * - 数值反馈保持为零，避免尚未运行的状态被误认为有效数据
 * - 目标速度、反馈速度和速度环状态集中发布，避免调参结构体混入运行反馈
 */
volatile MotorChassisMonitor_t g_motor_chassis_monitor =
{
    .motor_online =
    {
        false,
        false,
        false,
        false,
    },
    .temperature_c =
    {
        0.0F,
        0.0F,
        0.0F,
        0.0F,
    },
    .speed_loop_ok =
    {
        false,
        false,
        false,
        false,
    },
    .speed =
    {
        .requested_speed_rpm =
        {
            0.0F,
            0.0F,
            0.0F,
            0.0F,
        },
        .actual_speed_rpm =
        {
            0.0F,
            0.0F,
            0.0F,
            0.0F,
        },
    },
};

/*
 * GM6020 Ozone 速度环在线参数初值：
 * - 当前调试配置开启速度控制，目标角速度为 1.0471976 rad/s，对应 10 rpm
 * - PID 参数使用 GM6020 模块的独立保守初值，供后续上板整定
 */
volatile MotorGM6020Tune_t g_motor_gm6020_tune =
{
    .speed_control_enable = true,
    .target_speed_rad_s = 1.0471976F,
    .pid_kp = GIMBAL_PID_KP,
    .pid_ki = GIMBAL_PID_KI,
    .pid_kd = GIMBAL_PID_KD,
};

/*
 * GM6020 Ozone 速度控制与物理反馈初值：
 * - 控制器和设备状态在对应初始化完成前均标记为不可用
 * - 目标、反馈和电流相关物理量保持为零
 */
volatile MotorGM6020Monitor_t g_motor_gm6020_monitor =
{
    .control_status = GIMBAL_NOT_INITIALIZED,
    .control_enabled = false,
    .target_speed_rad_s = 0.0F,
    .speed_error_rad_s = 0.0F,
    .current_command_a = 0.0F,
    .online = false,
    .speed_rad_s = 0.0F,
    .torque_current_a = 0.0F,
    .diagnostics =
    {
        .control_init_status = GIMBAL_NOT_INITIALIZED,
        .register_status = CAN_DEVICES_DEVICE_UNAVAILABLE,
        .angle_rad = 0.0F,
        .raw_current_lsb = 0,
        .temperature_c = 0.0F,
    },
};

/**
 * @brief 将 CAN 设备集合初始化结果发布到 Ozone 监控区
 *
 * @param[in] can_snapshot CAN 设备集合初始化结果快照
 * @return 无返回值
 */
void OzoneDebug_UpdateCANDevicesInit(const CANDevices_Snapshot_t *can_snapshot)
{
    if (can_snapshot == NULL)
    {
        return;
    }

    g_motor_gm6020_monitor.diagnostics.register_status = can_snapshot->gm6020_register_status;
}

/**
 * @brief 将底盘速度控制器初始化结果发布到 Ozone 监控区
 *
 * @param[in] chassis_snapshot 底盘速度控制器初始化结果快照
 * @return 无返回值
 */
void OzoneDebug_UpdateChassisInit(const Chassis_Snapshot_t *chassis_snapshot)
{
    if (chassis_snapshot == NULL)
    {
        return;
    }

    // 清除底盘常规监视数据，避免初始化阶段沿用上一次运行结果
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
    {
        g_motor_chassis_monitor.motor_online[motor_index] = false;
        g_motor_chassis_monitor.temperature_c[motor_index] = 0.0F;
        g_motor_chassis_monitor.speed_loop_ok[motor_index] = false;
        g_motor_chassis_monitor.speed.requested_speed_rpm[motor_index] = 0.0F;
        g_motor_chassis_monitor.speed.actual_speed_rpm[motor_index] = 0.0F;
    }
    (void)chassis_snapshot;
}

/**
 * @brief 将独立故障检测结果发布到 Ozone 监控区
 *
 * @param[in] fault_snapshot 本周期故障检测结果快照
 * @return 无返回值
 */
void OzoneDebug_UpdateFaultDetect(const FaultDetect_Snapshot_t *fault_snapshot)
{
    if (fault_snapshot == NULL)
    {
        return;
    }

    // 以完整快照发布诊断结果，避免调试器读取到跨周期混合字段
    g_fault_detect_monitor = *fault_snapshot;
}

/**
 * @brief 从 Ozone 在线参数生成本周期 GM6020 速度控制输入
 *
 * @param[out] input 待写入的 GM6020 速度控制输入
 * @param[in] control_period_s 控制周期，单位 s，必须大于 0
 * @return 无返回值
 */
void OzoneDebug_GetMotorGimbalInput(Gimbal_Input_t *input, float control_period_s)
{
    if (input == NULL)
    {
        return;
    }

    // 集中读取本周期允许由 Ozone 修改的速度目标和 GM6020 独立 PID 参数
    *input = (Gimbal_Input_t)
    {
        .enabled = g_motor_gm6020_tune.speed_control_enable,
        .target_speed_rad_s = g_motor_gm6020_tune.target_speed_rad_s,
        .pid_tune =
        {
            .kp = g_motor_gm6020_tune.pid_kp,
            .ki = g_motor_gm6020_tune.pid_ki,
            .kd = g_motor_gm6020_tune.pid_kd,
        },
        .control_period_s = control_period_s,
    };
}

/**
 * @brief 发布 GM6020 本周期速度控制与物理反馈数据
 *
 * @param[in] can_snapshot CAN 设备集合本周期快照，允许为 NULL
 * @param[in] gimbal_snapshot GM6020 速度控制结果快照，允许为 NULL
 * @return 无返回值
 */
void OzoneDebug_UpdateMotorGimbal(const CANDevices_Snapshot_t *can_snapshot,
                                  const Gimbal_Snapshot_t *gimbal_snapshot)
{
    if (gimbal_snapshot != NULL)
    {
        // 发布速度环状态、目标、反馈误差和最终电流指令
        g_motor_gm6020_monitor.diagnostics.control_init_status = gimbal_snapshot->init_status;
        g_motor_gm6020_monitor.control_status = gimbal_snapshot->control_status;
        g_motor_gm6020_monitor.control_enabled = gimbal_snapshot->enabled;
        g_motor_gm6020_monitor.target_speed_rad_s = gimbal_snapshot->ramped_target_speed_rad_s;
        g_motor_gm6020_monitor.speed_error_rad_s = gimbal_snapshot->speed_error_rad_s;
        g_motor_gm6020_monitor.current_command_a = gimbal_snapshot->current_command_a;
    }

    // 缺少设备快照时明确显示离线，清除物理反馈以免沿用旧读数
    if (can_snapshot == NULL)
    {
        g_motor_gm6020_monitor.online = false;
        g_motor_gm6020_monitor.diagnostics.angle_rad = 0.0F;
        g_motor_gm6020_monitor.speed_rad_s = 0.0F;
        g_motor_gm6020_monitor.diagnostics.raw_current_lsb = 0;
        g_motor_gm6020_monitor.torque_current_a = 0.0F;
        g_motor_gm6020_monitor.diagnostics.temperature_c = 0.0F;
        return;
    }

    // 从同一份 CAN 快照发布日常反馈与详细诊断，保持数据来源一致
    g_motor_gm6020_monitor.diagnostics.register_status = can_snapshot->gm6020_register_status;
    g_motor_gm6020_monitor.online = can_snapshot->gm6020_online;
    g_motor_gm6020_monitor.diagnostics.angle_rad = can_snapshot->gm6020_angle_rad;
    g_motor_gm6020_monitor.speed_rad_s = can_snapshot->gm6020_speed_rad_s;
    g_motor_gm6020_monitor.diagnostics.raw_current_lsb = can_snapshot->gm6020_raw_current_lsb;
    g_motor_gm6020_monitor.torque_current_a = can_snapshot->gm6020_torque_current_a;
    g_motor_gm6020_monitor.diagnostics.temperature_c = can_snapshot->gm6020_temperature_c;
}

/**
 * @brief 将 CAN 设备集合、GM6020 和底盘模块单周期结果发布到 Ozone 监控区
 *
 * @param[in] input 本周期实际采用的底盘控制输入快照
 * @param[in] can_snapshot 本周期 CAN 设备反馈与输出结果快照，允许为 NULL
 * @param[in] chassis_snapshot 本周期底盘速度控制结果快照
 * @return 无返回值
 */
void OzoneDebug_UpdateMotorChassis(const Chassis_Input_t *input, const CANDevices_Snapshot_t *can_snapshot,
                                   const Chassis_Snapshot_t *chassis_snapshot)
{
    if (chassis_snapshot == NULL)
    {
        return;
    }

    (void)input;

    // 汇总四路速度环状态和运动学目标，保持目标与反馈在同一组监视数据中
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
    {
        if (can_snapshot != NULL)
        {
            g_motor_chassis_monitor.motor_online[motor_index] = can_snapshot->motor_online[motor_index];
            g_motor_chassis_monitor.temperature_c[motor_index] = can_snapshot->temperature_c[motor_index];
        }
        else
        {
            g_motor_chassis_monitor.motor_online[motor_index] = false;
            g_motor_chassis_monitor.temperature_c[motor_index] = 0.0F;
        }

        g_motor_chassis_monitor.speed_loop_ok[motor_index] =
            chassis_snapshot->control_status[motor_index] == CHASSIS_MOTOR_OK;
        g_motor_chassis_monitor.speed.requested_speed_rpm[motor_index] =
            chassis_snapshot->requested_speed_rpm[motor_index];
        g_motor_chassis_monitor.speed.actual_speed_rpm[motor_index] = chassis_snapshot->actual_speed_rpm[motor_index];
    }
}
