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
 * @brief 将 CAN 设备集合和底盘模块单周期结果发布到 Ozone 监控区
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
        g_motor_chassis_monitor.speed.requested_speed_rpm[motor_index] = chassis_snapshot->requested_speed_rpm[motor_index];
        g_motor_chassis_monitor.speed.actual_speed_rpm[motor_index] = chassis_snapshot->actual_speed_rpm[motor_index];
    }
}
