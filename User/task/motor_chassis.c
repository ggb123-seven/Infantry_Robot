#include "task/motor_chassis.h"

#include "device/can_devices.h"
#include "module/chassis.h"
#include "task/ozone_debug.h"
#include "task/user_task.h"

_Static_assert(CHASSIS_MOTOR_COUNT == CAN_DEVICES_CHASSIS_MOTOR_COUNT, "底盘控制与 CAN 设备数量必须一致");

/*
 * 底盘任务私有状态：
 * - can_devices_snapshot：保存 CAN 设备初始化、本周期反馈和电流提交结果。
 * - chassis_feedback：保存从 CAN 设备快照映射出的本周期底盘控制反馈。
 * - chassis_output：保存底盘模块本周期计算得到的四路电流命令。
 * - chassis_snapshot：保存底盘速度控制器初始化和本周期控制结果。
 */
static CANDevices_Snapshot_t can_devices_snapshot;
static Chassis_Feedback_t chassis_feedback;
static Chassis_Output_t chassis_output;
static Chassis_Snapshot_t chassis_snapshot;

/**
 * @brief 初始化并周期运行四个 M3508 的速度控制链
 *
 * 调试使能关闭、设备离线或任一控制步骤失败时，对应电机电流指令保持为零。
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_motor_chassis(void *argument)
{
    (void)argument;

    const uint32_t delay_tick = osKernelGetTickFreq() / MOTOR_CHASSIS_FREQ;

    // 等待系统外设与业务模块完成统一初始化后再进入周期控制
    osDelay(MOTOR_CHASSIS_INIT_DELAY);

    // 建立绝对周期基准，避免控制周期累计漂移
    uint32_t tick = osKernelGetTickCount();
    while (1)
    {
        Chassis_Input_t chassis_input;

        // 刷新设备反馈并映射为底盘模块输入，单路反馈异常时只禁止对应速度控制器
        CANDevices_UpdateFeedback(&can_devices_snapshot);
        for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
        {
            chassis_feedback.valid[motor_index] =
                can_devices_snapshot.feedback_update_status[motor_index] == CAN_DEVICES_OK;
            chassis_feedback.motor_online[motor_index] = can_devices_snapshot.motor_online[motor_index];
            chassis_feedback.actual_speed_rpm[motor_index] = can_devices_snapshot.actual_speed_rpm[motor_index];
        }
        CANDevices_Command_t can_command =
        {
            .sequence = can_devices_snapshot.sequence,
        };

        // 取得一致控制参数并完成纯速度控制计算，设备输出尚未提交
        OzoneDebug_GetMotorChassisInput(&chassis_input, 1.0F / (float)MOTOR_CHASSIS_FREQ);
        Chassis_Run(&chassis_input, &chassis_feedback, &chassis_output, &chassis_snapshot);

        // 将四路计算电流组装为同序号命令，并通过设备集合统一发送一次控制帧
        for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
        {
            can_command.current_a[motor_index] = chassis_output.current_command_a[motor_index];
        }
        CANDevices_ApplyCurrent(&can_command, &can_devices_snapshot);

        // 将设备与控制结果集中发布到独立 Ozone 调试区，不在任务内维护诊断字段
        OzoneDebug_UpdateMotorChassis(&chassis_input, &can_devices_snapshot, &chassis_snapshot);

        tick += delay_tick;
        // 等待至下一个绝对任务周期，保持稳定控制频率
        osDelayUntil(tick);
    }
}

/**
 * @brief 初始化底盘任务私有控制链
 *
 * @return 底盘控制链可运行时返回 true，否则返回 false
 */
bool Task_motor_chassis_Init(void)
{
    // 分别初始化 CAN 设备集合与底盘速度控制器，保留单电机注册失败后的隔离运行能力
    CANDevices_Init(&can_devices_snapshot);
    Chassis_Init((float)MOTOR_CHASSIS_FREQ, &chassis_snapshot);

    // 将两层初始化结果集中发布到 Ozone 调试区，任一层整体不可用时拒绝启动周期任务
    OzoneDebug_UpdateMotorChassisInit(&can_devices_snapshot, &chassis_snapshot);
    return can_devices_snapshot.initialized && chassis_snapshot.initialized;
}
