#include "task/motor_chassis.h"

#include "module/chassis.h"
#include "task/ozone_debug.h"
#include "task/user_task.h"

/*
 * 底盘任务私有状态：
 * - chassis_snapshot：保存底盘初始化结果及本周期反馈、控制和电流提交结果。
 */
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

        // 从独立调试区取得一致控制参数，交由底盘模块完成本周期反馈、计算和发送
        OzoneDebug_GetMotorChassisInput(&chassis_input, 1.0F / (float)MOTOR_CHASSIS_FREQ);
        Chassis_Run(&chassis_input, &chassis_snapshot);

        // 将模块输出集中发布到独立 Ozone 调试区，不在任务内维护诊断字段
        OzoneDebug_UpdateMotorChassis(&chassis_input, &chassis_snapshot);

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
    // 初始化完整底盘控制链，并把设备和控制器结果发布到 Ozone 调试区
    Chassis_Init((float)MOTOR_CHASSIS_FREQ, &chassis_snapshot);
    OzoneDebug_UpdateMotorChassisInit(&chassis_snapshot);
    return chassis_snapshot.initialized;
}
