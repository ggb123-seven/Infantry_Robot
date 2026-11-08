#include "task/motor_gimbal.h"

#include <math.h>
#include <stddef.h>

#include "bsp/time.h"
#include "module/cmd.h"
#include "device/dr16.h"
#include "module/config.h"
#include "task/user_task.h"

#if MOTOR_GIMBAL_ENABLE_SINE_TEST
/*
 * yaw 正弦跟随测试参数，角度单位为度，相位角速度单位为 rad/s：
 * - 中心取启动测试时的当前姿态，振幅 30°，前馈峰值为 π rad/s
 * - 2π 用于相位归一化，避免长时间累计相位导致浮点分辨率下降
 */
#define MOTOR_GIMBAL_SINE_AMPLITUDE_DEG (30.0f)
#define MOTOR_GIMBAL_SINE_OMEGA_RAD_S (6.0f)
#define MOTOR_GIMBAL_TWO_PI (6.28318530718f)
#define MOTOR_GIMBAL_DEG_TO_RAD (0.0174532925199f)

// 正弦测试待机时相位归零，yaw 进入角度闭环后才开始推进，异常状态回到待机
typedef enum
{
    MOTOR_GIMBAL_SINE_IDLE = 0,
    MOTOR_GIMBAL_SINE_RUNNING,
} MotorGimbal_SineState_t;
#endif

// 调试观测与命令入口，声明及使用方式见 motor_gimbal.h
Gimbal_t gimbal;
Gimbal_Command_t gimbal_command =
{
    .mode = GIMBAL_MODE_RELAX,
};
#if MOTOR_GIMBAL_ENABLE_SINE_TEST
volatile bool gimbal_sine_test_enabled = false;
volatile bool gimbal_sine_feedforward_enabled = true;

static MotorGimbal_SineState_t sine_state;
static float sine_phase_rad;
static float sine_center_deg;
#endif

// 遥控缓存及输入状态仅由云台任务访问，不与 ISR 共享
static DR16_State_t remote_cache;
static bool remote_cache_valid;
static Cmd_Gimbal_t remote_command;
// 底盘状态仅由本任务消费，零初始化对应离线，在线未自旋与在线小陀螺独立表示
static Gimbal_ChassisState_t chassis_state_cache;

static void MotorGimbal_ApplyDR16Input(float period_s);
#if MOTOR_GIMBAL_ENABLE_SINE_TEST
static void MotorGimbal_UpdateSineTest(float period_s);
#endif
static bool MotorGimbal_ReadFeedback(osMessageQueueId_t queue, MOTOR_RM_Feedback_t *feedback);
static bool MotorGimbal_PublishOutput(const Gimbal_Output_t *output);

/**
 * @brief 初始化云台模块并周期执行反馈读取、控制计算和电流发布
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_motor_gimbal(void *argument)
{
    (void)argument;

    const uint32_t tick_frequency = osKernelGetTickFreq();
    const uint32_t delay_tick = tick_frequency / MOTOR_GIMBAL_FREQ;

    // 等待统一装配完成后初始化云台模块，失败时只发布零电流
    osDelay(MOTOR_GIMBAL_INIT_DELAY);
    const bool initialized = Gimbal_Init(&gimbal, &gimbal_config, (float)MOTOR_GIMBAL_FREQ) == 0;
    uint32_t tick = osKernelGetTickCount();
    uint32_t previous_tick = tick - delay_tick;
    while (true)
    {
        // 按实际调度间隔计算控制周期，错过节拍时由模块拒绝过长周期
        const uint32_t now_tick = osKernelGetTickCount();
        const float period_s = (float)(now_tick - previous_tick) / (float)tick_frequency;
        previous_tick = now_tick;
        Gimbal_Output_t output =
        {
            0,
        };

        if (initialized)
        {
            // 消费完整惯性快照，无新消息时由模块保留源时间并检查帧龄
            Gimbal_ImuFeedback_t imu;
            const osMessageQueueId_t imu_queue = task_runtime.msgq.gimbal_imu;
            const bool imu_received = osMessageQueueGet(imu_queue, &imu, NULL, 0U) == osOK;

            // 消费两轴最新反馈帧，邮箱为空时传 NULL 由模块检查旧帧时效
            MOTOR_RM_Feedback_t yaw_feedback;
            MOTOR_RM_Feedback_t pitch_feedback;
            const bool yaw_received = MotorGimbal_ReadFeedback(task_runtime.msgq.gimbal_yaw_feedback, &yaw_feedback);
            const bool pitch_received = MotorGimbal_ReadFeedback(
                task_runtime.msgq.gimbal_pitch_feedback, &pitch_feedback);
            Gimbal_UpdateFeedback(&gimbal, yaw_received ? &yaw_feedback : NULL,
                                  pitch_received ? &pitch_feedback : NULL, imu_received ? &imu : NULL, BSP_TIME_Get());

            // 底盘在线与小陀螺共用同一状态快照，空邮箱保留上次状态，读取错误按离线确认
            Gimbal_ChassisState_t received_state;
            const osStatus_t chassis_status = osMessageQueueGet(task_runtime.msgq.chassis_gimbal, &received_state,
                                                                NULL, 0U);
            if (chassis_status == osOK)
            {
                chassis_state_cache = received_state;
            }
            else if (chassis_status != osErrorResource)
            {
                chassis_state_cache = GIMBAL_CHASSIS_OFFLINE;
            }

            // 遥控输入先决定是否使能，正弦测试同样受遥控失能约束，再执行模块闭环
            MotorGimbal_ApplyDR16Input(period_s);
#if MOTOR_GIMBAL_ENABLE_SINE_TEST
            MotorGimbal_UpdateSineTest(period_s);
#endif
            // 按底盘在线及小陀螺状态执行两轴闭环与 yaw 悬空保护，再导出两轴电流
            Gimbal_Control(&gimbal, &gimbal_command, chassis_state_cache, period_s);
            Gimbal_DumpOutput(&gimbal, &output);
        }

        // 发布失败时 CAN 任务按空邮箱输出零电流，同步让两轴停机，避免积分在零输出期间累计
        if (!MotorGimbal_PublishOutput(&output) && initialized)
        {
            Gimbal_Command_t relax_command =
            {
                .mode = GIMBAL_MODE_RELAX,
            };
            Gimbal_Control(&gimbal, &relax_command, chassis_state_cache, period_s);
            Cmd_ResetGimbal(&remote_command);
            gimbal_command.mode = GIMBAL_MODE_RELAX;
            gimbal_command.yaw.rate_rad_s = 0.0f;
            gimbal_command.pitch.rate_rad_s = 0.0f;
        }

        // 正常运行使用绝对节拍，错过周期时跳过补算，避免追赶产生连续重复输出
        if ((uint32_t)(now_tick - tick) >= delay_tick)
        {
            tick = now_tick;
        }
        tick += delay_tick;
        osDelayUntil(tick);
    }
}

/**
 * @brief 消费独立遥控邮箱并将右拨杆和右摇杆转换为云台角度命令
 *
 * @param[in] period_s 实际控制间隔，单位秒，合法范围为 (0, 0.1]
 * @return 无返回值
 */
static void MotorGimbal_ApplyDR16Input(float period_s)
{
    // 邮箱只更新缓存，不改写源时间，读取错误立即撤销缓存
    DR16_State_t received;
    const osStatus_t status = osMessageQueueGet(task_runtime.msgq.dr16_gimbal_state, &received, NULL, 0U);
    if (status == osOK)
    {
        remote_cache = received;
        remote_cache_valid = true;
    }
    else if (status != osErrorResource)
    {
        remote_cache_valid = false;
    }

    // 读取快照后再采时间，由命令层统一检查帧龄和人工使能顺序
    const uint64_t now_us = BSP_TIME_Get();
    remote_cache_valid = Cmd_ParseGimbal(&remote_command, &remote_cache, remote_cache_valid,
        now_us, period_s, Gimbal_HasLatchedFault(&gimbal), &gimbal_command);
}

#if MOTOR_GIMBAL_ENABLE_SINE_TEST
/**
 * @brief yaw 处于角度闭环时生成正弦目标及其解析角速度前馈
 *
 * 测试只改写 yaw 命令；退出时恢复增量来源，沿用模块最后限幅目标并清零前馈
 *
 * @param[in] period_s 实际控制间隔，单位 s
 * @return 无返回值
 */
static void MotorGimbal_UpdateSineTest(float period_s)
{
    const bool active = gimbal_sine_test_enabled && gimbal_command.mode == GIMBAL_MODE_ANGLE &&
                        gimbal.yaw.state == GIMBAL_AXIS_RUNNING && gimbal.imu_state == GIMBAL_IMU_READY &&
                        isfinite(gimbal.yaw.angle_deg) && isfinite(period_s) && period_s > 0.0f;

    // 测试状态机：待机时相位归零，首次运行从中心开始，之后按实际周期推进
    switch (sine_state)
    {
        case MOTOR_GIMBAL_SINE_IDLE:
            if (!active)
            {
                return;
            }
            sine_phase_rad = 0.0f;
            sine_center_deg = gimbal.yaw.angle_deg;
            sine_state = MOTOR_GIMBAL_SINE_RUNNING;
            break;

        case MOTOR_GIMBAL_SINE_RUNNING:
            if (!active)
            {
                // 退出测试时撤销前馈，增量来源由本周期命令层恢复
                gimbal_command.yaw.rate_rad_s = 0.0f;
                sine_phase_rad = 0.0f;
                sine_state = MOTOR_GIMBAL_SINE_IDLE;
                return;
            }
            sine_phase_rad = fmodf(sine_phase_rad + MOTOR_GIMBAL_SINE_OMEGA_RAD_S * period_s, MOTOR_GIMBAL_TWO_PI);
            break;

        default:
            gimbal_command.yaw.rate_rad_s = 0.0f;
            sine_phase_rad = 0.0f;
            sine_state = MOTOR_GIMBAL_SINE_IDLE;
            return;
    }

    // 角度与前馈来自同一相位，前馈开关不重置轨迹以便连续对比
    gimbal_command.yaw.source = GIMBAL_TARGET_ABSOLUTE;
    gimbal_command.yaw.angle_deg = sine_center_deg + MOTOR_GIMBAL_SINE_AMPLITUDE_DEG * sinf(sine_phase_rad);
    gimbal_command.yaw.rate_rad_s = gimbal_sine_feedforward_enabled ?
        MOTOR_GIMBAL_SINE_AMPLITUDE_DEG * MOTOR_GIMBAL_DEG_TO_RAD * MOTOR_GIMBAL_SINE_OMEGA_RAD_S *
        cosf(sine_phase_rad) : 0.0f;
}

#endif

/**
 * @brief 从容量为 1 的反馈邮箱取得一帧新反馈
 *
 * @param[in] queue 元素为 MOTOR_RM_Feedback_t 的反馈邮箱
 * @param[out] feedback 新反馈帧
 * @return 取得新帧时返回 true，邮箱为空或读取失败时返回 false
 */
static bool MotorGimbal_ReadFeedback(osMessageQueueId_t queue, MOTOR_RM_Feedback_t *feedback)
{
    return osMessageQueueGet(queue, feedback, NULL, 0U) == osOK;
}

/**
 * @brief 向容量为 1 的云台输出邮箱发布最新电流
 *
 * @param[in] output 本周期两轴输出电流
 * @return 邮箱重置并写入成功时返回 true，否则返回 false
 */
static bool MotorGimbal_PublishOutput(const Gimbal_Output_t *output)
{
    osMessageQueueId_t queue = task_runtime.msgq.gimbal_output;
    return osMessageQueueReset(queue) == osOK && osMessageQueuePut(queue, output, 0U, 0U) == osOK;
}
