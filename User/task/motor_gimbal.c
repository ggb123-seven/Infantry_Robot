#include "task/motor_gimbal.h"

#include <math.h>
#include <stddef.h>

#include "bsp/time.h"
#include "component/user_math.h"
#include "device/dr16.h"
#include "module/config.h"
#include "task/user_task.h"

/*
 * yaw 正弦跟随测试参数，角度单位为度，相位角速度单位为 rad/s：
 * - 中心取启动测试时的当前姿态，振幅 30°，前馈峰值为 π rad/s
 * - 2π 用于相位归一化，避免长时间累计相位导致浮点分辨率下降
 */
#define MOTOR_GIMBAL_SINE_AMPLITUDE_DEG (30.0f)
#define MOTOR_GIMBAL_SINE_OMEGA_RAD_S (6.0f)
#define MOTOR_GIMBAL_TWO_PI (6.28318530718f)
#define MOTOR_GIMBAL_DEG_TO_RAD (0.0174532925199f)

/*
 * DR16 云台命令参数，使用右拨杆和右摇杆，左侧输入继续分配给底盘
 * - CHANNEL_LIMIT：DR16 去中心后的满量程计数，来源为设备协议的 ±660
 * - DEADZONE：中心死区比例 5%，死区外重新归一化到满量程
 * - YAW_RATE_RAD_S、PITCH_RATE_RAD_S：满杆目标角速度均取 6 rad/s，对齐参考工程的遥控灵敏度
 * - YAW_ACCEL_DEG_S2、PITCH_ACCEL_DEG_S2：速度指令加速上限均取 60000 度每平方秒，限制满杆突变
 * - YAW_DECEL_DEG_S2、PITCH_DECEL_DEG_S2：速度指令减速上限均取 60000 度每平方秒，松杆后快速停止推进目标角
 * - RATE_SMOOTH_TIME_S：速度指令的一阶平滑时间常数为 4 毫秒，抑制小幅输入和换向突变
 * - RATE_SETTLE_DEG_S：速度误差不超过 0.1 度每秒且满足单周期变化上限时直接收敛，避免回中长尾
 *   实际过渡包含末端平滑时间，换向先减到零再反向加速，以上参数作为待实机调整的手感起点
 * - YAW_DIRECTION、PITCH_DIRECTION：通道到当前 INS 角度坐标的符号，初始均取反向，实机核对左右与抬头方向
 * - MAX_PERIOD_S：输入积分允许的最大间隔，沿用云台模块的 0.1 秒控制周期上限
 * 遥控帧龄使用 DR16_RECEIVER_OFFLINE_TIMEOUT_US，沿用设备层的 100 毫秒离线阈值
 */
#define MOTOR_GIMBAL_DR16_CHANNEL_LIMIT (660.0f)
#define MOTOR_GIMBAL_DR16_DEADZONE (0.05f)
#define MOTOR_GIMBAL_DR16_YAW_RATE_RAD_S (6.0f)
#define MOTOR_GIMBAL_DR16_PITCH_RATE_RAD_S (6.0f)
#define MOTOR_GIMBAL_DR16_YAW_ACCEL_DEG_S2 (60000.0f)
#define MOTOR_GIMBAL_DR16_PITCH_ACCEL_DEG_S2 (60000.0f)
#define MOTOR_GIMBAL_DR16_YAW_DECEL_DEG_S2 (60000.0f)
#define MOTOR_GIMBAL_DR16_PITCH_DECEL_DEG_S2 (60000.0f)
#define MOTOR_GIMBAL_DR16_RATE_SMOOTH_TIME_S (0.004f)
#define MOTOR_GIMBAL_DR16_RATE_SETTLE_DEG_S (0.1f)
#define MOTOR_GIMBAL_DR16_YAW_DIRECTION (-1.0f)
#define MOTOR_GIMBAL_DR16_PITCH_DIRECTION (-1.0f)
#define MOTOR_GIMBAL_MAX_PERIOD_S (0.1f)

/*
 * 遥控输入状态仅编排命令来源，闭环和设备故障继续由 Gimbal 模块管理
 * - WAIT_MIDDLE：启动、遥控失联或锁存故障后保持失能，等待收到有效的右拨杆中位
 * - READY：已确认中位，继续零输出，右拨杆上位后允许角度模式
 * - ACTIVE：仅右拨杆上位时发布角度命令，中位退回 READY，其他档位退回 WAIT_MIDDLE
 */
typedef enum
{
    MOTOR_GIMBAL_REMOTE_WAIT_MIDDLE = 0,
    MOTOR_GIMBAL_REMOTE_READY,
    MOTOR_GIMBAL_REMOTE_ACTIVE,
} MotorGimbal_RemoteState_t;

// 正弦测试待机时相位归零，yaw 进入角度闭环后才开始推进，异常状态回到待机
typedef enum
{
    MOTOR_GIMBAL_SINE_IDLE = 0,
    MOTOR_GIMBAL_SINE_RUNNING,
} MotorGimbal_SineState_t;

// 调试观测与命令入口，声明及使用方式见 motor_gimbal.h
Gimbal_t gimbal;
Gimbal_Command_t gimbal_command =
{
    .mode = GIMBAL_MODE_RELAX,
};
volatile bool gimbal_sine_test_enabled = false;
volatile bool gimbal_sine_feedforward_enabled = true;

static MotorGimbal_SineState_t sine_state;
static float sine_phase_rad;
static float sine_center_deg;

// 遥控缓存及输入状态仅由云台任务访问，不与 ISR 共享
static DR16_State_t remote_cache;
static bool remote_cache_valid;
static MotorGimbal_RemoteState_t remote_state;
static float remote_yaw_rate_deg_s;
static float remote_pitch_rate_deg_s;

static bool MotorGimbal_Init(void);
static void MotorGimbal_ApplyDR16Input(float period_s);
static float MotorGimbal_NormalizeChannel(int16_t channel);
static float MotorGimbal_UpdateRemoteRate(float current, float requested, float accel, float decel, float period_s);
static void MotorGimbal_UpdateSineTest(float period_s);
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
    const bool timing_valid = tick_frequency >= MOTOR_GIMBAL_FREQ && tick_frequency % MOTOR_GIMBAL_FREQ == 0U;
    const uint32_t delay_tick = timing_valid ? tick_frequency / MOTOR_GIMBAL_FREQ : 1U;

    // 等待统一装配完成后初始化云台模块，失败时只发布零电流
    osDelay(MOTOR_GIMBAL_INIT_DELAY);
    const bool initialized = MotorGimbal_Init();
    uint32_t tick = osKernelGetTickCount();
    uint32_t previous_tick = tick - delay_tick;
    while (true)
    {
        // 按实际调度间隔计算控制周期，错过节拍时由模块拒绝过长周期
        const uint32_t now_tick = osKernelGetTickCount();
        const float period_s = timing_valid ? (float)(now_tick - previous_tick) / (float)tick_frequency : 0.0f;
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
            const bool imu_received = imu_queue != NULL && osMessageQueueGet(imu_queue, &imu, NULL, 0U) == osOK;

            // 消费两轴最新反馈帧，邮箱为空时传 NULL 由模块检查旧帧时效
            MOTOR_RM_Feedback_t yaw_feedback;
            MOTOR_RM_Feedback_t pitch_feedback;
            const bool yaw_received = MotorGimbal_ReadFeedback(task_runtime.msgq.gimbal_yaw_feedback, &yaw_feedback);
            const bool pitch_received =
                MotorGimbal_ReadFeedback(task_runtime.msgq.gimbal_pitch_feedback, &pitch_feedback);
            Gimbal_UpdateFeedback(&gimbal, yaw_received ? &yaw_feedback : NULL,
                                  pitch_received ? &pitch_feedback : NULL, imu_received ? &imu : NULL, BSP_TIME_Get());

            // 遥控输入先决定是否使能，正弦测试同样受遥控失能约束，再执行模块闭环
            MotorGimbal_ApplyDR16Input(period_s);
            MotorGimbal_UpdateSineTest(period_s);
            Gimbal_Control(&gimbal, &gimbal_command, period_s);
            Gimbal_DumpOutput(&gimbal, &output);
        }

        // 发布失败时 CAN 任务按空邮箱输出零电流，同步让两轴停机，避免积分在零输出期间累计
        if (!MotorGimbal_PublishOutput(&output) && initialized)
        {
            Gimbal_Command_t relax_command =
            {
                .mode = GIMBAL_MODE_RELAX,
            };
            Gimbal_Control(&gimbal, &relax_command, period_s);
            remote_state = MOTOR_GIMBAL_REMOTE_WAIT_MIDDLE;
            gimbal_command.mode = GIMBAL_MODE_RELAX;
            gimbal_command.yaw.target_speed_rad_s = 0.0f;
            gimbal_command.pitch.target_speed_rad_s = 0.0f;
            remote_yaw_rate_deg_s = 0.0f;
            remote_pitch_rate_deg_s = 0.0f;
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
 * @brief 校验节拍频率并以整机配置初始化云台模块
 *
 * @return 节拍可精确表达控制周期且模块初始化成功时返回 true
 */
static bool MotorGimbal_Init(void)
{
    // 内核节拍须为控制频率整数倍，避免零周期或频率截断
    const uint32_t tick_frequency = osKernelGetTickFreq();
    if (tick_frequency < MOTOR_GIMBAL_FREQ || tick_frequency % MOTOR_GIMBAL_FREQ != 0U)
    {
        return false;
    }
    return Gimbal_Init(&gimbal, &gimbal_config, (float)MOTOR_GIMBAL_FREQ) == 0;
}

/**
 * @brief 消费独立遥控邮箱并将右拨杆和右摇杆转换为云台角度命令
 *
 * @param[in] period_s 实际控制间隔，单位秒，合法范围为 (0, 0.1]
 * @return 无返回值
 */
static void MotorGimbal_ApplyDR16Input(float period_s)
{
    // 先保存上周期速度再默认清零，所有失能和异常早退均立即撤销平滑状态与前馈
    const float previous_yaw_rate_deg_s = remote_yaw_rate_deg_s;
    const float previous_pitch_rate_deg_s = remote_pitch_rate_deg_s;
    remote_yaw_rate_deg_s = 0.0f;
    remote_pitch_rate_deg_s = 0.0f;
    gimbal_command.mode = GIMBAL_MODE_RELAX;
    gimbal_command.yaw.target_speed_rad_s = 0.0f;
    gimbal_command.pitch.target_speed_rad_s = 0.0f;
    const osMessageQueueId_t queue = task_runtime.msgq.dr16_gimbal_state;
    if (queue == NULL)
    {
        remote_cache_valid = false;
        remote_state = MOTOR_GIMBAL_REMOTE_WAIT_MIDDLE;
        return;
    }

    // 空邮箱保留最近合法帧的源时间，实际读取错误立即撤销缓存，避免沿用故障前的使能
    DR16_State_t received;
    const osStatus_t status = osMessageQueueGet(queue, &received, NULL, 0U);
    if (status == osOK)
    {
        remote_cache = received;
        remote_cache_valid = true;
    }
    else if (status != osErrorResource)
    {
        remote_cache_valid = false;
    }

    // 取得遥控快照后再采当前时间，避免高优先级接收任务抢占时将合法新帧误判为未来时间
    const uint64_t now_us = BSP_TIME_Get();

    // 帧龄、时基或控制周期异常均回到人工确认状态，恢复上位帧不会自动使能
    if (!remote_cache_valid || !remote_cache.header.online || now_us < remote_cache.header.last_online_time ||
        now_us - remote_cache.header.last_online_time > DR16_RECEIVER_OFFLINE_TIMEOUT_US ||
        !isfinite(period_s) || period_s <= 0.0f || period_s > MOTOR_GIMBAL_MAX_PERIOD_S)
    {
        remote_cache_valid = false;
        remote_state = MOTOR_GIMBAL_REMOTE_WAIT_MIDDLE;
        return;
    }

    // 锁存故障撤销两轴使能，电机反馈超时由模块单轴等待恢复，保留当前拨档状态
    if (gimbal.imu_state == GIMBAL_IMU_FAULT || gimbal.yaw.state == GIMBAL_AXIS_FAULT ||
        gimbal.pitch.state == GIMBAL_AXIS_FAULT)
    {
        remote_state = MOTOR_GIMBAL_REMOTE_WAIT_MIDDLE;
        return;
    }

    // 集中管理输入使能顺序，非法状态和中位以外的停机档都要求重新确认中位
    const DR16_SwitchPosition_t position = remote_cache.data.sw_r;
    switch (remote_state)
    {
        case MOTOR_GIMBAL_REMOTE_WAIT_MIDDLE:
            if (position == DR16_SWITCH_MIDDLE)
            {
                remote_state = MOTOR_GIMBAL_REMOTE_READY;
            }
            return;

        case MOTOR_GIMBAL_REMOTE_READY:
        case MOTOR_GIMBAL_REMOTE_ACTIVE:
            if (position == DR16_SWITCH_MIDDLE)
            {
                remote_state = MOTOR_GIMBAL_REMOTE_READY;
                return;
            }
            if (position != DR16_SWITCH_UP)
            {
                remote_state = MOTOR_GIMBAL_REMOTE_WAIT_MIDDLE;
                return;
            }
            remote_state = MOTOR_GIMBAL_REMOTE_ACTIVE;
            break;

        default:
            remote_state = MOTOR_GIMBAL_REMOTE_WAIT_MIDDLE;
            return;
    }

    // 用模块上周期实际限幅后的目标继续积分，避免持续推向机械边界时积累不可达目标
    // 按实际周期平滑摇杆速度的大小变化及换向，同一平滑速度用于目标积分和前馈，避免两者不一致
    // 首次使能及单轴失联等待期间不积分且前馈为零，由模块在起控或恢复时捕获当前姿态
    gimbal_command.mode = GIMBAL_MODE_ANGLE;
    // 正弦测试接管及退出交接周期不积累遥控速度，避免切回遥控时沿用被覆盖的前馈
    if (gimbal.yaw.state == GIMBAL_AXIS_RUNNING && isfinite(gimbal.yaw.target_angle_deg) &&
        !gimbal_sine_test_enabled && sine_state == MOTOR_GIMBAL_SINE_IDLE)
    {
        const float yaw_rate_deg_s = MOTOR_GIMBAL_DR16_YAW_DIRECTION *
                                     MotorGimbal_NormalizeChannel(remote_cache.data.ch_r_x) *
                                     MOTOR_GIMBAL_DR16_YAW_RATE_RAD_S * M_RAD2DEG_MULT;
        remote_yaw_rate_deg_s = MotorGimbal_UpdateRemoteRate(previous_yaw_rate_deg_s, yaw_rate_deg_s,
            MOTOR_GIMBAL_DR16_YAW_ACCEL_DEG_S2, MOTOR_GIMBAL_DR16_YAW_DECEL_DEG_S2, period_s);
        gimbal_command.yaw.target_angle_deg = gimbal.yaw.target_angle_deg + remote_yaw_rate_deg_s * period_s;
        gimbal_command.yaw.target_speed_rad_s = remote_yaw_rate_deg_s * MOTOR_GIMBAL_DEG_TO_RAD;
    }
    if (gimbal.pitch.state == GIMBAL_AXIS_RUNNING && isfinite(gimbal.pitch.target_angle_deg))
    {
        const float pitch_rate_deg_s = MOTOR_GIMBAL_DR16_PITCH_DIRECTION *
                                       MotorGimbal_NormalizeChannel(remote_cache.data.ch_r_y) *
                                       MOTOR_GIMBAL_DR16_PITCH_RATE_RAD_S * M_RAD2DEG_MULT;
        remote_pitch_rate_deg_s = MotorGimbal_UpdateRemoteRate(previous_pitch_rate_deg_s, pitch_rate_deg_s,
            MOTOR_GIMBAL_DR16_PITCH_ACCEL_DEG_S2, MOTOR_GIMBAL_DR16_PITCH_DECEL_DEG_S2, period_s);
        gimbal_command.pitch.target_angle_deg = gimbal.pitch.target_angle_deg + remote_pitch_rate_deg_s * period_s;
        gimbal_command.pitch.target_speed_rad_s = remote_pitch_rate_deg_s * MOTOR_GIMBAL_DEG_TO_RAD;
    }
}

/**
 * @brief 将遥控速度指令按加减速上限平滑到目标值，换向时先经过零速
 *
 * @param[in] current 上周期遥控角速度，单位度每秒，须为有限值
 * @param[in] requested 本周期摇杆请求角速度，单位度每秒，须为有限值
 * @param[in] accel 加速上限，单位度每平方秒，须为有限正值
 * @param[in] decel 减速上限，单位度每平方秒，须为有限正值
 * @param[in] period_s 已校验的控制周期，单位秒，范围为 (0, 0.1]
 * @return 本周期遥控角速度，单位度每秒
 */
static float MotorGimbal_UpdateRemoteRate(float current, float requested, float accel, float decel, float period_s)
{
    // 换向先以零速为本周期边界，避免同一步骤越过零速后仍沿用较大的减速上限
    const float target = current * requested < 0.0f ? 0.0f : requested;
    const float rate_limit = fabsf(target) < fabsf(current) ? decel : accel;
    const float max_step = rate_limit * period_s;
    const float difference = fabsf(target - current);

    // 小误差有限时间收敛且仍受变化上限约束，保证松杆后目标角能停止推进
    if (difference <= MOTOR_GIMBAL_DR16_RATE_SETTLE_DEG_S && difference <= max_step)
    {
        return target;
    }

    // 小幅输入和接近目标时也保留过渡，实际周期决定平滑比例，较大变化继续交给通用斜坡限幅
    const float blend = 1.0f - expf(-period_s / MOTOR_GIMBAL_DR16_RATE_SMOOTH_TIME_S);
    return RampTo(current, target, fminf(max_step, difference * blend));
}

/**
 * @brief 将 DR16 右摇杆原始计数限幅并去除中心死区
 *
 * @param[in] channel 去中心后的原始计数，典型范围为 [-660, 660]
 * @return 死区内为零，死区外线性映射到 [-1, 1]
 */
static float MotorGimbal_NormalizeChannel(int16_t channel)
{
    const float value = fmaxf(-1.0f, fminf(1.0f, (float)channel / MOTOR_GIMBAL_DR16_CHANNEL_LIMIT));
    if (fabsf(value) <= MOTOR_GIMBAL_DR16_DEADZONE)
    {
        return 0.0f;
    }
    return copysignf((fabsf(value) - MOTOR_GIMBAL_DR16_DEADZONE) / (1.0f - MOTOR_GIMBAL_DR16_DEADZONE), value);
}

/**
 * @brief yaw 处于角度闭环时生成正弦目标及其解析角速度前馈
 *
 * 测试只改写 yaw 命令；关闭测试或 yaw 退出角度闭环后保留最后目标角并清零前馈
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
                // 退出测试时撤销前馈，目标角停在最后位置
                gimbal_command.yaw.target_speed_rad_s = 0.0f;
                sine_phase_rad = 0.0f;
                sine_state = MOTOR_GIMBAL_SINE_IDLE;
                return;
            }
            sine_phase_rad = fmodf(sine_phase_rad + MOTOR_GIMBAL_SINE_OMEGA_RAD_S * period_s, MOTOR_GIMBAL_TWO_PI);
            break;

        default:
            gimbal_command.yaw.target_speed_rad_s = 0.0f;
            sine_phase_rad = 0.0f;
            sine_state = MOTOR_GIMBAL_SINE_IDLE;
            return;
    }

    // 角度与前馈来自同一相位，前馈开关不重置轨迹以便连续对比
    gimbal_command.yaw.target_angle_deg = sine_center_deg + MOTOR_GIMBAL_SINE_AMPLITUDE_DEG * sinf(sine_phase_rad);
    gimbal_command.yaw.target_speed_rad_s = gimbal_sine_feedforward_enabled ?
        MOTOR_GIMBAL_SINE_AMPLITUDE_DEG * MOTOR_GIMBAL_DEG_TO_RAD * MOTOR_GIMBAL_SINE_OMEGA_RAD_S *
        cosf(sine_phase_rad) : 0.0f;
}

/**
 * @brief 从容量为 1 的反馈邮箱取得一帧新反馈
 *
 * @param[in] queue 元素为 MOTOR_RM_Feedback_t 的反馈邮箱
 * @param[out] feedback 新反馈帧
 * @return 取得新帧时返回 true，邮箱为空、句柄无效或读取失败时返回 false
 */
static bool MotorGimbal_ReadFeedback(osMessageQueueId_t queue, MOTOR_RM_Feedback_t *feedback)
{
    return queue != NULL && osMessageQueueGet(queue, feedback, NULL, 0U) == osOK;
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
    return queue != NULL && osMessageQueueReset(queue) == osOK && osMessageQueuePut(queue, output, 0U, 0U) == osOK;
}
