#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "component/mixer.h"
#include "component/pid.h"
#include "device/can_devices.h"

#define CHASSIS_WHEEL_COUNT (4U)

/*
 * 底盘模式：待机复位并输出零电流，独立模式按车体坐标向量运行
 * 跟随云台与自旋均按云台坐标解释平移，自旋另叠加云台坐标系的固定角速度
 * 跟随模式由角度环把车头对齐云台，自旋模式不修正车头朝向
 */
typedef enum
{
    CHASSIS_MODE_RELAX = 0,
    CHASSIS_MODE_INDEPENDENT,
    CHASSIS_MODE_FOLLOW_GIMBAL,
    CHASSIS_MODE_SPIN,
} Chassis_Mode_t;

// 单轮状态：待机、闭环运行、反馈离线；离线恢复后自动重新预置
typedef enum
{
    CHASSIS_WHEEL_RELAX = 0,
    CHASSIS_WHEEL_RUNNING,
    CHASSIS_WHEEL_OFFLINE,
} Chassis_WheelState_t;

/*
 * 底盘配置，运行中可由调试器修改，模块每周期校验并采纳：
 * - layout：机械布局，仅初始化时生效，当前为 MIXER_OMNICROSS
 * - max_wheel_speed_rpm：输出轴目标转速绝对值上限，单位 rpm，须大于 0
 * - wheel_accel_rpm_s：目标斜坡变化率，单位 rpm/s，须大于 0
 * - reverse_release_speed_rpm：允许反向目标的实测转速绝对值阈值，单位 rpm，须非负
 * - speed_filter_cutoff_hz：反馈二阶低通截止频率，单位 Hz，非正时直通，仅初始化时生效
 * - current_filter_cutoff_hz：电流二阶低通截止频率，单位 Hz，非正时直通，仅初始化时生效
 * - max_current_a：最终转子侧电流绝对值上限，单位 A，须大于 0
 * - wheel_speed_pid：四轮共用速度 PID，输入 rpm、输出 A，运行中修改后四轮同时生效
 *   k 须大于 0，p/i/d/i_limit/out_limit/range 须非负，截止频率须小于采样频率一半
 * - follow_pid：跟随角度环，输入云台相对车头的角度误差 rad、输出归一化转向
 *   误差按 2π 环绕，out_limit 须在 (0, 1] 内，参数改变时清除跟随历史
 *   父树实测正轮速使相对角增加，正角差经本环产生负转向请求，据此确定反馈符号
 *
 * 独立模式使用 wheel_accel_rpm_s 与 reverse_release_speed_rpm 约束单轮目标
 * 跟随与自旋模式改用 wheel_accel_rpm_s 约束归一化指令斜坡，单轮目标不再二次限速
 */
typedef struct
{
    Mixer_Mode_t layout;
    float max_wheel_speed_rpm;
    float wheel_accel_rpm_s;
    float reverse_release_speed_rpm;
    float speed_filter_cutoff_hz;
    float current_filter_cutoff_hz;
    float max_current_a;
    KPID_Params_t wheel_speed_pid;
    KPID_Params_t follow_pid;
} Chassis_Config_t;

/*
 * 底盘命令：
 * - mode：控制模式，未知值按待机处理
 * - move：归一化运动向量，vx 前向、vy 左向、wz 俯视逆时针为正，约定范围 [-1, 1]
 *   独立模式按车体坐标解释，跟随与自旋模式按云台坐标解释并由模块旋转到车体坐标
 * - wheel_speed_scale_rpm：归一化轮速 1 对应的输出轴 rpm，须非负，超过配置上限时限幅
 */
typedef struct
{
    Chassis_Mode_t mode;
    MoveVector_t move;
    float wheel_speed_scale_rpm;
} Chassis_Command_t;

/*
 * 单轮运行数据，调试器可直接观察：
 * - state：单轮状态，非运行状态电流为 0
 * - measured_speed_rpm：本周期输出轴实测转速，单位 rpm，反馈无效时为 NaN
 * - filtered_speed_rpm：送入速度环的滤波转速，单位 rpm
 * - requested_speed_rpm：运动学解算目标，单位 rpm
 * - target_speed_rpm：限幅与必要时斜坡后实际进入速度环的目标，单位 rpm
 * - output_current_a：本周期转子侧电流，单位 A
 * - speed_pid：本轮独立的积分和微分历史，参数指向实例中的公共副本
 * - speed_filter、current_filter：本轮反馈和电流二阶低通状态
 */
typedef struct
{
    Chassis_WheelState_t state;
    float measured_speed_rpm;
    float filtered_speed_rpm;
    float requested_speed_rpm;
    float target_speed_rpm;
    float output_current_a;
    KPID_t speed_pid;
    LowPassFilter2p_t speed_filter;
    LowPassFilter2p_t current_filter;
} Chassis_Wheel_t;

/*
 * 底盘实例，初始化后不得整体复制，避免内部 PID 参数指针失效：
 * - config：默认配置的可调副本
 * - initialized：混合器、跟随 PID、四路 PID 和滤波器全部初始化成功时为 true
 * - config_valid：本周期配置校验结果，非法时四轮零输出
 * - applied_wheel_speed_pid：四路 PID 共同引用的参数副本，变更时清空四轮积分
 * - mode：本周期实际采用的控制模式
 * - ramped_move：跟随与自旋模式斜坡后的归一化运动向量，其他模式为零
 * - mixer：运动学混合器，仅初始化时选择布局
 * - wheel[0~3]：电调 ID1~4 的单轮状态，依次为左前、左后、右后、右前
 * - follow_relative_angle_deg：云台相对车头的最短机械角，单位度，反馈无效时为 NaN
 * - follow_relative_speed_rad_s：方向修正后的云台相对底盘角速度，单位 rad/s
 * - follow_wz：本周期跟随角度环的归一化转向请求，不代表实测车体角速度
 */
typedef struct
{
    Chassis_Config_t config;
    bool initialized;
    bool config_valid;
    KPID_Params_t applied_wheel_speed_pid;
    Chassis_Mode_t mode;
    MoveVector_t ramped_move;
    Mixer_t mixer;
    Chassis_Wheel_t wheel[CHASSIS_WHEEL_COUNT];
    float follow_relative_angle_deg;
    float follow_relative_speed_rad_s;
    float follow_wz;
} Chassis_t;

/**
 * @brief 复制配置并初始化运动学、跟随 PID、四轮 PID 与滤波器
 *
 * @param[out] chassis 待初始化的底盘实例，初始化后不得整体复制
 * @param[in] config 默认配置，允许为 NULL，空指针时初始化失败
 * @param[in] frequency_hz 控制采样频率，单位 Hz，须为有限正值
 * @return 成功返回 0，失败返回 -1 并保持未初始化和零输出
 */
int8_t Chassis_Init(Chassis_t *chassis, const Chassis_Config_t *config, float frequency_hz);

/**
 * @brief 从 CAN 快照更新四轮实测转速
 *
 * @param[in,out] chassis 底盘实例
 * @param[in] can 本周期 CAN 快照，允许为 NULL，缺失或无效反馈写入 NaN
 * @return 无返回值
 */
void Chassis_UpdateFeedback(Chassis_t *chassis, const CANDevices_Snapshot_t *can);

/**
 * @brief 校验云台 yaw 反馈并更新车头相对角度与相对角速度，后续控制只使用本次结果
 *
 * @param[in,out] chassis 底盘实例
 * @param[in] feedback 最近一次 yaw 电机反馈，未取得时传 NULL
 * @param[in] now_us 读取反馈后的单调时间，单位微秒
 * @param[in] zero_deg 已标定的方向修正后编码器零点，单位度，须为有限值
 * @param[in] reversed 云台编码器方向是否反向
 * @return 无返回值
 */
void Chassis_UpdateGimbalFeedback(Chassis_t *chassis, const MOTOR_RM_Feedback_t *feedback,
                                  uint64_t now_us, float zero_deg, bool reversed);

/**
 * @brief 推进底盘模式和每轮状态机并计算电流
 *
 * @param[in,out] chassis 底盘实例
 * @param[in] command 归一化控制命令，允许为 NULL，非法命令按待机处理
 * @param[in] period_s 实测控制周期，单位 s，须在 (0, 0.1] 内
 * @return 无返回值
 */
void Chassis_Control(Chassis_t *chassis, const Chassis_Command_t *command, float period_s);

/**
 * @brief 导出四路电流，保留调用方设置的命令序号
 *
 * @param[in] chassis 底盘实例，允许为 NULL，未初始化时输出全零
 * @param[out] output CAN 电流命令，仅写 current_a[0~3]
 * @return 无返回值
 */
void Chassis_DumpOutput(const Chassis_t *chassis, CANDevices_Command_t *output);

#ifdef __cplusplus
}
#endif
