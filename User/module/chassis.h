/*
 * 底盘模块对外接口
 *
 * 消费 CAN 反馈与遥控归一化运动指令，经斜坡、运动学混控和四轮速度环输出四路电流
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ----------------------------------------------------------------- */
#include "component/mixer.h"
#include "component/pid.h"
#include "device/can_devices.h"
#include "module/power.h"

/* Exported constants ------------------------------------------------------- */
#define CHASSIS_WHEEL_COUNT (4U) /* 底盘驱动轮数量，电调 ID1~4 */

/* Exported variables ------------------------------------------------------- */
/*
 * 四轮速度前馈的在线调参入口，依次对应左前、左后、右后、右前
 * 系数单位 A/rpm，零表示关闭；初值来自四轮架空正向 30 rpm 实测整定，其他转速和负载需另行验证
 * 前馈电流为本轮系数乘以本周期目标轮速，与速度 PID 相加后统一滤波和限流，任一系数非法时四轮待机
 */
extern volatile float chassis_speed_feedforward_gain_a_per_rpm[CHASSIS_WHEEL_COUNT];

/* Exported types ----------------------------------------------------------- */
/* 底盘模式：待机零电流，独立模式含零速刹车，跟随和自旋均以云台为平移坐标 */
typedef enum
{
    CHASSIS_MODE_RELAX = 0,     /* 待机，零电流 */
    CHASSIS_MODE_INDEPENDENT,   /* 独立模式，含零速刹车 */
    CHASSIS_MODE_FOLLOW_GIMBAL, /* 跟随云台朝向，平移以云台为坐标 */
    CHASSIS_MODE_SPIN,          /* 小陀螺自旋，平移以云台为坐标 */
} Chassis_Mode_t;

/* 单轮状态：离线恢复后自动重新预置 */
typedef enum
{
    CHASSIS_WHEEL_RELAX = 0, /* 待机，零电流 */
    CHASSIS_WHEEL_RUNNING,   /* 闭环运行 */
    CHASSIS_WHEEL_OFFLINE,   /* 反馈离线 */
} Chassis_WheelState_t;

/* 底盘配置，可由调试器在线修改，模块每周期校验并采纳 */
typedef struct
{
    Mixer_Mode_t layout;             /* 机械布局，仅初始化时生效，当前为 MIXER_OMNICROSS */
    float max_wheel_speed_rpm;       /* 输出轴目标转速绝对值上限，单位：rpm */
    float wheel_accel_rpm_s;         /* 归一化指令斜坡的单轮转速变化率，单位：rpm/s */
    float reverse_release_speed_rpm; /* 换向放行阈值，单位：rpm，换向置零逻辑已删除，当前不参与控制 */
    float speed_filter_cutoff_hz;    /* 反馈二阶低通截止频率，单位：Hz，非正时直通，仅初始化生效 */
    float current_filter_cutoff_hz;  /* 电流二阶低通截止频率，单位：Hz，非正时直通，仅初始化生效 */
    float max_current_a;             /* 最终转子侧电流绝对值上限，单位：A */
    KPID_Params_t wheel_speed_pid;   /* 四轮共用速度环参数，输入 rpm、输出 A，运行中修改四轮同时生效 */
    KPID_Params_t follow_pid;        /* 跟随角度环参数，输入 rad、输出归一化转向，参数改变时清除跟随历史 */
} Chassis_Config_t;

/* 底盘命令：跟随与自旋的遥控平移以云台为坐标，由 Control 转换到车体坐标，命令本身不被模块回写 */
typedef struct
{
    Chassis_Mode_t mode;         /* 控制模式，未知值按待机处理 */
    MoveVector_t move;           /* 归一化运动向量，vx 前向、vy 左向、wz 俯视逆时针为正 */
    float wheel_speed_scale_rpm; /* 归一化轮速 1 对应的输出轴转速，单位：rpm */
} Chassis_Command_t;

/* 单轮运行数据，调试器可直接观察 */
typedef struct
{
    Chassis_WheelState_t state;       /* 单轮状态，非运行状态电流为零 */
    float measured_speed_rpm;         /* 本周期输出轴实测转速，单位：rpm，反馈无效时为 NaN */
    float filtered_speed_rpm;         /* 送入速度环的滤波转速，单位：rpm */
    float requested_speed_rpm;        /* 运动学解算目标，单位：rpm */
    float target_speed_rpm;           /* 实际进入速度环和前馈的目标，单位：rpm，运行时等于解算目标 */
    float output_current_a;           /* 本周期转子侧电流，单位：A */
    KPID_t speed_pid;                 /* 本轮积分和微分历史，参数指向实例中的公共副本 */
    LowPassFilter2p_t speed_filter;   /* 本轮反馈二阶低通状态 */
    LowPassFilter2p_t current_filter; /* 本轮电流二阶低通状态 */
} Chassis_Wheel_t;

/* 底盘实例，本工程使用单一实例，初始化后不得整体复制，避免内部 PID 参数指针失效 */
typedef struct
{
    Chassis_Config_t config;                    /* 默认配置的可调副本 */
    bool initialized;                           /* 混合器、跟随 PID、四轮 PID 与滤波器全部初始化成功 */
    bool config_valid;                          /* 本周期配置与四轮前馈系数校验结果，非法时四轮零输出 */
    KPID_Params_t applied_wheel_speed_pid;      /* 四路 PID 共同引用的参数副本，变更时清空四轮积分 */
    Chassis_Mode_t mode;                        /* 本周期实际采用的控制模式 */
    MoveVector_t ramped_move;                   /* 斜坡后的运动向量，待机和独立模式为零 */
    Mixer_t mixer;                              /* 运动学混合器，仅初始化时选择布局 */
    Chassis_Wheel_t wheel[CHASSIS_WHEEL_COUNT]; /* 电调 ID1~4，依次为左前、左后、右后、右前 */
    float follow_relative_angle_deg;            /* 云台相对车头的最短机械角，单位：度，反馈无效时为非数值 */
    float follow_relative_speed_rad_s;          /* 方向修正后的云台相对底盘角速度，单位：rad/s */
    float follow_wz;                            /* 本周期跟随 PID 的归一化转向请求，不代表实测车体角速度 */
    Power_t power;                              /* 功率限制实例，配置与观测字段见 power.h */
} Chassis_t;

/* Exported functions ------------------------------------------------------- */
/**
 * @brief 复制配置并初始化运动学、跟随 PID、四轮 PID 与滤波器
 *
 * @param chassis 待初始化的底盘实例，初始化后不得整体复制
 * @param config 默认配置
 * @param power_config 功率限制默认配置
 * @param frequency_hz 控制采样频率，单位：Hz
 * @return int8_t 成功返回 0，失败返回 -1 并保持未初始化和零输出
 */
int8_t Chassis_Init(Chassis_t *chassis, const Chassis_Config_t *config, const Power_Config_t *power_config,
                    float frequency_hz);

/**
 * @brief 从 CAN 快照更新四轮实测转速
 *
 * @param chassis 底盘实例
 * @param can 本周期 CAN 快照，无新消息时传 NULL，缺失或无效反馈写入 NaN
 * @return 无返回值
 */
void Chassis_UpdateFeedback(Chassis_t *chassis, const CANDevices_Snapshot_t *can);

/**
 * @brief 推进底盘模式和每轮状态机并计算电流
 *
 * @param chassis 底盘实例
 * @param command 归一化控制命令，无命令时传 NULL，跟随及自旋平移使用云台坐标
 * @param period_s 实测控制周期，单位：s，须在 (0, 0.1] 内
 * @return 无返回值
 */
void Chassis_Control(Chassis_t *chassis, const Chassis_Command_t *command, float period_s);

/**
 * @brief 在反馈边界校验云台帧并更新跟随角度与相对角速度，后续控制只使用本次校验结果
 *
 * @param chassis 底盘实例
 * @param feedback 最近一次云台电机反馈，未取得时传 NULL
 * @param now_us 读取反馈后的单调时间，单位：微秒
 * @param zero_deg 已标定的方向修正后编码器零点，单位：度
 * @param reversed 云台编码器是否反向
 * @return 无返回值
 */
void Chassis_UpdateGimbalFeedback(Chassis_t *chassis, const MOTOR_RM_Feedback_t *feedback,
                                   uint64_t now_us, float zero_deg, bool reversed);

/**
 * @brief 导出四路电流，保留调用方设置的命令序号
 *
 * @param chassis 底盘实例，未初始化时输出全零
 * @param output CAN 电流命令，仅写 current_a[0~3]
 * @return 无返回值
 */
void Chassis_DumpOutput(const Chassis_t *chassis, CANDevices_Command_t *output);

/**
 * @brief 查询本周期是否至少一轮取得有效在线反馈，供云台判断底盘能否主动旋转
 *
 * @param chassis 已完成本周期反馈更新的底盘实例
 * @return bool 任一轮实测转速有效时返回 true，四轮全部离线或无新快照时返回 false
 */
bool Chassis_HasOnlineWheel(const Chassis_t *chassis);

#ifdef __cplusplus
}
#endif
