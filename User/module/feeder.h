#pragma once

#include "component/pid.h"
#include "device/motor_rm.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/*
 * 拨弹运行状态：
 * - FEEDER_RELAX：待机零输出，进给倍率为零或命令回到待机档时进入
 * - FEEDER_RUNNING：连发速度环或手动目标角串级控制运行
 * - FEEDER_OFFLINE：反馈超时或角度连续性丢失，零输出等待有效反馈后自动重新起控
 * - FEEDER_FAULT：初始化失败、配置非法、周期异常或控制量溢出，零输出锁存至命令回到待机档
 */
typedef enum
{
    FEEDER_RELAX = 0,
    FEEDER_RUNNING,
    FEEDER_OFFLINE,
    FEEDER_FAULT,
} Feeder_State_t;

// 拨弹命令档位：离线档不解除故障锁存，待机档可人工解除故障，运行档允许当前环路结构输出
typedef enum
{
    FEEDER_CMD_OFFLINE = 0,
    FEEDER_CMD_RELAX,
    FEEDER_CMD_RUN,
} Feeder_CmdMode_t;

/*
 * 拨弹进给模式，由 EmberProbe 直接写 feeder_fire_mode 切换，上电默认连发速度环
 * 自动档为连发：只走速度环，摇杆倍率直接映射输出轴转速，角度环不参与
 * 模式 1 暂用于角度环调试：右拨杆下位使能串级控制，进入时捕获当前位置，不自动执行 45° 单发
 */
#define FEEDER_FIRE_MODE_AUTO (0U)
#define FEEDER_FIRE_MODE_SINGLE (1U)

// 进给模式开关，运行期写入 0 或 1 即切换，控制周期入口只读一次形成本周期快照
extern volatile uint8_t feeder_fire_mode;
// 模式 1 的输出轴目标角，单位度，右拨杆下位后由 EmberProbe 写入，每周期入口只读一次
extern volatile float feeder_angle_target_deg;

/*
 * 拨弹命令由命令层每周期根据遥控快照重新生成，模块只读取不修改
 * mode 为命令档位，拨杆非法或遥控不新鲜时保持离线，只有待机档能解除故障锁存
 * speed_ratio 沿用命令层既有字段名，语义为连发归一化进给倍率，调用方保证有限且位于 [-1, 1]
 * 连发档取绝对值映射速度目标，方向始终由 config.direction 决定；模式 1 忽略该字段
 * trigger_level 为右拨杆下位电平，暂时保留给后续单发策略，当前运行使能由 mode 统一表达
 */
typedef struct
{
    Feeder_CmdMode_t mode;
    float speed_ratio;
    bool trigger_level;
} Feeder_Command_t;

/*
 * 拨弹可调配置，初始化复制到实例，运行中通过 EmberProbe 修改实例副本
 * tooth_count 为一圈齿数，须为有限值且至少为 2，暂时保留给后续单发步距
 * feed_hz 为每秒进给齿数上限，须大于零且不超过采样频率，暂时保留给后续进给策略
 * direction 为角度递增方向，只接受 -1 或 +1；默认 -1 沿用参考工程正常拨弹方向
 * max_speed_rpm 为输出轴速度目标上限，单位 rpm
 * max_current_a 为转子电流上限，单位 A，范围 (0,10]
 * position_pid 为角度外环参数，输入输出轴弧度，输出 rad/s，range 固定为 2π，输出限幅不得超过速度上限换算值
 * speed_pid 为速度内环参数，输入输出轴 rpm，输出 A，range 固定为零，输出限幅不得超过电流上限
 * speed_feedforward_gain_a_per_rpm 为目标逐周期差分的电流前馈系数，单位 A/rpm，非负，零表示关闭
 * 两环的 k 须为正，p/i/d/i_limit 非负，out_limit 大于 0.000001，微分截止频率小于采样频率一半
 */
typedef struct
{
    float tooth_count;
    float feed_hz;
    float direction;
    float max_speed_rpm;
    float max_current_a;
    KPID_Params_t position_pid;
    KPID_Params_t speed_pid;
    float speed_feedforward_gain_a_per_rpm;
} Feeder_Config_t;

/*
 * 拨弹实例由拨弹任务独占，初始化后不得整体复制，内部 PID 参数指针引用实例内持久快照
 * config 为可调副本，cycle_config 为本周期集中校验的持久快照，两环参数指针长期指向后者
 * initialized 与 config_valid 分别表示初始化结果与本周期配置校验结果
 * state 为唯一运行状态，非运行状态由 Feeder_DumpOutput 统一输出零电流
 * feedback 为最近一帧输出轴反馈，feedback_valid 表示角度连续有效且未超过 10 ms
 * 角度外环依赖角度连续性，反馈无效或超时进入离线态，恢复后按当前位置重新捕获目标
 * measured_angle_rad、measured_angle_deg 为输出轴单圈角度，角度不连续或超时为非数值
 * measured_speed_rpm 为速度内环反馈，单位 rpm；帧未超时即为数值，角度不连续不影响连发速度环
 * measured_speed_rad_s 为角度外环微分输入，单位 rad/s
 * target_angle_rad、target_angle_deg 为本周期采用的手动目标角，仅串级模式维护，速度环模式保持上次值
 * target_speed_rpm 为速度内环目标，单位 rpm；串级时来自角度外环输出，连发时来自摇杆倍率映射
 * single_active 暂存模式 1 与右拨杆下位的组合使能，用于识别环路切换并安全捕获当前位置
 * phase_s、trigger_level_last、single_elapsed_s、single_in_position_rad、single_timeout_s 暂时保留给后续单发策略
 * output_current_a 为候选转子电流，单位 A，非运行时保留但由输出闸门归零
 * position_pid、speed_pid 分别保存外环和内环控制历史，不由任务或其他模块直接执行
 * frequency_hz 为初始化时的控制采样频率，单位 Hz
 */
typedef struct
{
    Feeder_Config_t config;
    Feeder_Config_t cycle_config;
    bool initialized;
    bool config_valid;
    Feeder_State_t state;
    MOTOR_RM_Feedback_t feedback;
    bool feedback_valid;
    bool trigger_level_last;
    bool single_active;
    float measured_angle_rad;
    float measured_angle_deg;
    float measured_speed_rpm;
    float measured_speed_rad_s;
    float target_angle_rad;
    float target_angle_deg;
    float target_speed_rpm;
    float single_elapsed_s;
    float single_in_position_rad;
    float single_timeout_s;
    double phase_s;
    float output_current_a;
    KPID_t position_pid;
    KPID_t speed_pid;
    float frequency_hz;
} Feeder_t;

/**
 * @brief 复制默认拨弹配置并初始化角度外环和速度内环
 *
 * @param[out] feeder 拨弹实例，调用方保证非空
 * @param[in] frequency_hz 调度频率，单位 Hz，须为有限正值
 * @return 成功返回 0，失败返回 -1 并保持安全状态
 */
int8_t Feeder_Init(Feeder_t *feeder, float frequency_hz);

/**
 * @brief 在输入边界接收新反馈并按源时间检查缓存帧龄
 *
 * @param[in,out] feeder 拨弹实例，调用方保证非空
 * @param[in] feedback 驱动反馈，调用方保证非空，转速为输出轴 rpm，角度为 rad
 * @param[in] received 驱动取得完整新反馈时为 true
 * @param[in] now_us 读取反馈后的完整单调微秒时间
 * @return 无返回值
 */
void Feeder_UpdateFeedback(Feeder_t *feeder, const MOTOR_RM_Feedback_t *feedback, bool received, uint64_t now_us);

/**
 * @brief 校验周期配置，按命令档位推进状态机并按环路结构执行速度环或角度速度串级控制
 *
 * 连发时按进给倍率直接映射输出轴转速目标；模式 1 仅在右拨杆下位时采用 EmberProbe 手动目标角
 * 模式 1 进入时捕获当前位置且不自动推进 45°，右拨杆离开下位后回到待机
 *
 * @param[in,out] feeder 拨弹实例，调用方保证非空
 * @param[in] command 本周期拨弹命令，无命令时传 NULL，按离线档处理且不解除故障锁存
 * @param[in] period_s 实测控制周期，单位秒，合法范围 (0,0.1]
 * @return 无返回值
 */
void Feeder_Control(Feeder_t *feeder, const Feeder_Command_t *command, float period_s);

/**
 * @brief 根据唯一运行状态导出电流，其他状态统一输出零
 *
 * @param[in] feeder 拨弹实例，调用方保证非空
 * @return 转子侧目标电流，单位 A
 */
float Feeder_DumpOutput(const Feeder_t *feeder);

#ifdef __cplusplus
}
#endif
