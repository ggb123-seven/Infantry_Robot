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
 * - FEEDER_RUNNING：速度环、逐发单角度环或角度速度串级控制运行
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
 * 拨弹接口保留三种进给模式，遥控默认选择五连发
 * 连发要求速度环使能，倍率映射输出轴转速；单发与五连发按使能接口选择单角度环或串级控制
 * 固定发数完成后持续请求不重复，释放请求取消剩余发数并允许重新触发
 */
#define FEEDER_FIRE_MODE_AUTO (0U)
#define FEEDER_FIRE_MODE_SINGLE (1U)
#define FEEDER_FIRE_MODE_BURST (2U)

// 本周期实际采用的进给模式，仅供观测，命令层为唯一选择来源
extern volatile uint8_t feeder_fire_mode;
// 单发输出轴目标角，单位度，仅供观测，由模块捕获及逐齿推进
extern volatile float feeder_angle_target_deg;

/*
 * 拨弹环路在线使能接口，每个控制周期在入口各读取一次，复位默认只启用角度环
 * angle=true、speed=false：单发和五连发的角度环直接输出电流 A，不计算速度环或速度前馈
 * angle=true、speed=true：单发和五连发恢复角度速度串级，角度环输出 rad/s
 * angle=false、speed=true：仅速度环，连续进给使用命令倍率，定量发射无速度倍率时保持撤驱
 * angle=false、speed=false：全部撤驱；连续进给在速度环关闭时也撤驱，不把速度目标解释为电流
 * 环路结构改变会取消本组目标并清除控制历史，定量发射须重新释放请求后才能触发
 */
extern volatile bool feeder_angle_loop_enabled;
extern volatile bool feeder_speed_loop_enabled;

/*
 * 拨弹命令由命令层每周期根据遥控快照重新生成，模块只读取不修改
 * mode 为命令档位，拨杆非法或遥控不新鲜时保持离线，只有待机档能解除故障锁存
 * speed_ratio 沿用命令层既有字段名，语义为连发归一化转速倍率，调用方保证有限且位于 [-1, 1]
 * 连发档保留倍率正负号，目标方向由 config.direction 与倍率符号共同决定
 * fire_mode 为命令层已校验的进给模式，0 连发、1 单发、2 五连发
 * single_fire 为单发或五连发的持续开火请求，释放立即取消未完成进给，完成后须释放再触发
 */
typedef struct
{
    Feeder_CmdMode_t mode;
    float speed_ratio;
    uint8_t fire_mode;
    bool single_fire;
} Feeder_Command_t;

/*
 * 拨弹可调配置，初始化复制到实例，运行中通过 EmberProbe 修改实例副本
 * tooth_count 为一圈齿数，须为有限值且至少为 2，单发步距为 360 度除以齿数
 * feed_hz 为单发重复时每秒进给齿数上限，须大于零且不超过采样频率，实际推进还须等待到位
 * direction 为角度递增方向，只接受 -1 或 +1；默认 -1 沿用参考工程正常拨弹方向
 * max_speed_rpm 为输出轴速度目标上限，单位 rpm
 * max_current_a 为转子电流上限，单位 A，范围 (0,10]
 * position_pid 输入输出轴弧度，range 固定为 2π，串级输出 rad/s，单角度环输出 A，切换后须按单位调参
 * 串级的角度输出限幅不得超过速度上限换算值，单角度环的有效限幅取其 out_limit 与 max_current_a 的较小值
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
 * 本项目仅使用一个拨弹实例，结构体只保留 EmberProbe 配置、运行状态和必要控制观测
 * config 为 EmberProbe 可写配置副本，Control 每周期复制并校验后供控制器使用
 * config_valid 为最近一次 Control 周期的配置校验结果
 * state 为唯一运行状态，非运行状态由 Feeder_DumpOutput 统一输出零电流
 * measured_angle_deg 为输出轴单圈角度，单位度；角度无效或超时时为非数值
 * measured_speed_rpm 为输出轴实测转速，单位 rpm；反馈超时时为非数值
 * target_angle_deg 为角度模式采用的目标，单位度
 * target_speed_rpm 为速度内环目标，单位 rpm，单角度环时固定为零，不代表角度环撤驱
 * output_current_a 为限幅后的候选转子电流，单位 A，非运行状态由输出闸门归零
 */
typedef struct
{
    Feeder_Config_t config;
    bool config_valid;
    Feeder_State_t state;
    float measured_angle_deg;
    float measured_speed_rpm;
    float target_angle_deg;
    float target_speed_rpm;
    float output_current_a;
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
 * @brief 校验周期配置，按环路使能接口执行速度环、单角度环或逐发串级控制
 *
 * 单发及五连发使能或反馈恢复时捕获当前位置并等待请求释放，触发后逐齿执行固定发数
 * 到位须满足输出轴误差不超过 0.5 度、绝对转速不超过 1 rpm 并持续 20 毫秒，超时 1 秒锁存故障
 * 释放请求立即取消剩余目标；固定发数完成后持续请求不重复，退出使能立即撤驱
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
