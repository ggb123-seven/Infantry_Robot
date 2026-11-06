#ifndef SMC_H
#define SMC_H

#include <stdbool.h>

/*
 * 滑模角度控制器参数与运行观测，参数快照和运行实例共用扁平类型
 * C：角误差滑模面系数，单位 1/s；K：趋近项速度增益，单位 deg/s
 * target：受限参考角度，单位 deg；error_eps：角误差连续过渡尺度，单位 deg，必须大于零
 * q、p：终端滑模幂指数的分子、分母，满足 0 < q/p <= 1
 * speed_max_deg_s：SMC 速度目标绝对值上限，单位 deg/s，必须大于零
 * surface_eps_deg_s：滑模面连续饱和边界宽度，单位 deg/s，必须大于零
 * angle、ang_vel：本周期实际角度和角速度，单位 deg、deg/s
 * error、error_last：本周期及上周期的实际角减参考角，单位 deg
 * target_dot、target_ddot：受限参考角速度和角加速度，单位 deg/s、deg/s²
 * target_last：上一次完成计算的参考角度，单位 deg；s：滑模面，单位 deg/s
 * speed_target_deg_s：SMC 输出的速度目标，单位 deg/s
 * 参数字段由调用方在初始化或在线调参边界校验，计算函数信任已校验参数和非空实例
 */
typedef struct
{
    float C;
    float K;
    float target;
    float error_eps;
    float q;
    float p;
    float speed_max_deg_s;
    float surface_eps_deg_s;
    float angle;
    float ang_vel;
    float error;
    float error_last;
    float target_dot;
    float target_ddot;
    float target_last;
    float s;
    float speed_target_deg_s;
} SMC_t;

/**
 * @brief 在初始化或在线调参边界校验控制参数
 *
 * @param[in] smc 参数快照，调用方保证非空，仅检查参数字段
 * @return 参数有限且满足正滑模系数、正边界宽度、合法幂指数及正速度上限时为真
 */
bool SMC_IsConfigValid(const SMC_t *smc);

/**
 * @brief 复制已校验参数并清零控制器历史状态
 *
 * @param[out] smc 控制器实例，调用方保证非空且所有参数已经校验
 * @param[in] config 已在调用边界校验的滑模参数快照
 * @param[in] ref 初始参考角度，单位 deg
 * @return 无返回值
 */
void SMC_Init(SMC_t *smc, const SMC_t *config, float ref);

/**
 * @brief 执行一次滑模角度控制并生成限幅后的速度目标
 *
 * @param[in,out] smc 已初始化的控制器实例，调用方保证非空、参数有效且参考角度已更新
 * @param[in] angle_now 当前角度，单位为度
 * @param[in] angle_vel 当前角速度，单位为度每秒
 * @return 计算成功为真，速度目标写入 speed_target_deg_s；浮点计算异常时返回假
 */
bool SMC_Tick(SMC_t *smc, float angle_now, float angle_vel);

/**
 * @brief 按实际周期推进有速度和加速度约束的参考轨迹
 *
 * @param[in,out] smc 参考状态，调用方保证非空、已经捕获初始参考且参数有效
 * @param[in] target_angle 期望参考角度，单位度
 * @param[in] target_speed 同时刻命令前馈，单位度每秒
 * @param[in] max_speed 正参考速度上限，单位度每秒
 * @param[in] max_acceleration 正常推进的参考加速度上限，单位度每平方秒，在线降速上限优先
 * @param[in] reference_gain 正位置追踪增益，单位每秒
 * @param[in] period_s 已在控制入口校验的实际周期，单位秒
 * @return 无返回值，浮点计算有效性由随后 SMC_Tick 的计算链出口检查
 */
void SMC_AdvanceReference(SMC_t *smc, float target_angle, float target_speed, float max_speed,
                          float max_acceleration, float reference_gain, float period_s);

#endif
