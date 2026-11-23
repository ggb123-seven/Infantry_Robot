#ifndef SMC_H
#define SMC_H

#include <stdbool.h>

/*
 * 线性滑模角度控制器参数与运行观测，参数快照和运行实例共用扁平类型
 * 控制律 u = J * [target_ddot - C * error_dot - K * s - epsilon * sat(s / surface_eps)]，输出电流，单位 A
 * C：滑模面系数，单位 1/s，必须大于零
 * K：线性趋近系数，单位 1/s，不得为负
 * epsilon_deg_s2：切换项幅值，单位 deg/s^2，不得为负
 * J：等效惯量与电流换算增益，单位 A/(deg/s^2)，必须大于零，上板由固定电流下的角加速度辨识
 * surface_eps_deg_s：滑模面饱和边界宽度，单位 deg/s，必须大于零，加宽可减小静止电流抖动
 * target、target_dot、target_ddot：受限参考角度、角速度和角加速度，单位 deg、deg/s、deg/s^2
 * angle、ang_vel：本周期实际角度和角速度，单位 deg、deg/s
 * error、error_last：本周期及上周期实际角减参考角，单位 deg
 * target_last：上一次完成计算的参考角度，单位 deg；s：滑模面，单位 deg/s
 * u：本周期输出电流，单位 A，正方向与角度增大方向一致
 * 参数字段由调用方在初始化或在线调参边界校验，计算函数信任已校验参数和非空实例
 */
typedef struct
{
    float C;
    float K;
    float epsilon_deg_s2;
    float J;
    float surface_eps_deg_s;
    float target;
    float target_dot;
    float target_ddot;
    float angle;
    float ang_vel;
    float error;
    float error_last;
    float target_last;
    float s;
    float u;
} SMC_t;

/**
 * @brief 在初始化或在线调参边界校验控制参数
 *
 * @param[in] smc 参数快照，调用方保证非空，仅检查参数字段
 * @return 参数有限且满足正滑模系数、非负趋近和切换系数、正换算增益及正边界宽度时为真
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
 * @brief 执行一次线性滑模角度控制并生成候选电流
 *
 * 参考角速度和参考角加速度必须已由 SMC_AdvanceReference 更新
 *
 * @param[in,out] smc 已初始化的控制器实例，调用方保证非空、参数有效且参考已更新
 * @param[in] angle_now 当前角度，单位 deg
 * @param[in] angle_vel 当前角速度，单位 deg/s
 * @return 计算成功为真，电流写入 u；浮点计算异常时返回假
 */
bool SMC_Tick(SMC_t *smc, float angle_now, float angle_vel);

/**
 * @brief 按实际周期推进有速度和加速度约束的参考轨迹
 *
 * @param[in,out] smc 参考状态，调用方保证非空、已经捕获初始参考且参数有效
 * @param[in] target_angle 期望参考角度，单位 deg
 * @param[in] target_speed 同时刻命令前馈，单位 deg/s
 * @param[in] max_speed 正参考速度上限，单位 deg/s
 * @param[in] max_acceleration 正常推进的参考加速度上限，单位 deg/s^2，在线降速上限优先
 * @param[in] reference_gain 正位置追踪增益，单位 1/s
 * @param[in] period_s 已在控制入口校验的实际周期，单位 s
 * @return 无返回值，浮点计算有效性由随后 SMC_Tick 的计算链出口检查
 */
void SMC_AdvanceReference(SMC_t *smc, float target_angle, float target_speed, float max_speed,
                          float max_acceleration, float reference_gain, float period_s);

#endif
