#ifndef SMC_H
#define SMC_H

#include <stdbool.h>

/*
 * 滑模控制器实例，保留原类的全部字段及排列顺序
 * C：滑模面系数，单位每秒；K：趋近律增益，单位每秒
 * target：参考角度，单位度；error_eps：近零连续过渡尺度，单位度，必须大于零，不再作为撤驱死区
 * q、p：终端滑模幂指数的分子、分母，满足零小于 q/p 且 q/p 不超过一
 * u_max：输出电流绝对值上限，单位安培，必须大于零；J：等效输出增益，单位安培乘平方秒每度
 * angle：当前角度，单位为度；ang_vel：当前角速度，单位为度每秒
 * epsilon：饱和趋近项增益，单位度每平方秒；u：保护前电流，单位安培
 * error、error_last：本周期及上周期的实际角减参考角，单位度
 * target_dot、target_ddot：本周期参考角速度和角加速度，单位度每秒、度每平方秒
 * target_last：上一次完成计算的目标角度，单位为度
 * s：滑模变量，单位度每秒；ds：趋近律，单位度每平方秒
 * 参数及运行状态仍沿用原字段布局，计算入口由调用方保证参数有效、指针非空
 */
typedef struct
{
    float C;
    float K;
    float target;
    float error_eps;
    float q;
    float p;
    float u_max;
    float J;
    float angle;
    float ang_vel;
    float epsilon;
    float u;

    float error;
    float error_last;
    float target_dot;
    float target_ddot;
    float target_last;
    float s;
    float ds;
} SMC_t;

/**
 * @brief 在初始化或在线调参边界校验控制参数
 *
 * @param[in] smc 参数快照，调用方保证非空，仅检查参数字段
 * @return 参数有限且满足正增益、正过渡尺度、合法幂指数及正电流上限时为真
 */
bool SMC_IsConfigValid(const SMC_t *smc);

/**
 * @brief 设置控制参数并清零控制器历史状态，替代原 C++ 构造函数
 *
 * @param[out] smc 控制器实例，调用方保证非空且所有参数已经校验
 * @param[in] C 滑模面系数，单位每秒
 * @param[in] K 趋近律增益，单位每秒
 * @param[in] ref 初始目标角度，单位为度
 * @param[in] error_eps 近零连续过渡尺度，单位度，必须大于零
 * @param[in] q 终端滑模幂指数的分子
 * @param[in] p 终端滑模幂指数的分母，必须不小于正分子
 * @param[in] u_max 正电流限幅，单位安培
 * @param[in] J 正等效输出增益，单位安培乘平方秒每度
 * @param[in] epsilon 非负趋近项增益，单位度每平方秒
 * @return 无返回值
 */
void SMC_Init(SMC_t *smc, float C, float K, float ref, float error_eps,
              float q, float p, float u_max, float J, float epsilon);

/**
 * @brief 执行一次滑模控制计算，将输出保存到 smc->u
 *
 * @param[in,out] smc 已初始化的控制器实例，调用方保证非空、参数有效且参考角度已更新
 * @param[in] angle_now 当前角度，单位为度
 * @param[in] angle_vel 当前角速度，单位为度每秒
 * @param[in] target_speed 同时刻参考角速度，单位度每秒，由调用方按实际周期生成
 * @param[in] target_acceleration 同时刻参考角加速度，单位度每平方秒
 * @return 计算成功为真；浮点运算异常为假，由调用方进入故障并阻断输出
 */
bool SMC_Tick(SMC_t *smc, float angle_now, float angle_vel, float target_speed, float target_acceleration);

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
