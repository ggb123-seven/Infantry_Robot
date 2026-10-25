#ifndef SMC_H
#define SMC_H

/*
 * 滑模控制器实例，保留原类的全部字段及排列顺序
 * C：滑模面系数；K：趋近律增益，均按原控制律整定
 * target：目标角度，单位为度；error_eps：角度误差死区，单位为度，必须大于零
 * q、p：终端滑模幂指数的分子、分母，p 必须非零
 * u_max：输出绝对值上限，必须非负，单位与 u 相同；J：原控制律的等效惯量系数
 * angle：当前角度，单位为度；ang_vel：当前角速度，单位为度每秒
 * epsilon：饱和趋近项增益；u：控制输出，沿用原工程的电流指令尺度
 * error：当前角度误差，单位为度；error_last：原类保留的历史误差字段，当前算法未使用
 * target_dot、target_ddot：目标角度的一阶、二阶逐周期差分，未按采样周期归一化
 * target_last：上一次完成计算的目标角度，单位为度
 * s：滑模面数值；ds：滑模面趋近律数值，均为算法内部状态
 * 原 private 字段在 C 中可见，由控制器维护，初始化时全部清零
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
 * @brief 设置控制参数并清零控制器历史状态，替代原 C++ 构造函数
 *
 * @param[out] smc 控制器实例，空指针时不执行操作
 * @param[in] C 滑模面系数，沿用原控制律整定值
 * @param[in] K 趋近律增益，沿用原控制律整定值
 * @param[in] ref 初始目标角度，单位为度
 * @param[in] error_eps 角度误差死区，单位为度，必须大于零
 * @param[in] q 终端滑模幂指数的分子
 * @param[in] p 终端滑模幂指数的分母，必须非零
 * @param[in] u_max 输出绝对值上限，必须非负，单位与输出相同
 * @param[in] J 等效惯量系数，沿用原控制律整定值
 * @param[in] epsilon 饱和趋近项增益，沿用原控制律整定值
 * @return 无返回值
 */
void SMC_Init(SMC_t *smc, float C, float K, float ref, float error_eps,
              float q, float p, float u_max, float J, float epsilon);

/**
 * @brief 执行一次滑模控制计算，将输出保存到 smc->u
 *
 * @param[in,out] smc 已初始化的控制器实例，空指针时不执行操作
 * @param[in] angle_now 当前角度，单位为度
 * @param[in] angle_vel 当前角速度，单位为度每秒
 * @return 无返回值
 */
void SMC_Tick(SMC_t *smc, float angle_now, float angle_vel);

#endif
