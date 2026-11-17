/*
 * 云台滑模控制器接口
 *
 * 按终端滑模面计算控制量并做饱和趋近，输出沿用原工程的电流指令尺度
 */

#ifndef SMC_H
#define SMC_H

/* Exported types ----------------------------------------------------------- */
/* 滑模控制器实例，保留原类的字段与排列顺序；target_dot、target_ddot 为逐周期差分 */
typedef struct
{
    float C;         /* 滑模面系数 */
    float K;         /* 趋近律增益 */
    float target;    /* 目标角度，单位：度 */
    float error_eps; /* 角度误差死区，单位：度 */
    float q;         /* 终端滑模幂指数分子 */
    float p;         /* 终端滑模幂指数分母 */
    float u_max;     /* 输出绝对值上限 */
    float J;         /* 等效惯量系数 */
    float angle;     /* 当前角度，单位：度 */
    float ang_vel;   /* 当前角速度，单位：度每秒 */
    float epsilon;   /* 饱和趋近项增益 */
    float u;         /* 控制输出 */

    float error;       /* 当前角度误差，单位：度 */
    float error_last;  /* 历史误差，当前算法未使用 */
    float target_dot;  /* 目标角速度差分 */
    float target_ddot; /* 目标角加速度差分 */
    float target_last; /* 上一周期目标角度 */
    float s;           /* 滑模面数值 */
    float ds;          /* 趋近律数值 */
} SMC_t;

/* Exported functions ------------------------------------------------------- */
/**
 * @brief 设置控制参数并清零控制器历史状态，替代原 C++ 构造函数
 *
 * @param smc 控制器实例，空指针时不执行操作
 * @param C 滑模面系数
 * @param K 趋近律增益
 * @param ref 初始目标角度，单位：度
 * @param error_eps 角度误差死区，单位：度
 * @param q 终端滑模幂指数分子
 * @param p 终端滑模幂指数分母
 * @param u_max 输出绝对值上限
 * @param J 等效惯量系数
 * @param epsilon 饱和趋近项增益
 * @return 无返回值
 */
void SMC_Init(SMC_t *smc, float C, float K, float ref, float error_eps,
              float q, float p, float u_max, float J, float epsilon);

/**
 * @brief 执行一次滑模控制计算，结果保存在 smc->u
 *
 * @param smc 已初始化的控制器实例，空指针时不执行操作
 * @param angle_now 当前角度，单位：度
 * @param angle_vel 当前角速度，单位：度每秒
 * @return 无返回值
 */
void SMC_Tick(SMC_t *smc, float angle_now, float angle_vel);

#endif
