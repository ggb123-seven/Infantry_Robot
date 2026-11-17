/*
 * 底盘运动学混合器接口
 *
 * 把归一化的 vx、vy、wz 运动向量按底盘机械布局分解为各轮目标转速
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ----------------------------------------------------------------- */
#include "component/user_math.h"

#include <stdint.h>

/* Exported constants ------------------------------------------------------- */
#define MIXER_MAX_OUTPUT_COUNT (4U) /* 输出数组最大长度 */

/* Exported types ----------------------------------------------------------- */
/* 混合器运行状态，负值为错误 */
typedef enum
{
    MIXER_OK = 0,             /* 成功 */
    MIXER_ERROR = -1,         /* 通用错误 */
    MIXER_NULL_ERROR = -2,    /* 空指针 */
    MIXER_LENGTH_ERROR = -3,  /* 输出数组长度不匹配 */
    MIXER_MODE_ERROR = -4,    /* 布局非法 */
    MIXER_INVALID_VALUE = -5, /* 数值非法 */
    MIXER_UNSUPPORTED = -6,   /* 布局未实现 */
} Mixer_Status_t;

/* 底盘机械布局，各布局的轮序按对应安装约定排列 */
typedef enum
{
    MIXER_MECANUM = 0, /* 麦轮四轮 */
    MIXER_PARLFIX4,    /* 四轮平行 */
    MIXER_PARLFIX2,    /* 两轮平行 */
    MIXER_OMNICROSS,   /* 全向十字 */
    MIXER_OMNIPLUS,    /* 全向加号，未实现 */
    MIXER_SINGLE,      /* 单轮 */
    MIXER_MODE_COUNT,  /* 模式数量 */
} Mixer_Mode_t;

/* 混合器运行上下文 */
typedef struct
{
    Mixer_Mode_t mode; /* 当前底盘机械布局 */
} Mixer_t;

/* Exported functions ------------------------------------------------------- */
/**
 * @brief 初始化底盘运动学混合器
 *
 * @param mixer 待初始化的混合器上下文
 * @param mode 底盘机械布局
 * @return int8_t 成功返回 MIXER_OK，失败返回对应状态码
 */
int8_t Mixer_Init(Mixer_t *mixer, Mixer_Mode_t mode);

/**
 * @brief 将归一化底盘运动向量转换为各轮输出轴目标转速
 *
 * 输入采用前向为正的 vx、左向为正的 vy 和俯视逆时针为正的 wz，
 * 输出超过单位范围时按最大绝对值整体缩放，再乘以目标转速尺度
 *
 * @param mixer 已初始化的混合器上下文
 * @param move_vector 归一化底盘运动向量
 * @param output_rpm 各轮输出轴目标转速数组，单位：rpm
 * @param output_count 输出数组元素数量，须与当前布局的轮数一致
 * @param scale_rpm 归一化输出对应的转速尺度，单位：rpm
 * @return int8_t 成功返回 MIXER_OK，失败返回对应状态码并清零可访问输出
 */
int8_t Mixer_Apply(const Mixer_t *mixer, const MoveVector_t *move_vector, float *output_rpm,
                   uint32_t output_count, float scale_rpm);

#ifdef __cplusplus
}
#endif
