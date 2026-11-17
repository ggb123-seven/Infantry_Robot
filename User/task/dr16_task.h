/*
 * DR16 任务接口
 *
 * 对外暴露 DR16 接收任务入口及供主机测试使用的内部接口
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

/* DR16 任务线程标志，由 UART ISR 设置、任务消费 */
#define DR16_TASK_FLAG_RX_DATA (1UL << 0U)                 /* 新字节到达 */
#define DR16_TASK_FLAG_UART_ERROR (1UL << 1U)              /* 接收错误 */
#define DR16_TASK_FLAG_RING_BUFFER_OVERFLOW (1UL << 2U)    /* 接收缓冲溢出 */

/* Exported functions ------------------------------------------------------- */
/**
 * @brief 运行 DR16 UART 通信调度和状态发布任务
 *
 * @param argument 任务参数，本任务不使用
 * @return 无返回值
 */
void Task_dr16(void *argument);

#if defined(DR16_TASK_TEST)
/**
 * @brief 初始化 DR16 任务内部状态供主机测试使用
 *
 * @return bool UART 回调注册和字节流启动成功时返回 true，否则返回 false
 */
bool DR16_TaskTestInitialize(void);

/**
 * @brief 使用指定线程标志和时间执行一次任务处理供主机测试使用
 *
 * @param flags 待处理的任务线程标志
 * @param now_us 当前时间，单位微秒
 * @return 无返回值
 */
void DR16_TaskTestHandleFlags(uint32_t flags, uint64_t now_us);
#endif

#ifdef __cplusplus
}
#endif
