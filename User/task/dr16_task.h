#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

/*
 * DR16 任务线程标志：
 * - DR16_TASK_FLAG_RX_DATA：UART BSP 已向设备接收器写入新字节
 * - DR16_TASK_FLAG_UART_ERROR：UART BSP 检测到接收错误，任务需要清空并重启链路
 * - DR16_TASK_FLAG_RING_BUFFER_OVERFLOW：设备接收器空间不足，任务需要丢弃未验证数据并重启链路
 */
#define DR16_TASK_FLAG_RX_DATA (1UL << 0U)
#define DR16_TASK_FLAG_UART_ERROR (1UL << 1U)
#define DR16_TASK_FLAG_RING_BUFFER_OVERFLOW (1UL << 2U)

/**
 * @brief 运行 DR16 UART 通信调度和状态发布任务
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_dr16(void *argument);

#if defined(DR16_TASK_TEST)
/**
 * @brief 初始化 DR16 任务内部状态供主机测试使用
 *
 * @return UART 回调注册和字节流启动成功时返回 true，否则返回 false
 */
bool DR16_TaskTestInitialize(void);

/**
 * @brief 使用指定线程标志和时间执行一次任务处理供主机测试使用
 *
 * @param[in] flags 待处理的任务线程标志
 * @param[in] now_us 当前时间，单位微秒
 * @return 无返回值
 */
void DR16_TaskTestHandleFlags(uint32_t flags, uint64_t now_us);
#endif

#ifdef __cplusplus
}
#endif
