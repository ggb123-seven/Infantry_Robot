#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

#include "device/dr16.h"

/*
 * DR16 任务线程标志：
 * - DR16_TASK_FLAG_RX_DATA：UART BSP 已向任务私有 RingBuffer 写入新字节。
 * - DR16_TASK_FLAG_UART_ERROR：UART BSP 检测到接收错误，任务需要清空并重启链路。
 * - DR16_TASK_FLAG_RING_BUFFER_OVERFLOW：RingBuffer 空间不足，任务需要丢弃未验证数据并重启链路。
 */
#define DR16_TASK_FLAG_RX_DATA (1UL << 0U)
#define DR16_TASK_FLAG_UART_ERROR (1UL << 1U)
#define DR16_TASK_FLAG_RING_BUFFER_OVERFLOW (1UL << 2U)

/*
 * DR16 任务诊断数据：
 * - received_byte_count：UART BSP 交付的累计字节数。
 * - uart_event_count：UART BSP 交付的累计字节段数量。
 * - valid_frame_count、invalid_frame_count：合法帧数量和重同步期间尝试失败的候选帧数量。
 * - resync_discarded_byte_count：逐字节重同步累计丢弃的字节数。
 * - ring_buffer_overflow_count、uart_error_count：RingBuffer 溢出和 UART 错误次数。
 * - thread_notify_error_count、thread_wait_error_count：线程标志设置和等待异常次数。
 * - mailbox_error_count、stream_restart_error_count：状态邮箱发布和 UART 重启失败次数。
 * - last_uart_error：最近一次 UART HAL 错误位或 BSP 内部错误码。
 * - current_frame_age_us：当前时刻距离最后合法帧的时间，单位微秒，超过 32 位时饱和。
 * - online：最近一帧业务状态的在线标志。
 * - latest_data：最近一帧完整合法的解码值，仅供调试观察，不作为控制输入。
 */
typedef struct
{
    uint32_t received_byte_count;
    uint32_t uart_event_count;
    uint32_t valid_frame_count;
    uint32_t invalid_frame_count;
    uint32_t resync_discarded_byte_count;
    uint32_t ring_buffer_overflow_count;
    uint32_t uart_error_count;
    uint32_t thread_notify_error_count;
    uint32_t thread_wait_error_count;
    uint32_t mailbox_error_count;
    uint32_t stream_restart_error_count;
    uint32_t last_uart_error;
    uint32_t current_frame_age_us;
    bool online;
    DR16_Data_t latest_data;
} DR16_Monitor_t;

extern volatile DR16_Monitor_t g_dr16_monitor;

/**
 * @brief 运行 DR16 接收、重同步和状态发布任务
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
