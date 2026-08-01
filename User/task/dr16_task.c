#include "task/dr16_task.h"

#include "bsp/time.h"
#include "bsp/uart.h"
#include "module/fault_detect.h"
#include "task/ozone_debug.h"
#include "task/user_task.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

/*
 * DR16 任务运行参数：
 * - DR16_TASK_WAIT_INTERVAL_MS：在线时线程标志最长等待时间，也是离线和重启检查周期，单位毫秒
 */
#define DR16_TASK_WAIT_INTERVAL_MS (10U)
#define DR16_TASK_EVENT_FLAGS                                                                                       \
    (DR16_TASK_FLAG_RX_DATA | DR16_TASK_FLAG_UART_ERROR | DR16_TASK_FLAG_RING_BUFFER_OVERFLOW)

static DR16_State_t dr16_state;
static FaultDetect_DR16Snapshot_t dr16_fault_snapshot;
static bool dr16_receiver_ready;
static bool dr16_callbacks_registered;
static bool dr16_stream_running;

static void DR16_TaskRxDataCallback(const uint8_t *data, uint16_t length);
static void DR16_TaskUartErrorCallback(uint32_t error_code);
static bool DR16_TaskInitialize(void);
static bool DR16_TaskPublishState(void);
static bool DR16_TaskRecoverStream(void);
static void DR16_TaskHandleFlags(uint32_t flags, uint64_t now_us);
static void DR16_TaskUpdateMonitor(uint64_t now_us);
static uint32_t DR16_TaskGetWaitTicks(void);
static void DR16_TaskNotify(uint32_t flag);

/**
 * @brief 运行 DR16 UART 通信调度和状态发布任务
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_dr16(void *argument)
{
    (void)argument;

    // 初始化设备接收器、任务诊断、UART 回调和循环 DMA 接收
    DR16_TaskInitialize();
    const uint32_t wait_ticks = DR16_TaskGetWaitTicks();

    while (true)
    {
        uint32_t flags = 0U;
        if (dr16_stream_running)
        {
            // 等待接收或错误事件，同时保留固定周期执行离线检查
            const uint32_t wait_result = osThreadFlagsWait(DR16_TASK_EVENT_FLAGS, osFlagsWaitAny, wait_ticks);
            if ((wait_result & osFlagsError) == 0U)
            {
                flags = wait_result;
            }
            else if (wait_result != osFlagsErrorTimeout)
            {
                g_dr16_monitor.thread_wait_error_count++;
            }
        }
        else
        {
            // 接收链路未运行时限制重试频率，避免持续占用处理器
            if (osDelay(wait_ticks) != osOK)
            {
                g_dr16_monitor.thread_wait_error_count++;
            }
        }

        // 获取统一时间并优先恢复停机链路，再处理本轮字节和离线状态
        const uint64_t now_us = BSP_TIME_Get();
        if (!dr16_stream_running && (flags & (DR16_TASK_FLAG_UART_ERROR | DR16_TASK_FLAG_RING_BUFFER_OVERFLOW)) == 0U)
        {
            DR16_TaskRecoverStream();
        }
        DR16_TaskHandleFlags(flags, now_us);
    }
}

/**
 * @brief 将 UART BSP 交付的新字节转交给 DR16 接收器
 *
 * 本函数在 UART 中断回调上下文执行，只复制字节、更新诊断并设置线程标志
 *
 * @param[in] data 本次新增字节段首地址
 * @param[in] length 本次新增字节数
 * @return 无返回值
 */
static void DR16_TaskRxDataCallback(const uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0U)
    {
        return;
    }

    g_dr16_monitor.received_byte_count += length;
    g_dr16_monitor.uart_event_count++;

    // 接收器拒绝本次字节时通知任务停止 UART 并清理全部未验证数据
    if (DR16_ReceiverFeed(data, length) != DR16_RECEIVER_OK)
    {
        DR16_TaskNotify(DR16_TASK_FLAG_RING_BUFFER_OVERFLOW);
        return;
    }

    DR16_TaskNotify(DR16_TASK_FLAG_RX_DATA);
}

/**
 * @brief 将 UART BSP 错误转换为任务线程标志
 *
 * 本函数在 UART 中断回调上下文执行，不重启外设或处理协议
 *
 * @param[in] error_code HAL UART 错误位或 BSP 内部错误码
 * @return 无返回值
 */
static void DR16_TaskUartErrorCallback(uint32_t error_code)
{
    g_dr16_monitor.uart_error_count++;
    g_dr16_monitor.last_uart_error = error_code;
    DR16_TaskNotify(DR16_TASK_FLAG_UART_ERROR);
}

/**
 * @brief 初始化任务私有状态并启动 UART 循环 DMA 字节流
 *
 * @return UART 回调注册和字节流启动成功时返回 true，否则返回 false
 */
static bool DR16_TaskInitialize(void)
{
    memset((void *)&g_dr16_monitor, 0, sizeof(g_dr16_monitor));
    DR16_ResetState(&dr16_state);
    dr16_fault_snapshot = (FaultDetect_DR16Snapshot_t)
    {
        0,
    };
    dr16_receiver_ready = false;
    dr16_callbacks_registered = false;
    dr16_stream_running = false;

    // 先建立设备层协议接收器，再发布可立即消费的初始离线状态
    dr16_receiver_ready = DR16_ReceiverInit() == DR16_RECEIVER_OK;
    DR16_TaskPublishState();
    if (!dr16_receiver_ready)
    {
        g_dr16_monitor.stream_restart_error_count++;
        DR16_TaskUpdateMonitor(0U);
        return false;
    }

    // 在启动 DMA 前完成两个 ISR 回调注册，避免接收事件没有明确所有者
    const int8_t register_status = BSP_UART_StreamRegisterCallbacks(BSP_UART_DR16, DR16_TaskRxDataCallback,
                                                                    DR16_TaskUartErrorCallback);
    dr16_callbacks_registered = register_status == BSP_OK;
    if (!dr16_callbacks_registered)
    {
        g_dr16_monitor.last_uart_error = BSP_UART_StreamGetError(BSP_UART_DR16);
        DR16_TaskUpdateMonitor(0U);
        return false;
    }

    // 回调和缓冲区就绪后才开放循环 DMA 字节流交付
    dr16_stream_running = BSP_UART_StreamStart(BSP_UART_DR16) == BSP_OK;
    if (!dr16_stream_running)
    {
        g_dr16_monitor.last_uart_error = BSP_UART_StreamGetError(BSP_UART_DR16);
    }
    DR16_TaskUpdateMonitor(0U);
    return dr16_stream_running;
}

/**
 * @brief 使用长度为 1 的邮箱发布最新 DR16 一致快照
 *
 * @return 重置邮箱并写入状态均成功时返回 true，否则返回 false
 */
static bool DR16_TaskPublishState(void)
{
    // 发布前从设备层取得唯一状态源，读取失败时保持离线零值
    if (DR16_ReceiverGetState(&dr16_state) != DR16_RECEIVER_OK)
    {
        DR16_ResetState(&dr16_state);
    }
    if (task_runtime.msgq.dr16_state == NULL)
    {
        g_dr16_monitor.mailbox_error_count++;
        return false;
    }

    // 邮箱只保留最新状态，重置和写入均必须检查结果
    if (osMessageQueueReset(task_runtime.msgq.dr16_state) != osOK ||
        osMessageQueuePut(task_runtime.msgq.dr16_state, &dr16_state, 0U, 0U) != osOK)
    {
        g_dr16_monitor.mailbox_error_count++;
        return false;
    }
    return true;
}

/**
 * @brief 暂停 UART 交付、清空未验证字节并重新启动循环 DMA
 *
 * @return UART 回调已注册且字节流重启成功时返回 true，否则返回 false
 */
static bool DR16_TaskRecoverStream(void)
{
    dr16_stream_running = false;
    if (!dr16_receiver_ready)
    {
        // 接收器初始化失败时先重新建立协议状态，再恢复 UART 回调和字节流
        dr16_receiver_ready = DR16_ReceiverInit() == DR16_RECEIVER_OK;
        if (!dr16_receiver_ready)
        {
            g_dr16_monitor.stream_restart_error_count++;
            return false;
        }
    }

    // 初次注册失败时先恢复回调所有权，再操作接收链路
    if (!dr16_callbacks_registered)
    {
        const int8_t register_status = BSP_UART_StreamRegisterCallbacks(BSP_UART_DR16, DR16_TaskRxDataCallback,
                                                                        DR16_TaskUartErrorCallback);
        dr16_callbacks_registered = register_status == BSP_OK;
        if (!dr16_callbacks_registered)
        {
            g_dr16_monitor.stream_restart_error_count++;
            return false;
        }
    }

    // 只有确认 UART 不再向回调交付字节后，才复位设备层生产者与消费者共享的协议字节流
    if (BSP_UART_StreamStop(BSP_UART_DR16) != BSP_OK)
    {
        g_dr16_monitor.last_uart_error = BSP_UART_StreamGetError(BSP_UART_DR16);
        g_dr16_monitor.stream_restart_error_count++;
        return false;
    }
    if (DR16_ReceiverResetStream() != DR16_RECEIVER_OK)
    {
        g_dr16_monitor.stream_restart_error_count++;
        return false;
    }

    // 清空旧数据后重新开放接收，恢复后仍需新的完整合法帧才能上线
    dr16_stream_running = BSP_UART_StreamStart(BSP_UART_DR16) == BSP_OK;
    if (!dr16_stream_running)
    {
        g_dr16_monitor.last_uart_error = BSP_UART_StreamGetError(BSP_UART_DR16);
        g_dr16_monitor.stream_restart_error_count++;
    }
    return dr16_stream_running;
}

/**
 * @brief 按优先级处理错误恢复、设备更新和状态发布
 *
 * @param[in] flags 本轮任务线程标志
 * @param[in] now_us 当前时间，单位微秒
 * @return 无返回值
 */
static void DR16_TaskHandleFlags(uint32_t flags, uint64_t now_us)
{
    const uint32_t error_flags = DR16_TASK_FLAG_UART_ERROR | DR16_TASK_FLAG_RING_BUFFER_OVERFLOW;
    if ((flags & error_flags) != 0U)
    {
        // 错误优先于协议处理，立即发布离线安全状态并丢弃全部未验证字节
        DR16_ReceiverSetOffline();
        DR16_TaskPublishState();
        DR16_TaskRecoverStream();
        DR16_TaskUpdateMonitor(now_us);
        return;
    }

    // 每轮推进协议解析和离线判断，避免线程标志异常时未处理已经复制的字节
    if (DR16_ReceiverUpdate(now_us))
    {
        DR16_TaskPublishState();
    }

    // 汇总接收器、UART 和故障检测状态供 Ozone 观察
    DR16_TaskUpdateMonitor(now_us);
}

/**
 * @brief 刷新不参与控制的 Ozone 诊断状态
 *
 * @param[in] now_us 当前时间，单位微秒
 * @return 无返回值
 */
static void DR16_TaskUpdateMonitor(uint64_t now_us)
{
    DR16_ReceiverDiagnostics_t diagnostics;
    if (DR16_ReceiverGetState(&dr16_state) != DR16_RECEIVER_OK)
    {
        DR16_ResetState(&dr16_state);
    }
    if (DR16_ReceiverGetDiagnostics(&diagnostics) != DR16_RECEIVER_OK)
    {
        diagnostics = (DR16_ReceiverDiagnostics_t)
        {
            0,
        };
    }

    // 接收器未初始化时显式报告状态缺失，避免把清零快照误认为有效设备状态
    const DR16_State_t *fault_state = diagnostics.initialized ? &dr16_state : NULL;
    FaultDetect_UpdateDR16(fault_state, dr16_stream_running, &dr16_fault_snapshot);

    // 计算最后合法帧年龄并在超出 Ozone 32 位显示范围时饱和
    uint64_t frame_age_us = 0U;
    if (dr16_state.header.last_online_time != 0U && now_us >= dr16_state.header.last_online_time)
    {
        frame_age_us = now_us - dr16_state.header.last_online_time;
    }
    if (frame_age_us > UINT32_MAX)
    {
        frame_age_us = UINT32_MAX;
    }

    g_dr16_monitor.valid_frame_count = diagnostics.valid_frame_count;
    g_dr16_monitor.invalid_frame_count = diagnostics.invalid_frame_count;
    g_dr16_monitor.resync_discarded_byte_count = diagnostics.resync_discarded_byte_count;
    g_dr16_monitor.ring_buffer_overflow_count = diagnostics.ring_buffer_overflow_count;
    g_dr16_monitor.current_frame_age_us = (uint32_t)frame_age_us;
    g_dr16_monitor.receiver_initialized = diagnostics.initialized;
    g_dr16_monitor.online = dr16_state.header.online;
    g_dr16_monitor.fault_detected = dr16_fault_snapshot.has_fault;
    g_dr16_monitor.fault_flags = dr16_fault_snapshot.fault_flags;
    g_dr16_monitor.latest_data = dr16_state.data;
}

/**
 * @brief 将任务等待周期从毫秒换算为向上取整的内核节拍数
 *
 * @return 至少为 1 的等待节拍数
 */
static uint32_t DR16_TaskGetWaitTicks(void)
{
    const uint32_t tick_frequency = osKernelGetTickFreq();
    if (tick_frequency == 0U)
    {
        return 1U;
    }

    const uint64_t scaled_ticks = (uint64_t)tick_frequency * DR16_TASK_WAIT_INTERVAL_MS + 999ULL;
    const uint64_t wait_ticks = scaled_ticks / 1000ULL;
    return wait_ticks > UINT32_MAX ? UINT32_MAX : (uint32_t)wait_ticks;
}

/**
 * @brief 从 UART ISR 回调向 DR16 任务设置线程标志
 *
 * @param[in] flag 单个 DR16_TASK_FLAG_* 线程标志
 * @return 无返回值
 */
static void DR16_TaskNotify(uint32_t flag)
{
    const uint32_t notify_result = osThreadFlagsSet(task_runtime.thread.dr16, flag);
    if ((notify_result & osFlagsError) != 0U)
    {
        g_dr16_monitor.thread_notify_error_count++;
    }
}

#if defined(DR16_TASK_TEST)
/**
 * @brief 初始化 DR16 任务内部状态供主机测试使用
 *
 * @return UART 回调注册和字节流启动成功时返回 true，否则返回 false
 */
bool DR16_TaskTestInitialize(void)
{
    return DR16_TaskInitialize();
}

/**
 * @brief 使用指定线程标志和时间执行一次任务处理供主机测试使用
 *
 * @param[in] flags 待处理的任务线程标志
 * @param[in] now_us 当前时间，单位微秒
 * @return 无返回值
 */
void DR16_TaskTestHandleFlags(uint32_t flags, uint64_t now_us)
{
    DR16_TaskHandleFlags(flags, now_us);
}
#endif
