#include "task/dr16_task.h"

#include "bsp/time.h"
#include "bsp/uart.h"
#include "ringbuffer/ringbuffer.h"
#include "task/user_task.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

/*
 * DR16 任务运行参数：
 * - DR16_TASK_RING_BUFFER_SIZE：任务私有接收缓冲区容量，实际可用容量比定义值少 1 字节。
 * - DR16_TASK_OFFLINE_TIMEOUT_US：连续未收到合法帧后判定离线的时间，单位微秒。
 * - DR16_TASK_WAIT_INTERVAL_MS：在线时线程标志最长等待时间，也是离线和重启检查周期，单位毫秒。
 */
#define DR16_TASK_RING_BUFFER_SIZE (128U)
#define DR16_TASK_OFFLINE_TIMEOUT_US (100000ULL)
#define DR16_TASK_WAIT_INTERVAL_MS (10U)
#define DR16_TASK_EVENT_FLAGS                                                                                       \
    (DR16_TASK_FLAG_RX_DATA | DR16_TASK_FLAG_UART_ERROR | DR16_TASK_FLAG_RING_BUFFER_OVERFLOW)

volatile DR16_Monitor_t g_dr16_monitor;

static lwrb_t dr16_ring_buffer;
static uint8_t dr16_ring_buffer_storage[DR16_TASK_RING_BUFFER_SIZE];
static DR16_State_t dr16_state;
static bool dr16_ring_buffer_ready;
static bool dr16_callbacks_registered;
static bool dr16_stream_running;

static void DR16_TaskRxDataCallback(const uint8_t *data, uint16_t length);
static void DR16_TaskUartErrorCallback(uint32_t error_code);
static bool DR16_TaskInitialize(void);
static bool DR16_TaskPublishState(void);
static void DR16_TaskProcessFrames(uint64_t now_us);
static void DR16_TaskSetOffline(bool force_publish);
static void DR16_TaskCheckOffline(uint64_t now_us);
static bool DR16_TaskRecoverStream(void);
static void DR16_TaskHandleFlags(uint32_t flags, uint64_t now_us);
static void DR16_TaskUpdateMonitor(uint64_t now_us);
static uint32_t DR16_TaskGetWaitTicks(void);
static void DR16_TaskNotify(uint32_t flag);

/**
 * @brief 运行 DR16 接收、重同步和状态发布任务
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_dr16(void *argument)
{
    (void)argument;

    // 初始化任务私有缓冲区、离线状态、UART 回调和循环 DMA 接收
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
 * @brief 将 UART BSP 交付的新字节复制到任务私有 RingBuffer
 *
 * 本函数在 UART 中断回调上下文执行，只复制字节、更新诊断并设置线程标志。
 *
 * @param[in] data 本次新增字节段首地址
 * @param[in] length 本次新增字节数
 * @return 无返回值
 */
static void DR16_TaskRxDataCallback(const uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0U || !dr16_ring_buffer_ready)
    {
        return;
    }

    g_dr16_monitor.received_byte_count += length;
    g_dr16_monitor.uart_event_count++;

    // 原子单生产者写入不足时通知任务丢弃全部未验证数据并恢复接收链路
    const lwrb_sz_t written = lwrb_write(&dr16_ring_buffer, data, length);
    if (written != length)
    {
        g_dr16_monitor.ring_buffer_overflow_count++;
        DR16_TaskNotify(DR16_TASK_FLAG_RING_BUFFER_OVERFLOW);
        return;
    }

    DR16_TaskNotify(DR16_TASK_FLAG_RX_DATA);
}

/**
 * @brief 将 UART BSP 错误转换为任务线程标志
 *
 * 本函数在 UART 中断回调上下文执行，不重启外设或处理协议。
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
    dr16_callbacks_registered = false;
    dr16_stream_running = false;

    // 先建立任务私有 RingBuffer，再发布可立即消费的初始离线状态
    dr16_ring_buffer_ready = lwrb_init(&dr16_ring_buffer, dr16_ring_buffer_storage,
                                       sizeof(dr16_ring_buffer_storage)) != 0U;
    DR16_TaskPublishState();
    if (!dr16_ring_buffer_ready)
    {
        g_dr16_monitor.stream_restart_error_count++;
        return false;
    }

    // 在启动 DMA 前完成两个 ISR 回调注册，避免接收事件没有明确所有者
    const int8_t register_status = BSP_UART_StreamRegisterCallbacks(BSP_UART_DR16, DR16_TaskRxDataCallback,
                                                                    DR16_TaskUartErrorCallback);
    dr16_callbacks_registered = register_status == BSP_OK;
    if (!dr16_callbacks_registered)
    {
        g_dr16_monitor.last_uart_error = BSP_UART_StreamGetError(BSP_UART_DR16);
        return false;
    }

    // 回调和缓冲区就绪后才开放循环 DMA 字节流交付
    dr16_stream_running = BSP_UART_StreamStart(BSP_UART_DR16) == BSP_OK;
    if (!dr16_stream_running)
    {
        g_dr16_monitor.last_uart_error = BSP_UART_StreamGetError(BSP_UART_DR16);
    }
    return dr16_stream_running;
}

/**
 * @brief 使用长度为 1 的邮箱发布最新 DR16 一致快照
 *
 * @return 重置邮箱并写入状态均成功时返回 true，否则返回 false
 */
static bool DR16_TaskPublishState(void)
{
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
 * @brief 从任务私有 RingBuffer 解码全部可用候选帧
 *
 * @param[in] now_us 当前时间，单位微秒
 * @return 无返回值
 */
static void DR16_TaskProcessFrames(uint64_t now_us)
{
    uint8_t frame[DR16_FRAME_LENGTH];
    while (lwrb_get_full(&dr16_ring_buffer) >= DR16_FRAME_LENGTH)
    {
        // 查看当前候选帧但暂不移除，保证非法帧可以仅丢弃一个字节后继续查找边界
        if (lwrb_peek(&dr16_ring_buffer, 0U, frame, sizeof(frame)) != sizeof(frame))
        {
            break;
        }

        DR16_Data_t decoded;
        if (DR16_Decode(frame, sizeof(frame), &decoded) == DEVICE_OK)
        {
            // 完整合法帧一次性提交，并按整帧长度推进读取位置
            lwrb_skip(&dr16_ring_buffer, DR16_FRAME_LENGTH);
            dr16_state.header.online = true;
            dr16_state.header.last_online_time = now_us;
            dr16_state.valid_frame_sequence++;
            dr16_state.data = decoded;
            g_dr16_monitor.valid_frame_count++;
            g_dr16_monitor.latest_data = decoded;
            DR16_TaskPublishState();
        }
        else
        {
            // 候选帧非法时只丢弃一个字节，以便从任意错位位置重新获得帧边界
            lwrb_skip(&dr16_ring_buffer, 1U);
            g_dr16_monitor.invalid_frame_count++;
            g_dr16_monitor.resync_discarded_byte_count++;
        }
    }
}

/**
 * @brief 将业务状态切换为离线并清零控制数据
 *
 * 最后合法帧时间和序号保持不变，避免离线事件伪装成新的遥控帧。
 *
 * @param[in] force_publish 为 true 时即使已离线也发布错误后的安全快照
 * @return 无返回值
 */
static void DR16_TaskSetOffline(bool force_publish)
{
    if (!dr16_state.header.online && !force_publish)
    {
        return;
    }

    dr16_state.header.online = false;
    memset(&dr16_state.data, 0, sizeof(dr16_state.data));
    DR16_TaskPublishState();
}

/**
 * @brief 检查连续无合法帧时间并只发布一次超时离线状态
 *
 * @param[in] now_us 当前时间，单位微秒
 * @return 无返回值
 */
static void DR16_TaskCheckOffline(uint64_t now_us)
{
    if (dr16_state.header.online && now_us - dr16_state.header.last_online_time >= DR16_TASK_OFFLINE_TIMEOUT_US)
    {
        DR16_TaskSetOffline(false);
    }
}

/**
 * @brief 暂停 UART 交付、清空未验证字节并重新启动循环 DMA
 *
 * @return UART 回调已注册且字节流重启成功时返回 true，否则返回 false
 */
static bool DR16_TaskRecoverStream(void)
{
    dr16_stream_running = false;
    if (!dr16_ring_buffer_ready)
    {
        g_dr16_monitor.stream_restart_error_count++;
        return false;
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

    // 只有确认 UART 不再向回调交付字节后，才复位生产者与消费者共享的 RingBuffer
    if (BSP_UART_StreamStop(BSP_UART_DR16) != BSP_OK)
    {
        g_dr16_monitor.last_uart_error = BSP_UART_StreamGetError(BSP_UART_DR16);
        g_dr16_monitor.stream_restart_error_count++;
        return false;
    }
    lwrb_reset(&dr16_ring_buffer);

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
 * @brief 按优先级处理错误恢复、接收解码和离线检测
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
        // 错误优先于缓存数据处理，立即发布离线安全状态并丢弃全部未验证字节
        DR16_TaskSetOffline(true);
        DR16_TaskRecoverStream();
        DR16_TaskUpdateMonitor(now_us);
        return;
    }

    if ((flags & DR16_TASK_FLAG_RX_DATA) != 0U)
    {
        // 对本轮积累的全部候选字节执行合法帧提交和逐字节重同步
        DR16_TaskProcessFrames(now_us);
    }

    // 无论本轮是否有新字节都检查 100 毫秒离线阈值并刷新调试状态
    DR16_TaskCheckOffline(now_us);
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
    uint64_t frame_age_us = 0U;
    if (dr16_state.header.last_online_time != 0U && now_us >= dr16_state.header.last_online_time)
    {
        frame_age_us = now_us - dr16_state.header.last_online_time;
    }
    if (frame_age_us > UINT32_MAX)
    {
        frame_age_us = UINT32_MAX;
    }

    g_dr16_monitor.current_frame_age_us = (uint32_t)frame_age_us;
    g_dr16_monitor.online = dr16_state.header.online;
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
