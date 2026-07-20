#include "bsp/uart.h"
#include "task/dr16_task.h"
#include "task/user_task.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

Task_Runtime_t task_runtime;

static BSP_UART_StreamRxCallback_t mock_rx_callback;
static BSP_UART_StreamErrorCallback_t mock_error_callback;
static DR16_State_t mock_mailbox_state;
static uint32_t mock_pending_flags;
static uint32_t mock_mailbox_publish_count;
static uint32_t mock_stream_start_count;
static uint32_t mock_stream_stop_count;
static uint32_t mock_stream_error;
static int mock_queue_token;
static int mock_thread_token;

static void DR16TaskTest_ResetMocks(void);
static void DR16TaskTest_WriteUint16LittleEndian(uint8_t *data, uint16_t value);
static void DR16TaskTest_PackFrame(uint8_t frame[DR16_FRAME_LENGTH], const uint16_t channel[5]);
static void DR16TaskTest_EmitData(const uint8_t *data, uint16_t length, uint64_t now_us);
static void DR16TaskTest_EmitError(uint32_t error_code, uint64_t now_us);
static void DR16TaskTest_ResynchronizesByteByByte(void);
static void DR16TaskTest_PublishesOfflineOnceAfterTimeout(void);
static void DR16TaskTest_RecoversFromRingBufferOverflow(void);
static void DR16TaskTest_RecoversFromUartError(void);

/**
 * @brief 运行 DR16 任务主机测试
 *
 * @return 全部断言通过时返回 0
 */
int main(void)
{
    DR16TaskTest_ResynchronizesByteByByte();
    DR16TaskTest_PublishesOfflineOnceAfterTimeout();
    DR16TaskTest_RecoversFromRingBufferOverflow();
    DR16TaskTest_RecoversFromUartError();

    puts("DR16 task tests passed");
    return 0;
}

/**
 * @brief 模拟设置 DR16 任务线程标志
 *
 * @param[in] thread_id 目标任务句柄
 * @param[in] flags 待设置标志
 * @return 设置后的任务标志
 */
uint32_t osThreadFlagsSet(osThreadId_t thread_id, uint32_t flags)
{
    assert(thread_id == &mock_thread_token);
    mock_pending_flags |= flags;
    return mock_pending_flags;
}

/**
 * @brief 模拟线程标志等待超时
 *
 * @param[in] flags 等待的标志集合
 * @param[in] options 等待选项
 * @param[in] timeout 超时节拍数
 * @return 固定返回超时状态
 */
uint32_t osThreadFlagsWait(uint32_t flags, uint32_t options, uint32_t timeout)
{
    (void)flags;
    (void)options;
    (void)timeout;
    return osFlagsErrorTimeout;
}

/**
 * @brief 模拟任务延时
 *
 * @param[in] ticks 延时节拍数
 * @return 固定返回成功
 */
osStatus_t osDelay(uint32_t ticks)
{
    assert(ticks > 0U);
    return osOK;
}

/**
 * @brief 返回主机测试使用的内核节拍频率
 *
 * @return 固定返回 1000 赫兹
 */
uint32_t osKernelGetTickFreq(void)
{
    return 1000U;
}

/**
 * @brief 模拟清空长度为 1 的 DR16 状态邮箱
 *
 * @param[in] message_queue 邮箱句柄
 * @return 句柄正确时返回成功
 */
osStatus_t osMessageQueueReset(osMessageQueueId_t message_queue)
{
    assert(message_queue == &mock_queue_token);
    memset(&mock_mailbox_state, 0, sizeof(mock_mailbox_state));
    return osOK;
}

/**
 * @brief 模拟写入 DR16 状态邮箱
 *
 * @param[in] message_queue 邮箱句柄
 * @param[in] message 待发布状态
 * @param[in] priority 消息优先级
 * @param[in] timeout 等待节拍数
 * @return 参数正确时返回成功
 */
osStatus_t osMessageQueuePut(osMessageQueueId_t message_queue, const void *message, uint8_t priority, uint32_t timeout)
{
    assert(message_queue == &mock_queue_token);
    assert(message != NULL);
    assert(priority == 0U);
    assert(timeout == 0U);
    mock_mailbox_state = *(const DR16_State_t *)message;
    mock_mailbox_publish_count++;
    return osOK;
}

/**
 * @brief 模拟注册 UART 字节流回调
 *
 * @param[in] uart 板级 UART 端口
 * @param[in] rx_callback 新字节回调
 * @param[in] error_callback 错误回调
 * @return 参数正确时返回成功
 */
int8_t BSP_UART_StreamRegisterCallbacks(BSP_UART_t uart, BSP_UART_StreamRxCallback_t rx_callback,
                                        BSP_UART_StreamErrorCallback_t error_callback)
{
    assert(uart == BSP_UART_DR16);
    assert(rx_callback != NULL);
    assert(error_callback != NULL);
    mock_rx_callback = rx_callback;
    mock_error_callback = error_callback;
    return BSP_OK;
}

/**
 * @brief 模拟启动 UART 循环 DMA 字节流
 *
 * @param[in] uart 板级 UART 端口
 * @return 固定返回成功
 */
int8_t BSP_UART_StreamStart(BSP_UART_t uart)
{
    assert(uart == BSP_UART_DR16);
    mock_stream_start_count++;
    return BSP_OK;
}

/**
 * @brief 模拟停止 UART 循环 DMA 字节流
 *
 * @param[in] uart 板级 UART 端口
 * @return 固定返回成功
 */
int8_t BSP_UART_StreamStop(BSP_UART_t uart)
{
    assert(uart == BSP_UART_DR16);
    mock_stream_stop_count++;
    return BSP_OK;
}

/**
 * @brief 返回最近一次模拟 UART 错误
 *
 * @param[in] uart 板级 UART 端口
 * @return 最近一次模拟错误码
 */
uint32_t BSP_UART_StreamGetError(BSP_UART_t uart)
{
    assert(uart == BSP_UART_DR16);
    return mock_stream_error;
}

/**
 * @brief 提供任务入口链接所需的模拟微秒时间
 *
 * @return 固定返回 0
 */
uint64_t BSP_TIME_Get(void)
{
    return 0U;
}

/**
 * @brief 清空主机测试状态并初始化 DR16 任务
 *
 * @return 无返回值
 */
static void DR16TaskTest_ResetMocks(void)
{
    memset(&task_runtime, 0, sizeof(task_runtime));
    memset(&mock_mailbox_state, 0, sizeof(mock_mailbox_state));
    mock_rx_callback = NULL;
    mock_error_callback = NULL;
    mock_pending_flags = 0U;
    mock_mailbox_publish_count = 0U;
    mock_stream_start_count = 0U;
    mock_stream_stop_count = 0U;
    mock_stream_error = 0U;
    task_runtime.thread.dr16 = &mock_thread_token;
    task_runtime.msgq.dr16_state = &mock_queue_token;

    assert(DR16_TaskTestInitialize());
    assert(mock_rx_callback != NULL);
    assert(mock_error_callback != NULL);
    assert(mock_mailbox_publish_count == 1U);
    assert(!mock_mailbox_state.header.online);
    assert(mock_stream_start_count == 1U);
}

/**
 * @brief 按小端序写入无符号 16 位测试字段
 *
 * @param[out] data 两字节字段首地址
 * @param[in] value 待写入数值
 * @return 无返回值
 */
static void DR16TaskTest_WriteUint16LittleEndian(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0x00FFU);
    data[1] = (uint8_t)(value >> 8U);
}

/**
 * @brief 将五个通道和两个合法拨杆编码为测试帧
 *
 * @param[out] frame 十八字节测试帧
 * @param[in] channel 五个通道原始值
 * @return 无返回值
 */
static void DR16TaskTest_PackFrame(uint8_t frame[DR16_FRAME_LENGTH], const uint16_t channel[5])
{
    memset(frame, 0, DR16_FRAME_LENGTH);
    frame[0] = (uint8_t)channel[0];
    frame[1] = (uint8_t)((channel[0] >> 8U) | (channel[1] << 3U));
    frame[2] = (uint8_t)((channel[1] >> 5U) | (channel[2] << 6U));
    frame[3] = (uint8_t)(channel[2] >> 2U);
    frame[4] = (uint8_t)((channel[2] >> 10U) | (channel[3] << 1U));
    frame[5] = (uint8_t)((channel[3] >> 7U) | ((uint16_t)DR16_SWITCH_UP << 4U) |
                         ((uint16_t)DR16_SWITCH_MIDDLE << 6U));
    DR16TaskTest_WriteUint16LittleEndian(&frame[16], channel[4]);
}

/**
 * @brief 通过已注册回调送入字节并执行一次任务处理
 *
 * @param[in] data 待送入字节
 * @param[in] length 字节数
 * @param[in] now_us 当前时间，单位微秒
 * @return 无返回值
 */
static void DR16TaskTest_EmitData(const uint8_t *data, uint16_t length, uint64_t now_us)
{
    mock_rx_callback(data, length);
    const uint32_t flags = mock_pending_flags;
    mock_pending_flags = 0U;
    DR16_TaskTestHandleFlags(flags, now_us);
}

/**
 * @brief 通过已注册回调送入 UART 错误并执行一次任务处理
 *
 * @param[in] error_code 模拟 UART 错误码
 * @param[in] now_us 当前时间，单位微秒
 * @return 无返回值
 */
static void DR16TaskTest_EmitError(uint32_t error_code, uint64_t now_us)
{
    mock_stream_error = error_code;
    mock_error_callback(error_code);
    const uint32_t flags = mock_pending_flags;
    mock_pending_flags = 0U;
    DR16_TaskTestHandleFlags(flags, now_us);
}

/**
 * @brief 验证前置噪声按字节丢弃并恢复到合法帧边界
 *
 * @return 无返回值
 */
static void DR16TaskTest_ResynchronizesByteByByte(void)
{
    DR16TaskTest_ResetMocks();
    const uint16_t channel[5] =
    {
        1024U, 1124U, 924U, 1224U, 1024U,
    };
    uint8_t stream[DR16_FRAME_LENGTH + 3U] =
    {
        0xFFU, 0xFFU, 0xFFU,
    };
    DR16TaskTest_PackFrame(&stream[3], channel);

    DR16TaskTest_EmitData(stream, sizeof(stream), 1000U);

    assert(mock_mailbox_state.header.online);
    assert(mock_mailbox_state.header.last_online_time == 1000U);
    assert(mock_mailbox_state.valid_frame_sequence == 1U);
    assert(mock_mailbox_state.data.ch_r_y == 100);
    assert(g_dr16_monitor.valid_frame_count == 1U);
    assert(g_dr16_monitor.invalid_frame_count == 3U);
    assert(g_dr16_monitor.resync_discarded_byte_count == 3U);
}

/**
 * @brief 验证 100 毫秒无合法帧只发布一次离线零状态
 *
 * @return 无返回值
 */
static void DR16TaskTest_PublishesOfflineOnceAfterTimeout(void)
{
    DR16TaskTest_ResetMocks();
    const uint16_t channel[5] =
    {
        1024U, 1024U, 1024U, 1024U, 1024U,
    };
    uint8_t frame[DR16_FRAME_LENGTH];
    DR16TaskTest_PackFrame(frame, channel);
    DR16TaskTest_EmitData(frame, sizeof(frame), 1000U);
    const uint32_t online_publish_count = mock_mailbox_publish_count;

    DR16_TaskTestHandleFlags(0U, 100999U);
    assert(mock_mailbox_state.header.online);
    assert(mock_mailbox_publish_count == online_publish_count);

    DR16_TaskTestHandleFlags(0U, 101000U);
    assert(!mock_mailbox_state.header.online);
    assert(mock_mailbox_state.header.last_online_time == 1000U);
    assert(mock_mailbox_state.valid_frame_sequence == 1U);
    const DR16_Data_t zero_data =
    {
        0,
    };
    assert(memcmp(&mock_mailbox_state.data, &zero_data, sizeof(zero_data)) == 0);
    assert(mock_mailbox_publish_count == online_publish_count + 1U);

    DR16_TaskTestHandleFlags(0U, 201000U);
    assert(mock_mailbox_publish_count == online_publish_count + 1U);
}

/**
 * @brief 验证 RingBuffer 溢出后丢弃旧数据并等待新合法帧恢复
 *
 * @return 无返回值
 */
static void DR16TaskTest_RecoversFromRingBufferOverflow(void)
{
    DR16TaskTest_ResetMocks();
    const uint16_t channel[5] =
    {
        1024U, 1024U, 1024U, 1024U, 1024U,
    };
    uint8_t frame[DR16_FRAME_LENGTH];
    DR16TaskTest_PackFrame(frame, channel);
    DR16TaskTest_EmitData(frame, sizeof(frame), 1000U);

    uint8_t oversized_data[200];
    memset(oversized_data, 0xFF, sizeof(oversized_data));
    DR16TaskTest_EmitData(oversized_data, sizeof(oversized_data), 2000U);

    assert(!mock_mailbox_state.header.online);
    assert(mock_mailbox_state.valid_frame_sequence == 1U);
    assert(g_dr16_monitor.ring_buffer_overflow_count == 1U);
    assert(mock_stream_stop_count == 1U);
    assert(mock_stream_start_count == 2U);

    DR16TaskTest_EmitData(frame, sizeof(frame), 3000U);
    assert(mock_mailbox_state.header.online);
    assert(mock_mailbox_state.valid_frame_sequence == 2U);
}

/**
 * @brief 验证 UART 错误立即离线、重启并等待新合法帧恢复
 *
 * @return 无返回值
 */
static void DR16TaskTest_RecoversFromUartError(void)
{
    DR16TaskTest_ResetMocks();
    const uint16_t channel[5] =
    {
        1024U, 1024U, 1024U, 1024U, 1024U,
    };
    uint8_t frame[DR16_FRAME_LENGTH];
    DR16TaskTest_PackFrame(frame, channel);
    DR16TaskTest_EmitData(frame, sizeof(frame), 1000U);

    DR16TaskTest_EmitError(0x08U, 2000U);
    assert(!mock_mailbox_state.header.online);
    assert(mock_mailbox_state.valid_frame_sequence == 1U);
    assert(g_dr16_monitor.uart_error_count == 1U);
    assert(g_dr16_monitor.last_uart_error == 0x08U);
    assert(mock_stream_stop_count == 1U);
    assert(mock_stream_start_count == 2U);

    DR16TaskTest_EmitData(frame, sizeof(frame), 3000U);
    assert(mock_mailbox_state.header.online);
    assert(mock_mailbox_state.valid_frame_sequence == 2U);
}
