#include "bsp/uart.h"

#include <stddef.h>

/*
 * UART 循环 DMA 接收参数：
 * - BSP_UART_STREAM_RX_BUFFER_SIZE：每个板级 UART 的 DMA 循环缓冲区容量，单位字节。
 * - 64 字节可覆盖多个连续接收周期，HT、TC 和 IDLE 事件共同推进读取位置。
 * @configuration CubeMX DMA1 Stream1 Channel 4 Circular 模式
 */
#define BSP_UART_STREAM_RX_BUFFER_SIZE (64U)

/*
 * UART 字节流运行状态：
 * - rx_callback、error_callback：由上层任务在启动前注册的 ISR 回调。
 * - last_position：最近一次已交付的 DMA 写入位置，范围 0~BSP_UART_STREAM_RX_BUFFER_SIZE。
 * - last_error：最近一次 HAL UART 错误位或 BSP 内部错误。
 * - callbacks_registered：数据和错误回调均注册成功时为 true。
 * - receive_active：HAL Receive-to-Idle DMA 已启动且尚未终止时为 true。
 * - running：允许向上层交付新增字节时为 true。
 */
typedef struct
{
    BSP_UART_StreamRxCallback_t rx_callback;
    BSP_UART_StreamErrorCallback_t error_callback;
    volatile uint16_t last_position;
    volatile uint32_t last_error;
    bool callbacks_registered;
    volatile bool receive_active;
    volatile bool running;
} BSP_UART_StreamState_t;

static void (*uart_callback[BSP_UART_NUM][BSP_UART_CB_NUM])(void);
static uint8_t uart_stream_buffer[BSP_UART_NUM][BSP_UART_STREAM_RX_BUFFER_SIZE];
static BSP_UART_StreamState_t uart_stream_state[BSP_UART_NUM];

static bool UART_IsValid(BSP_UART_t uart);
static BSP_UART_t UART_Get(UART_HandleTypeDef *uart_handle);
static void UART_InvokeCallback(UART_HandleTypeDef *uart_handle, BSP_UART_Callback_t callback_type);
static uint32_t UART_StreamReadHalError(BSP_UART_t uart);
static void UART_StreamNotifyError(BSP_UART_t uart, uint32_t error_code);
static void UART_StreamHandleRxEvent(BSP_UART_t uart, uint16_t position);

/**
 * @brief 转发 HAL UART 发送完成事件
 *
 * @param[in] huart 产生事件的 HAL UART 句柄
 * @return 无返回值
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    UART_InvokeCallback(huart, BSP_UART_TX_CPLT_CB);
}

/**
 * @brief 转发 HAL UART 发送半完成事件
 *
 * @param[in] huart 产生事件的 HAL UART 句柄
 * @return 无返回值
 */
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    UART_InvokeCallback(huart, BSP_UART_TX_HALF_CPLT_CB);
}

/**
 * @brief 转发 HAL UART 固定长度接收完成事件
 *
 * @param[in] huart 产生事件的 HAL UART 句柄
 * @return 无返回值
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    UART_InvokeCallback(huart, BSP_UART_RX_CPLT_CB);
}

/**
 * @brief 转发 HAL UART 固定长度接收半完成事件
 *
 * @param[in] huart 产生事件的 HAL UART 句柄
 * @return 无返回值
 */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    UART_InvokeCallback(huart, BSP_UART_RX_HALF_CPLT_CB);
}

/**
 * @brief 记录 UART 字节流错误并转发普通错误回调
 *
 * @param[in] huart 产生错误的 HAL UART 句柄
 * @return 无返回值
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    const BSP_UART_t uart = UART_Get(huart);
    if (!UART_IsValid(uart))
    {
        return;
    }

    BSP_UART_StreamState_t *state = &uart_stream_state[uart];
    if (state->receive_active || state->running)
    {
        state->receive_active = false;
        UART_StreamNotifyError(uart, UART_StreamReadHalError(uart));
    }
    UART_InvokeCallback(huart, BSP_UART_ERROR_CB);
}

/**
 * @brief 转发 HAL UART 全部操作终止完成事件
 *
 * @param[in] huart 产生事件的 HAL UART 句柄
 * @return 无返回值
 */
void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart)
{
    UART_InvokeCallback(huart, BSP_UART_ABORT_CPLT_CB);
}

/**
 * @brief 转发 HAL UART 发送终止完成事件
 *
 * @param[in] huart 产生事件的 HAL UART 句柄
 * @return 无返回值
 */
void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart)
{
    UART_InvokeCallback(huart, BSP_UART_ABORT_TX_CPLT_CB);
}

/**
 * @brief 转发 HAL UART 接收终止完成事件
 *
 * @param[in] huart 产生事件的 HAL UART 句柄
 * @return 无返回值
 */
void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart)
{
    UART_InvokeCallback(huart, BSP_UART_ABORT_RX_CPLT_CB);
}

/**
 * @brief 将 HAL Receive-to-Idle 事件路由到对应板级 UART 字节流
 *
 * @param[in] huart 产生接收事件的 HAL UART 句柄
 * @param[in] size DMA 从缓冲区起点累计写入的位置，范围 0~缓冲区容量
 * @return 无返回值
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    const BSP_UART_t uart = UART_Get(huart);
    if (UART_IsValid(uart))
    {
        UART_StreamHandleRxEvent(uart, size);
    }
}

/**
 * @brief 处理不使用 HAL Receive-to-Idle 时的传统 UART IDLE 事件
 *
 * @param[in] huart 产生中断的 HAL UART 句柄
 * @return 无返回值
 */
void BSP_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    const BSP_UART_t uart = UART_Get(huart);
    if (!UART_IsValid(uart) || uart_stream_state[uart].receive_active)
    {
        return;
    }

    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        if (uart_callback[uart][BSP_UART_IDLE_LINE_CB] != NULL)
        {
            uart_callback[uart][BSP_UART_IDLE_LINE_CB]();
        }
    }
}

/**
 * @brief 获取板级 UART 对应的 HAL 句柄
 *
 * @param[in] uart 板级 UART 端口
 * @return 有效端口返回 HAL 句柄，否则返回 NULL
 */
UART_HandleTypeDef *BSP_UART_GetHandle(BSP_UART_t uart)
{
    switch (uart)
    {
        case BSP_UART_DR16:
            return &huart3;

        default:
            return NULL;
    }
}

/**
 * @brief 注册普通 UART 回调
 *
 * @param[in] uart 板级 UART 端口
 * @param[in] type 回调类型
 * @param[in] callback 无参数回调函数
 * @return 成功返回 BSP_OK，失败返回对应 BSP 状态码
 */
int8_t BSP_UART_RegisterCallback(BSP_UART_t uart, BSP_UART_Callback_t type, void (*callback)(void))
{
    if (!UART_IsValid(uart) || (uint32_t)type >= (uint32_t)BSP_UART_CB_NUM)
    {
        return BSP_ERR;
    }
    if (callback == NULL)
    {
        return BSP_ERR_NULL;
    }

    uart_callback[uart][type] = callback;
    return BSP_OK;
}

/**
 * @brief 使用中断或 DMA 发送 UART 数据
 *
 * @param[in] uart 板级 UART 端口
 * @param[in] data 待发送数据
 * @param[in] size 待发送字节数
 * @param[in] dma 使用 DMA 时为 true，使用中断时为 false
 * @return 成功返回 BSP_OK，失败返回对应 BSP 状态码
 */
int8_t BSP_UART_Transmit(BSP_UART_t uart, uint8_t *data, uint16_t size, bool dma)
{
    if (!UART_IsValid(uart))
    {
        return BSP_ERR;
    }
    if (data == NULL || size == 0U)
    {
        return BSP_ERR_NULL;
    }

    UART_HandleTypeDef *uart_handle = BSP_UART_GetHandle(uart);
    const HAL_StatusTypeDef status = dma ? HAL_UART_Transmit_DMA(uart_handle, data, size)
                                         : HAL_UART_Transmit_IT(uart_handle, data, size);
    return status == HAL_OK ? BSP_OK : BSP_ERR;
}

/**
 * @brief 使用中断或 DMA 接收固定长度 UART 数据
 *
 * @param[in] uart 板级 UART 端口
 * @param[out] data 接收缓冲区
 * @param[in] size 接收字节数
 * @param[in] dma 使用 DMA 时为 true，使用中断时为 false
 * @return 成功返回 BSP_OK，失败返回对应 BSP 状态码
 */
int8_t BSP_UART_Receive(BSP_UART_t uart, uint8_t *data, uint16_t size, bool dma)
{
    if (!UART_IsValid(uart))
    {
        return BSP_ERR;
    }
    if (data == NULL || size == 0U)
    {
        return BSP_ERR_NULL;
    }

    UART_HandleTypeDef *uart_handle = BSP_UART_GetHandle(uart);
    const HAL_StatusTypeDef status = dma ? HAL_UART_Receive_DMA(uart_handle, data, size)
                                         : HAL_UART_Receive_IT(uart_handle, data, size);
    return status == HAL_OK ? BSP_OK : BSP_ERR;
}

/**
 * @brief 注册 UART 循环 DMA 字节流的数据与错误回调
 *
 * 运行期间禁止替换回调，所有回调必须在启动接收前完成注册。
 *
 * @param[in] uart 板级 UART 端口
 * @param[in] rx_callback 新增数据回调
 * @param[in] error_callback 接收错误回调
 * @return 成功返回 BSP_OK，失败返回对应 BSP 状态码
 */
int8_t BSP_UART_StreamRegisterCallbacks(BSP_UART_t uart, BSP_UART_StreamRxCallback_t rx_callback,
                                        BSP_UART_StreamErrorCallback_t error_callback)
{
    if (!UART_IsValid(uart))
    {
        return BSP_ERR;
    }
    if (rx_callback == NULL || error_callback == NULL)
    {
        return BSP_ERR_NULL;
    }

    BSP_UART_StreamState_t *state = &uart_stream_state[uart];
    if (state->running || state->receive_active)
    {
        return BSP_ERR_INITED;
    }

    state->rx_callback = rx_callback;
    state->error_callback = error_callback;
    state->callbacks_registered = true;
    return BSP_OK;
}

/**
 * @brief 启动 UART Receive-to-Idle 循环 DMA 接收
 *
 * @param[in] uart 板级 UART 端口
 * @return 成功返回 BSP_OK，失败返回对应 BSP 状态码
 */
int8_t BSP_UART_StreamStart(BSP_UART_t uart)
{
    if (!UART_IsValid(uart))
    {
        return BSP_ERR;
    }

    BSP_UART_StreamState_t *state = &uart_stream_state[uart];
    if (!state->callbacks_registered)
    {
        return BSP_ERR_NO_DEV;
    }
    if (state->running || state->receive_active)
    {
        return BSP_ERR_INITED;
    }

    UART_HandleTypeDef *uart_handle = BSP_UART_GetHandle(uart);
    if (uart_handle == NULL)
    {
        return BSP_ERR_NO_DEV;
    }

    // 先建立零位置基准，再启动 HAL 接收并开放事件交付
    state->last_position = 0U;
    state->last_error = BSP_UART_STREAM_ERROR_NONE;
    if (HAL_UARTEx_ReceiveToIdle_DMA(uart_handle, uart_stream_buffer[uart], BSP_UART_STREAM_RX_BUFFER_SIZE) != HAL_OK)
    {
        state->last_error = UART_StreamReadHalError(uart);
        return BSP_ERR;
    }
    state->receive_active = true;
    state->running = true;
    return BSP_OK;
}

/**
 * @brief 停止 UART 循环 DMA 接收
 *
 * @param[in] uart 板级 UART 端口
 * @return 成功返回 BSP_OK，失败返回对应 BSP 状态码
 */
int8_t BSP_UART_StreamStop(BSP_UART_t uart)
{
    if (!UART_IsValid(uart))
    {
        return BSP_ERR;
    }

    BSP_UART_StreamState_t *state = &uart_stream_state[uart];
    if (!state->receive_active)
    {
        state->running = false;
        return BSP_OK;
    }

    UART_HandleTypeDef *uart_handle = BSP_UART_GetHandle(uart);
    if (uart_handle == NULL)
    {
        return BSP_ERR_NO_DEV;
    }

    // 先关闭事件交付，再同步终止 HAL 接收，避免停止期间发布不完整区段
    state->running = false;
    if (HAL_UART_AbortReceive(uart_handle) != HAL_OK)
    {
        state->last_error = UART_StreamReadHalError(uart);
        return BSP_ERR;
    }

    state->receive_active = false;
    state->last_position = 0U;
    return BSP_OK;
}

/**
 * @brief 重新启动 UART Receive-to-Idle 循环 DMA 接收
 *
 * @param[in] uart 板级 UART 端口
 * @return 成功返回 BSP_OK，失败返回对应 BSP 状态码
 */
int8_t BSP_UART_StreamRestart(BSP_UART_t uart)
{
    const int8_t stop_status = BSP_UART_StreamStop(uart);
    if (stop_status != BSP_OK)
    {
        return stop_status;
    }
    return BSP_UART_StreamStart(uart);
}

/**
 * @brief 获取 UART 字节流最近一次错误
 *
 * @param[in] uart 板级 UART 端口
 * @return 最近一次 HAL UART 错误位或 BSP_UART_STREAM_ERROR_* 内部错误
 */
uint32_t BSP_UART_StreamGetError(BSP_UART_t uart)
{
    if (!UART_IsValid(uart))
    {
        return BSP_UART_STREAM_ERROR_INVALID_UART;
    }
    return uart_stream_state[uart].last_error;
}

/**
 * @brief 检查板级 UART 枚举是否有效
 *
 * @param[in] uart 板级 UART 端口
 * @return 有效返回 true，否则返回 false
 */
static bool UART_IsValid(BSP_UART_t uart)
{
    return (uint32_t)uart < (uint32_t)BSP_UART_NUM;
}

/**
 * @brief 根据 HAL UART 句柄查找板级端口
 *
 * @param[in] uart_handle HAL UART 句柄
 * @return 匹配时返回板级 UART 端口，否则返回 BSP_UART_ERR
 */
static BSP_UART_t UART_Get(UART_HandleTypeDef *uart_handle)
{
    if (uart_handle != NULL && uart_handle->Instance == USART3)
    {
        return BSP_UART_DR16;
    }
    return BSP_UART_ERR;
}

/**
 * @brief 将 HAL UART 事件转发到已注册的普通回调
 *
 * @param[in] uart_handle HAL UART 句柄
 * @param[in] callback_type 回调类型
 * @return 无返回值
 */
static void UART_InvokeCallback(UART_HandleTypeDef *uart_handle, BSP_UART_Callback_t callback_type)
{
    const BSP_UART_t uart = UART_Get(uart_handle);
    if (!UART_IsValid(uart) || (uint32_t)callback_type >= (uint32_t)BSP_UART_CB_NUM)
    {
        return;
    }
    if (uart_callback[uart][callback_type] != NULL)
    {
        uart_callback[uart][callback_type]();
    }
}

/**
 * @brief 读取 HAL UART 错误并为无具体错误位的失败补充内部错误
 *
 * @param[in] uart 板级 UART 端口
 * @return HAL UART 错误位或 BSP_UART_STREAM_ERROR_UNKNOWN
 */
static uint32_t UART_StreamReadHalError(BSP_UART_t uart)
{
    UART_HandleTypeDef *uart_handle = BSP_UART_GetHandle(uart);
    if (uart_handle == NULL)
    {
        return BSP_UART_STREAM_ERROR_UNKNOWN;
    }

    const uint32_t error_code = HAL_UART_GetError(uart_handle);
    return error_code != HAL_UART_ERROR_NONE ? error_code : BSP_UART_STREAM_ERROR_UNKNOWN;
}

/**
 * @brief 记录错误、停止事件交付并通知上层任务
 *
 * @param[in] uart 板级 UART 端口
 * @param[in] error_code HAL UART 错误位或 BSP 内部错误
 * @return 无返回值
 */
static void UART_StreamNotifyError(BSP_UART_t uart, uint32_t error_code)
{
    BSP_UART_StreamState_t *state = &uart_stream_state[uart];
    state->last_error = error_code;
    state->last_position = 0U;
    state->running = false;
    if (state->error_callback != NULL)
    {
        state->error_callback(error_code);
    }
}

/**
 * @brief 根据循环 DMA 写入位置交付本次新增的一个或两个连续字节段
 *
 * @param[in] uart 板级 UART 端口
 * @param[in] position DMA 从缓冲区起点累计写入的位置，范围 0~缓冲区容量
 * @return 无返回值
 */
static void UART_StreamHandleRxEvent(BSP_UART_t uart, uint16_t position)
{
    BSP_UART_StreamState_t *state = &uart_stream_state[uart];
    if (!state->running)
    {
        return;
    }
    if (position > BSP_UART_STREAM_RX_BUFFER_SIZE)
    {
        UART_StreamNotifyError(uart, BSP_UART_STREAM_ERROR_DMA_POSITION);
        return;
    }
    if (position == state->last_position)
    {
        return;
    }

    // 正向推进时交付单个连续区段，回绕时依次交付缓冲区尾部和头部
    if (position > state->last_position)
    {
        state->rx_callback(&uart_stream_buffer[uart][state->last_position], position - state->last_position);
    }
    else
    {
        const uint16_t tail_length = BSP_UART_STREAM_RX_BUFFER_SIZE - state->last_position;
        if (tail_length > 0U)
        {
            state->rx_callback(&uart_stream_buffer[uart][state->last_position], tail_length);
        }
        if (position > 0U)
        {
            state->rx_callback(uart_stream_buffer[uart], position);
        }
    }
    state->last_position = position;
}
