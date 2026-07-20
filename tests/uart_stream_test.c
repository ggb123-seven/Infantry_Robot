#include "bsp/uart.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

#define UART_STREAM_TEST_MAX_CHUNKS (8U)
#define UART_STREAM_TEST_MAX_CHUNK_SIZE (64U)

UART_HandleTypeDef huart3;
static UART_HandleTypeDef other_uart;
static HAL_StatusTypeDef mock_receive_status;
static HAL_StatusTypeDef mock_abort_status;
static HAL_StatusTypeDef mock_io_status;
static uint8_t *mock_dma_buffer;
static uint16_t mock_dma_buffer_size;
static uint32_t mock_receive_call_count;
static uint32_t mock_abort_call_count;
static bool mock_hal_receive_active;
static uint8_t received_chunk[UART_STREAM_TEST_MAX_CHUNKS][UART_STREAM_TEST_MAX_CHUNK_SIZE];
static uint16_t received_chunk_length[UART_STREAM_TEST_MAX_CHUNKS];
static uint32_t received_chunk_count;
static uint32_t received_error_code;
static uint32_t received_error_count;
static uint32_t ordinary_callback_count;

static void UARTStreamTest_ResetMocks(void);
static void UARTStreamTest_PrepareRunningStream(void);
static void UARTStreamTest_RxCallback(const uint8_t *data, uint16_t length);
static void UARTStreamTest_ErrorCallback(uint32_t error_code);
static void UARTStreamTest_OrdinaryCallback(void);
static void UARTStreamTest_PreservesOrdinaryUartApi(void);
static void UARTStreamTest_ValidatesRegistrationAndStart(void);
static void UARTStreamTest_DeliversOnlyNewContiguousBytes(void);
static void UARTStreamTest_DeliversWrappedBytesInOrder(void);
static void UARTStreamTest_ReportsHalErrorAndRestarts(void);
static void UARTStreamTest_RejectsInvalidDmaPosition(void);

/**
 * @brief 运行 UART 循环 DMA 字节流主机测试
 *
 * @return 全部断言通过时返回 0
 */
int main(void)
{
    UARTStreamTest_PreservesOrdinaryUartApi();
    UARTStreamTest_ValidatesRegistrationAndStart();
    UARTStreamTest_DeliversOnlyNewContiguousBytes();
    UARTStreamTest_DeliversWrappedBytesInOrder();
    UARTStreamTest_ReportsHalErrorAndRestarts();
    UARTStreamTest_RejectsInvalidDmaPosition();

    puts("UART stream tests passed");
    return 0;
}

/**
 * @brief 模拟 HAL 启动 Receive-to-Idle DMA 并保存缓冲区
 *
 * @param[in] huart UART 句柄
 * @param[out] data DMA 接收缓冲区
 * @param[in] size DMA 接收缓冲区容量
 * @return 当前测试配置的 HAL 状态
 */
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{
    assert(huart == &huart3);
    mock_dma_buffer = data;
    mock_dma_buffer_size = size;
    mock_receive_call_count++;
    if (mock_receive_status != HAL_OK)
    {
        return mock_receive_status;
    }
    if (mock_hal_receive_active)
    {
        return HAL_BUSY;
    }
    mock_hal_receive_active = true;
    return HAL_OK;
}

/**
 * @brief 模拟 HAL 同步终止 UART 接收
 *
 * @param[in] huart UART 句柄
 * @return 当前测试配置的 HAL 状态
 */
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart)
{
    assert(huart == &huart3);
    mock_abort_call_count++;
    if (mock_abort_status == HAL_OK)
    {
        mock_hal_receive_active = false;
    }
    return mock_abort_status;
}

/**
 * @brief 返回测试 UART 句柄中的错误位
 *
 * @param[in] huart UART 句柄
 * @return 当前 UART 错误位
 */
uint32_t HAL_UART_GetError(const UART_HandleTypeDef *huart)
{
    return huart->ErrorCode;
}

/**
 * @brief 模拟普通 UART HAL 收发接口
 *
 * @param[in] huart UART 句柄
 * @param[in,out] data 数据缓冲区
 * @param[in] size 数据字节数
 * @return 固定返回 HAL_OK
 */
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{
    (void)huart;
    (void)data;
    (void)size;
    return mock_io_status;
}

/**
 * @brief 模拟中断方式 UART 发送
 *
 * @param[in] huart UART 句柄
 * @param[in] data 数据缓冲区
 * @param[in] size 数据字节数
 * @return 固定返回 HAL_OK
 */
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{
    return HAL_UART_Transmit_DMA(huart, data, size);
}

/**
 * @brief 模拟 DMA 方式 UART 固定长度接收
 *
 * @param[in] huart UART 句柄
 * @param[out] data 数据缓冲区
 * @param[in] size 数据字节数
 * @return 固定返回 HAL_OK
 */
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{
    return HAL_UART_Transmit_DMA(huart, data, size);
}

/**
 * @brief 模拟中断方式 UART 固定长度接收
 *
 * @param[in] huart UART 句柄
 * @param[out] data 数据缓冲区
 * @param[in] size 数据字节数
 * @return 固定返回 HAL_OK
 */
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{
    return HAL_UART_Transmit_DMA(huart, data, size);
}

/**
 * @brief 清空 mock 状态并确保上一轮接收已停止
 *
 * @return 无返回值
 */
static void UARTStreamTest_ResetMocks(void)
{
    mock_abort_status = HAL_OK;
    BSP_UART_StreamStop(BSP_UART_DR16);

    memset(&huart3, 0, sizeof(huart3));
    huart3.Instance = USART3;
    memset(&other_uart, 0, sizeof(other_uart));
    memset(received_chunk, 0, sizeof(received_chunk));
    memset(received_chunk_length, 0, sizeof(received_chunk_length));
    mock_receive_status = HAL_OK;
    mock_abort_status = HAL_OK;
    mock_io_status = HAL_OK;
    mock_dma_buffer = NULL;
    mock_dma_buffer_size = 0U;
    mock_receive_call_count = 0U;
    mock_abort_call_count = 0U;
    mock_hal_receive_active = false;
    received_chunk_count = 0U;
    received_error_code = 0U;
    received_error_count = 0U;
    ordinary_callback_count = 0U;
}

/**
 * @brief 注册测试回调并启动字节流
 *
 * @return 无返回值
 */
static void UARTStreamTest_PrepareRunningStream(void)
{
    UARTStreamTest_ResetMocks();
    assert(BSP_UART_StreamRegisterCallbacks(BSP_UART_DR16, UARTStreamTest_RxCallback,
                                            UARTStreamTest_ErrorCallback) == BSP_OK);
    assert(BSP_UART_StreamStart(BSP_UART_DR16) == BSP_OK);
    assert(mock_dma_buffer != NULL);
    assert(mock_dma_buffer_size == UART_STREAM_TEST_MAX_CHUNK_SIZE);
}

/**
 * @brief 复制 BSP 上报的新增字节段
 *
 * @param[in] data 新增字节段首地址
 * @param[in] length 新增字节数
 * @return 无返回值
 */
static void UARTStreamTest_RxCallback(const uint8_t *data, uint16_t length)
{
    assert(received_chunk_count < UART_STREAM_TEST_MAX_CHUNKS);
    assert(length <= UART_STREAM_TEST_MAX_CHUNK_SIZE);
    memcpy(received_chunk[received_chunk_count], data, length);
    received_chunk_length[received_chunk_count] = length;
    received_chunk_count++;
}

/**
 * @brief 保存 BSP 上报的最近一次错误
 *
 * @param[in] error_code UART 错误位或 BSP 内部错误
 * @return 无返回值
 */
static void UARTStreamTest_ErrorCallback(uint32_t error_code)
{
    received_error_code = error_code;
    received_error_count++;
}

/**
 * @brief 记录普通 UART 回调执行次数
 *
 * @return 无返回值
 */
static void UARTStreamTest_OrdinaryCallback(void)
{
    ordinary_callback_count++;
}

/**
 * @brief 验证融合后仍保留普通 UART 句柄、回调和固定长度收发能力
 *
 * @return 无返回值
 */
static void UARTStreamTest_PreservesOrdinaryUartApi(void)
{
    UARTStreamTest_ResetMocks();
    uint8_t data = 0U;

    assert(BSP_UART_GetHandle(BSP_UART_DR16) == &huart3);
    assert(BSP_UART_GetHandle(BSP_UART_ERR) == NULL);
    assert(BSP_UART_RegisterCallback(BSP_UART_ERR, BSP_UART_TX_CPLT_CB, UARTStreamTest_OrdinaryCallback) == BSP_ERR);
    assert(BSP_UART_RegisterCallback(BSP_UART_DR16, BSP_UART_CB_NUM, UARTStreamTest_OrdinaryCallback) == BSP_ERR);
    assert(BSP_UART_RegisterCallback(BSP_UART_DR16, BSP_UART_TX_CPLT_CB, NULL) == BSP_ERR_NULL);
    assert(BSP_UART_RegisterCallback(BSP_UART_DR16, BSP_UART_TX_CPLT_CB,
                                     UARTStreamTest_OrdinaryCallback) == BSP_OK);
    HAL_UART_TxCpltCallback(&huart3);
    assert(ordinary_callback_count == 1U);

    assert(BSP_UART_Transmit(BSP_UART_DR16, &data, 1U, true) == BSP_OK);
    assert(BSP_UART_Receive(BSP_UART_DR16, &data, 1U, false) == BSP_OK);
    assert(BSP_UART_Transmit(BSP_UART_ERR, &data, 1U, true) == BSP_ERR);
    assert(BSP_UART_Receive(BSP_UART_DR16, NULL, 1U, true) == BSP_ERR_NULL);

    mock_io_status = HAL_ERROR;
    assert(BSP_UART_Transmit(BSP_UART_DR16, &data, 1U, false) == BSP_ERR);
    assert(BSP_UART_Receive(BSP_UART_DR16, &data, 1U, true) == BSP_ERR);
}

/**
 * @brief 验证注册、启动、停止和启动失败处理
 *
 * @return 无返回值
 */
static void UARTStreamTest_ValidatesRegistrationAndStart(void)
{
    UARTStreamTest_ResetMocks();
    assert(BSP_UART_StreamStart(BSP_UART_DR16) == BSP_ERR_NO_DEV);
    assert(BSP_UART_StreamRegisterCallbacks(BSP_UART_ERR, UARTStreamTest_RxCallback,
                                            UARTStreamTest_ErrorCallback) == BSP_ERR);
    assert(BSP_UART_StreamRegisterCallbacks(BSP_UART_DR16, NULL, UARTStreamTest_ErrorCallback) == BSP_ERR_NULL);

    assert(BSP_UART_StreamRegisterCallbacks(BSP_UART_DR16, UARTStreamTest_RxCallback,
                                            UARTStreamTest_ErrorCallback) == BSP_OK);
    mock_receive_status = HAL_ERROR;
    huart3.ErrorCode = HAL_UART_ERROR_DMA;
    assert(BSP_UART_StreamStart(BSP_UART_DR16) == BSP_ERR);
    assert(BSP_UART_StreamGetError(BSP_UART_DR16) == HAL_UART_ERROR_DMA);

    mock_receive_status = HAL_OK;
    assert(BSP_UART_StreamStart(BSP_UART_DR16) == BSP_OK);
    assert(BSP_UART_StreamStart(BSP_UART_DR16) == BSP_ERR_INITED);
    assert(BSP_UART_StreamRegisterCallbacks(BSP_UART_DR16, UARTStreamTest_RxCallback,
                                            UARTStreamTest_ErrorCallback) == BSP_ERR_INITED);
    assert(BSP_UART_StreamStop(BSP_UART_DR16) == BSP_OK);
    assert(mock_abort_call_count == 1U);
    assert(BSP_UART_StreamGetError(BSP_UART_ERR) == BSP_UART_STREAM_ERROR_INVALID_UART);
}

/**
 * @brief 验证连续事件只交付尚未处理的新字节
 *
 * @return 无返回值
 */
static void UARTStreamTest_DeliversOnlyNewContiguousBytes(void)
{
    UARTStreamTest_PrepareRunningStream();
    for (uint16_t index = 0U; index < 32U; index++)
    {
        mock_dma_buffer[index] = (uint8_t)index;
    }

    HAL_UARTEx_RxEventCallback(&huart3, 18U);
    assert(received_chunk_count == 1U);
    assert(received_chunk_length[0] == 18U);
    assert(received_chunk[0][17] == 17U);

    HAL_UARTEx_RxEventCallback(&huart3, 18U);
    assert(received_chunk_count == 1U);

    HAL_UARTEx_RxEventCallback(&huart3, 32U);
    assert(received_chunk_count == 2U);
    assert(received_chunk_length[1] == 14U);
    assert(received_chunk[1][0] == 18U);
    assert(received_chunk[1][13] == 31U);

    HAL_UARTEx_RxEventCallback(&other_uart, 20U);
    assert(received_chunk_count == 2U);
}

/**
 * @brief 验证 DMA 回绕时按尾部、头部顺序交付两个区段
 *
 * @return 无返回值
 */
static void UARTStreamTest_DeliversWrappedBytesInOrder(void)
{
    UARTStreamTest_PrepareRunningStream();
    for (uint16_t index = 0U; index < 60U; index++)
    {
        mock_dma_buffer[index] = (uint8_t)index;
    }
    HAL_UARTEx_RxEventCallback(&huart3, 60U);

    for (uint16_t index = 60U; index < 64U; index++)
    {
        mock_dma_buffer[index] = (uint8_t)(0xA0U + index - 60U);
    }
    for (uint16_t index = 0U; index < 12U; index++)
    {
        mock_dma_buffer[index] = (uint8_t)(0xB0U + index);
    }
    HAL_UARTEx_RxEventCallback(&huart3, 12U);

    assert(received_chunk_count == 3U);
    assert(received_chunk_length[1] == 4U);
    assert(received_chunk[1][0] == 0xA0U);
    assert(received_chunk[1][3] == 0xA3U);
    assert(received_chunk_length[2] == 12U);
    assert(received_chunk[2][0] == 0xB0U);
    assert(received_chunk[2][11] == 0xBBU);
}

/**
 * @brief 验证 HAL 错误会停止交付并可由任务重新启动
 *
 * @return 无返回值
 */
static void UARTStreamTest_ReportsHalErrorAndRestarts(void)
{
    UARTStreamTest_PrepareRunningStream();
    huart3.ErrorCode = HAL_UART_ERROR_ORE;
    mock_hal_receive_active = false;
    HAL_UART_ErrorCallback(&huart3);
    assert(received_error_count == 1U);
    assert(received_error_code == HAL_UART_ERROR_ORE);
    assert(BSP_UART_StreamGetError(BSP_UART_DR16) == HAL_UART_ERROR_ORE);

    HAL_UARTEx_RxEventCallback(&huart3, 18U);
    assert(received_chunk_count == 0U);

    assert(BSP_UART_StreamRestart(BSP_UART_DR16) == BSP_OK);
    assert(mock_receive_call_count == 2U);
    assert(BSP_UART_StreamGetError(BSP_UART_DR16) == BSP_UART_STREAM_ERROR_NONE);
}

/**
 * @brief 验证非法 DMA 写入位置会停止交付并通知上层
 *
 * @return 无返回值
 */
static void UARTStreamTest_RejectsInvalidDmaPosition(void)
{
    UARTStreamTest_PrepareRunningStream();
    HAL_UARTEx_RxEventCallback(&huart3, UART_STREAM_TEST_MAX_CHUNK_SIZE + 1U);

    assert(received_error_count == 1U);
    assert(received_error_code == BSP_UART_STREAM_ERROR_DMA_POSITION);
    assert(BSP_UART_StreamGetError(BSP_UART_DR16) == BSP_UART_STREAM_ERROR_DMA_POSITION);
    HAL_UARTEx_RxEventCallback(&huart3, 18U);
    assert(received_chunk_count == 0U);
    assert(BSP_UART_StreamRestart(BSP_UART_DR16) == BSP_OK);
    assert(mock_abort_call_count == 1U);
    assert(mock_receive_call_count == 2U);
}
