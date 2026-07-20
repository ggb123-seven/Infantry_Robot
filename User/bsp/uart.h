#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>
#include <usart.h>

#include "bsp/bsp.h"

/*
 * UART 字节流内部错误：
 * - BSP_UART_STREAM_ERROR_NONE：当前没有错误。
 * - BSP_UART_STREAM_ERROR_UNKNOWN：HAL 操作失败但未提供具体错误位。
 * - BSP_UART_STREAM_ERROR_DMA_POSITION：HAL 上报的 DMA 写入位置超过接收缓冲区。
 * - BSP_UART_STREAM_ERROR_INVALID_UART：查询了不存在的 UART 端口。
 */
#define BSP_UART_STREAM_ERROR_NONE (0x00000000UL)
#define BSP_UART_STREAM_ERROR_UNKNOWN (0x40000000UL)
#define BSP_UART_STREAM_ERROR_DMA_POSITION (0x80000000UL)
#define BSP_UART_STREAM_ERROR_INVALID_UART (UINT32_MAX)

/**
 * @brief 板级 UART 端口
 */
typedef enum
{
    BSP_UART_DR16 = 0,
    BSP_UART_NUM,
    BSP_UART_ERR,
} BSP_UART_t;

/**
 * @brief 普通 UART 中断和 DMA 回调类型
 */
typedef enum
{
    BSP_UART_TX_HALF_CPLT_CB = 0,
    BSP_UART_TX_CPLT_CB,
    BSP_UART_RX_HALF_CPLT_CB,
    BSP_UART_RX_CPLT_CB,
    BSP_UART_ERROR_CB,
    BSP_UART_ABORT_CPLT_CB,
    BSP_UART_ABORT_TX_CPLT_CB,
    BSP_UART_ABORT_RX_CPLT_CB,
    BSP_UART_IDLE_LINE_CB,
    BSP_UART_CB_NUM,
} BSP_UART_Callback_t;

/**
 * @brief UART 字节流新增数据回调
 *
 * 数据指针只在回调执行期间有效，调用方必须立即复制需要保留的字节。
 *
 * @param[in] data 本次新增的连续字节段首地址
 * @param[in] length 本次新增的连续字节数
 * @return 无返回值
 */
typedef void (*BSP_UART_StreamRxCallback_t)(const uint8_t *data, uint16_t length);

/**
 * @brief UART 字节流错误回调
 *
 * @param[in] error_code HAL UART 错误位或 BSP_UART_STREAM_ERROR_* 内部错误
 * @return 无返回值
 */
typedef void (*BSP_UART_StreamErrorCallback_t)(uint32_t error_code);

/**
 * @brief 获取板级 UART 对应的 HAL 句柄
 *
 * @param[in] uart 板级 UART 端口
 * @return 有效端口返回 HAL 句柄，否则返回 NULL
 */
UART_HandleTypeDef *BSP_UART_GetHandle(BSP_UART_t uart);

/**
 * @brief 处理不使用 HAL Receive-to-Idle 时的传统 UART IDLE 事件
 *
 * @param[in] huart 产生中断的 HAL UART 句柄
 * @return 无返回值
 */
void BSP_UART_IRQHandler(UART_HandleTypeDef *huart);

/**
 * @brief 注册普通 UART 回调
 *
 * @param[in] uart 板级 UART 端口
 * @param[in] type 回调类型
 * @param[in] callback 无参数回调函数
 * @return 成功返回 BSP_OK，失败返回对应 BSP 状态码
 */
int8_t BSP_UART_RegisterCallback(BSP_UART_t uart, BSP_UART_Callback_t type, void (*callback)(void));

/**
 * @brief 使用中断或 DMA 发送 UART 数据
 *
 * @param[in] uart 板级 UART 端口
 * @param[in] data 待发送数据
 * @param[in] size 待发送字节数
 * @param[in] dma 使用 DMA 时为 true，使用中断时为 false
 * @return 成功返回 BSP_OK，失败返回对应 BSP 状态码
 */
int8_t BSP_UART_Transmit(BSP_UART_t uart, uint8_t *data, uint16_t size, bool dma);

/**
 * @brief 使用中断或 DMA 接收固定长度 UART 数据
 *
 * @param[in] uart 板级 UART 端口
 * @param[out] data 接收缓冲区
 * @param[in] size 接收字节数
 * @param[in] dma 使用 DMA 时为 true，使用中断时为 false
 * @return 成功返回 BSP_OK，失败返回对应 BSP 状态码
 */
int8_t BSP_UART_Receive(BSP_UART_t uart, uint8_t *data, uint16_t size, bool dma);

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
                                        BSP_UART_StreamErrorCallback_t error_callback);

/**
 * @brief 启动 UART Receive-to-Idle 循环 DMA 接收
 *
 * @param[in] uart 板级 UART 端口
 * @return 成功返回 BSP_OK，失败返回对应 BSP 状态码
 */
int8_t BSP_UART_StreamStart(BSP_UART_t uart);

/**
 * @brief 停止 UART 循环 DMA 接收
 *
 * @param[in] uart 板级 UART 端口
 * @return 成功返回 BSP_OK，失败返回对应 BSP 状态码
 */
int8_t BSP_UART_StreamStop(BSP_UART_t uart);

/**
 * @brief 重新启动 UART Receive-to-Idle 循环 DMA 接收
 *
 * @param[in] uart 板级 UART 端口
 * @return 成功返回 BSP_OK，失败返回对应 BSP 状态码
 */
int8_t BSP_UART_StreamRestart(BSP_UART_t uart);

/**
 * @brief 获取 UART 字节流最近一次错误
 *
 * @param[in] uart 板级 UART 端口
 * @return 最近一次 HAL UART 错误位或 BSP_UART_STREAM_ERROR_* 内部错误
 */
uint32_t BSP_UART_StreamGetError(BSP_UART_t uart);

#ifdef __cplusplus
}
#endif
