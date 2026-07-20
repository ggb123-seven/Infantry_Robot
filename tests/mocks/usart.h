#pragma once

#include <stdint.h>

typedef enum
{
    HAL_OK = 0,
    HAL_ERROR = 1,
    HAL_BUSY = 2,
    HAL_TIMEOUT = 3,
} HAL_StatusTypeDef;

typedef struct
{
    void *Instance;
    uint32_t ErrorCode;
} UART_HandleTypeDef;

extern UART_HandleTypeDef huart3;

#define USART3 ((void *)0x00000003UL)
#define UART_FLAG_IDLE (0x00000010UL)
#define RESET (0U)
#define __HAL_UART_GET_FLAG(huart, flag) (0U)
#define __HAL_UART_CLEAR_IDLEFLAG(huart) ((void)(huart))

#define HAL_UART_ERROR_NONE (0x00000000UL)
#define HAL_UART_ERROR_PE (0x00000001UL)
#define HAL_UART_ERROR_NE (0x00000002UL)
#define HAL_UART_ERROR_FE (0x00000004UL)
#define HAL_UART_ERROR_ORE (0x00000008UL)
#define HAL_UART_ERROR_DMA (0x00000010UL)

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size);
uint32_t HAL_UART_GetError(const UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);
