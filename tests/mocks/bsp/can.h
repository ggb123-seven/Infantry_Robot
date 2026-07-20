#pragma once

#include <stdint.h>

#define BSP_OK (0)
#define BSP_ERR (-1)

typedef enum
{
    BSP_CAN_1,
    BSP_CAN_2,
    BSP_CAN_NUM,
    BSP_CAN_ERR,
} BSP_CAN_t;

int8_t BSP_CAN_Init(void);
