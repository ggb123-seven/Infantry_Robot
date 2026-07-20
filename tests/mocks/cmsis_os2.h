#pragma once

#include <stddef.h>
#include <stdint.h>

#define osFlagsWaitAny (0x00000000U)
#define osFlagsError (0x80000000U)
#define osFlagsErrorTimeout (0xFFFFFFFEU)

typedef void *osThreadId_t;
typedef void *osMessageQueueId_t;

typedef enum
{
    osOK = 0,
    osError = -1,
} osStatus_t;

typedef enum
{
    osPriorityNormal = 24,
    osPriorityAboveNormal = 32,
    osPriorityRealtime = 48,
} osPriority_t;

typedef struct
{
    const char *name;
    uint32_t attr_bits;
    void *cb_mem;
    uint32_t cb_size;
    void *stack_mem;
    uint32_t stack_size;
    osPriority_t priority;
    uint32_t tz_module;
    uint32_t reserved;
} osThreadAttr_t;

typedef struct
{
    const char *name;
} osMessageQueueAttr_t;

typedef void (*osThreadFunc_t)(void *argument);

int32_t osKernelLock(void);
int32_t osKernelUnlock(void);
uint32_t osKernelGetTickFreq(void);
osThreadId_t osThreadNew(osThreadFunc_t function, void *argument, const osThreadAttr_t *attr);
void osThreadExit(void);
osStatus_t osThreadTerminate(osThreadId_t thread_id);
uint32_t osThreadFlagsSet(osThreadId_t thread_id, uint32_t flags);
uint32_t osThreadFlagsWait(uint32_t flags, uint32_t options, uint32_t timeout);
osStatus_t osDelay(uint32_t ticks);
osMessageQueueId_t osMessageQueueNew(uint32_t message_count, uint32_t message_size,
                                     const osMessageQueueAttr_t *attr);
osStatus_t osMessageQueueDelete(osMessageQueueId_t message_queue);
osStatus_t osMessageQueueReset(osMessageQueueId_t message_queue);
osStatus_t osMessageQueuePut(osMessageQueueId_t message_queue, const void *message, uint8_t priority, uint32_t timeout);
