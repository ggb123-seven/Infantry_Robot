#include "bsp/time.h"

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "main.h"
#include "task.h"

// 本工程使用 32 位内核节拍，微秒换算要求每个节拍对应整数微秒
_Static_assert(configUSE_16_BIT_TICKS == 0, "时间扩展要求 32 位内核节拍");
_Static_assert(1000000U % configTICK_RATE_HZ == 0U, "内核节拍必须能精确换算为微秒");

// 任务与 CAN 中断共用扩展状态，由下方最小中断临界区保护
static uint32_t previous_tick;
static uint64_t tick_epoch;

/**
 * @brief 返回内核启动后的毫秒低 32 位
 *
 * @return 毫秒计数，按无符号差值处理回绕
 */
uint32_t BSP_TIME_Get_ms(void)
{
    return (uint32_t)((uint64_t)osKernelGetTickCount() * 1000U / configTICK_RATE_HZ);
}

/**
 * @brief 读取完整微秒时间，补偿待处理节拍并扩展 32 位节拍回绕
 *
 * 调用方保证内核节拍已启动，且相邻调用间隔小于一圈 32 位节拍
 * 本工程由周期任务持续调用，临界区只采样时间和更新扩展状态
 *
 * @return 单调微秒时间，任务和允许调用 RTOS 接口的中断均可使用
 */
uint64_t BSP_TIME_Get_us(void)
{
    const uint32_t irq_mask = __get_PRIMASK();
    __disable_irq();

    // 读取同一节拍内的硬件位置，溢出已挂起时重新采样并补上尚未入账的节拍
    uint32_t ticks = osKernelGetTickCount();
    uint32_t counter = SysTick->VAL;
    const uint32_t reload = SysTick->LOAD + 1U;
    if ((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) != 0U)
    {
        ticks++;
        counter = SysTick->VAL;
    }
    if (ticks < previous_tick)
    {
        tick_epoch += UINT64_C(1) << 32;
    }
    previous_tick = ticks;
    const uint64_t full_ticks = tick_epoch + ticks;
    __set_PRIMASK(irq_mask);

    // 从乘法开始使用 64 位，避免微秒值在约 71.6 分钟处先按 32 位溢出
    const uint32_t tick_us = 1000000U / configTICK_RATE_HZ;
    return full_ticks * tick_us + (uint64_t)(reload - counter) * tick_us / reload;
}

uint64_t BSP_TIME_Get(void) __attribute__((alias("BSP_TIME_Get_us")));

/**
 * @brief 根据内核状态选择任务延时或 HAL 延时
 *
 * @param[in] ms 延时间隔，单位毫秒
 * @return 可执行延时返回 BSP_OK，内核状态不允许等待时返回 BSP_ERR
 */
int8_t BSP_TIME_Delay_ms(uint32_t ms)
{
    const uint32_t ticks = (uint32_t)((uint64_t)ms * configTICK_RATE_HZ / 1000U);
    switch (osKernelGetState())
    {
        case osKernelError:
        case osKernelReserved:
        case osKernelLocked:
        case osKernelSuspended:
            return BSP_ERR;

        case osKernelRunning:
            osDelay(ticks != 0U ? ticks : 1U);
            break;

        case osKernelInactive:
        case osKernelReady:
            HAL_Delay(ms);
            break;
    }
    return BSP_OK;
}

/**
 * @brief 阻塞等待指定微秒间隔，仅用于设备要求的短等待
 *
 * @param[in] us 等待时长，单位微秒
 * @return 等待完成返回 BSP_OK
 */
int8_t BSP_TIME_Delay_us(uint32_t us)
{
    const uint64_t start = BSP_TIME_Get_us();
    // 轮询完整时间差，等待期间允许内核节拍推进
    while (BSP_TIME_Get_us() - start < us)
    {
    }
    return BSP_OK;
}

int8_t BSP_TIME_Delay(uint32_t ms) __attribute__((alias("BSP_TIME_Delay_ms")));
