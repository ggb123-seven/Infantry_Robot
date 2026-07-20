#include "device/dr16.h"
#include "task/dr16_task.h"
#include "task/motor_chassis.h"
#include "task/user_task.h"

#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

static jmp_buf mock_thread_exit_jump;
static int mock_queue_token;
static int mock_motor_thread_token;
static int mock_dr16_thread_token;
static int32_t mock_kernel_lock_result;
static int32_t mock_kernel_unlock_result;
static uint32_t mock_queue_create_count;
static uint32_t mock_queue_delete_count;
static uint32_t mock_thread_create_count;
static uint32_t mock_thread_terminate_count;
static uint32_t mock_motor_chassis_init_count;
static uint32_t mock_call_sequence;
static uint32_t mock_kernel_lock_sequence;
static uint32_t mock_motor_chassis_init_sequence;
static uint32_t mock_failed_thread_create_index;
static bool mock_queue_create_fails;
static bool mock_motor_chassis_init_result;

static void InitTaskTest_ResetMocks(void);
static void InitTaskTest_Run(void);
static void InitTaskTest_CreatesAllObjects(void);
static void InitTaskTest_StopsWhenQueueCreationFails(void);
static void InitTaskTest_CleansUpAfterMotorCreationFails(void);
static void InitTaskTest_CleansUpAfterDr16CreationFails(void);
static void InitTaskTest_StopsWhenMotorChassisInitializationFails(void);
static void InitTaskTest_StopsWhenKernelLockFails(void);

/**
 * @brief 运行业务 RTOS 对象创建结果主机测试
 *
 * @return 全部断言通过时返回 0
 */
int main(void)
{
    InitTaskTest_CreatesAllObjects();
    InitTaskTest_StopsWhenQueueCreationFails();
    InitTaskTest_CleansUpAfterMotorCreationFails();
    InitTaskTest_CleansUpAfterDr16CreationFails();
    InitTaskTest_StopsWhenMotorChassisInitializationFails();
    InitTaskTest_StopsWhenKernelLockFails();

    puts("Init task tests passed");
    return 0;
}

/**
 * @brief 模拟内核调度锁定
 *
 * @return 当前测试配置的结果
 */
int32_t osKernelLock(void)
{
    mock_kernel_lock_sequence = ++mock_call_sequence;
    return mock_kernel_lock_result;
}

/**
 * @brief 模拟内核调度解锁
 *
 * @return 当前测试配置的结果
 */
int32_t osKernelUnlock(void)
{
    return mock_kernel_unlock_result;
}

/**
 * @brief 模拟创建长度为 1 的 DR16 状态邮箱
 *
 * @param[in] message_count 邮箱容量
 * @param[in] message_size 单条消息大小
 * @param[in] attr 邮箱属性
 * @return 成功时返回模拟句柄，失败时返回 NULL
 */
osMessageQueueId_t osMessageQueueNew(uint32_t message_count, uint32_t message_size,
                                     const osMessageQueueAttr_t *attr)
{
    assert(message_count == 1U);
    assert(message_size == sizeof(DR16_State_t));
    assert(attr == NULL);
    mock_queue_create_count++;
    return mock_queue_create_fails ? NULL : &mock_queue_token;
}

/**
 * @brief 模拟删除 DR16 状态邮箱
 *
 * @param[in] message_queue 邮箱句柄
 * @return 固定返回成功
 */
osStatus_t osMessageQueueDelete(osMessageQueueId_t message_queue)
{
    assert(message_queue == &mock_queue_token);
    mock_queue_delete_count++;
    return osOK;
}

/**
 * @brief 模拟按顺序创建底盘和 DR16 任务
 *
 * @param[in] function 任务入口
 * @param[in] argument 任务参数
 * @param[in] attr 任务属性
 * @return 成功时返回对应模拟句柄，配置失败时返回 NULL
 */
osThreadId_t osThreadNew(osThreadFunc_t function, void *argument, const osThreadAttr_t *attr)
{
    assert(argument == NULL);
    assert(attr != NULL);
    mock_thread_create_count++;
    if (mock_thread_create_count == mock_failed_thread_create_index)
    {
        return NULL;
    }
    if (function == Task_motor_chassis)
    {
        assert(attr == &attr_motor_chassis);
        return &mock_motor_thread_token;
    }

    assert(function == Task_dr16);
    assert(attr == &attr_dr16);
    return &mock_dr16_thread_token;
}

/**
 * @brief 模拟终止尚未运行的业务任务
 *
 * @param[in] thread_id 任务句柄
 * @return 固定返回成功
 */
osStatus_t osThreadTerminate(osThreadId_t thread_id)
{
    assert(thread_id == &mock_motor_thread_token || thread_id == &mock_dr16_thread_token);
    mock_thread_terminate_count++;
    return osOK;
}

/**
 * @brief 结束本轮 Task_Init 测试调用
 *
 * @return 无返回值
 */
void osThreadExit(void)
{
    longjmp(mock_thread_exit_jump, 1);
}

/**
 * @brief 提供链接所需的空底盘任务入口
 *
 * @param[in] argument 任务参数
 * @return 无返回值
 */
void Task_motor_chassis(void *argument)
{
    (void)argument;
}

/**
 * @brief 提供底盘任务私有控制链初始化模拟结果
 *
 * @return 当前测试配置的初始化结果
 */
bool Task_motor_chassis_Init(void)
{
    mock_motor_chassis_init_count++;
    mock_motor_chassis_init_sequence = ++mock_call_sequence;
    return mock_motor_chassis_init_result;
}

/**
 * @brief 提供链接所需的空 DR16 任务入口
 *
 * @param[in] argument 任务参数
 * @return 无返回值
 */
void Task_dr16(void *argument)
{
    (void)argument;
}

/**
 * @brief 清空对象创建模拟状态
 *
 * @return 无返回值
 */
static void InitTaskTest_ResetMocks(void)
{
    memset(&task_runtime, 0, sizeof(task_runtime));
    mock_kernel_lock_result = 0;
    mock_kernel_unlock_result = 0;
    mock_queue_create_count = 0U;
    mock_queue_delete_count = 0U;
    mock_thread_create_count = 0U;
    mock_thread_terminate_count = 0U;
    mock_motor_chassis_init_count = 0U;
    mock_call_sequence = 0U;
    mock_kernel_lock_sequence = 0U;
    mock_motor_chassis_init_sequence = 0U;
    mock_failed_thread_create_index = 0U;
    mock_queue_create_fails = false;
    mock_motor_chassis_init_result = true;
}

/**
 * @brief 执行 Task_Init 并在模拟线程退出时返回
 *
 * @return 无返回值
 */
static void InitTaskTest_Run(void)
{
    if (setjmp(mock_thread_exit_jump) == 0)
    {
        Task_Init(NULL);
        assert(false);
    }
}

/**
 * @brief 验证邮箱和两个任务全部创建成功
 *
 * @return 无返回值
 */
static void InitTaskTest_CreatesAllObjects(void)
{
    InitTaskTest_ResetMocks();
    InitTaskTest_Run();

    assert(task_runtime.init_status == TASK_INIT_OK);
    assert(task_runtime.msgq.dr16_state == &mock_queue_token);
    assert(task_runtime.thread.motor_chassis == &mock_motor_thread_token);
    assert(task_runtime.thread.dr16 == &mock_dr16_thread_token);
    assert(mock_queue_create_count == 1U);
    assert(mock_thread_create_count == 2U);
    assert(mock_motor_chassis_init_count == 1U);
    assert(mock_motor_chassis_init_sequence < mock_kernel_lock_sequence);
    assert(mock_queue_delete_count == 0U);
    assert(mock_thread_terminate_count == 0U);
}

/**
 * @brief 验证邮箱创建失败后不会创建业务任务
 *
 * @return 无返回值
 */
static void InitTaskTest_StopsWhenQueueCreationFails(void)
{
    InitTaskTest_ResetMocks();
    mock_queue_create_fails = true;
    InitTaskTest_Run();

    assert(task_runtime.init_status == TASK_INIT_DR16_MAILBOX_FAILED);
    assert(mock_queue_create_count == 1U);
    assert(mock_thread_create_count == 0U);
    assert(mock_motor_chassis_init_count == 1U);
    assert(mock_queue_delete_count == 0U);
    assert(mock_thread_terminate_count == 0U);
}

/**
 * @brief 验证底盘任务创建失败后删除已创建邮箱
 *
 * @return 无返回值
 */
static void InitTaskTest_CleansUpAfterMotorCreationFails(void)
{
    InitTaskTest_ResetMocks();
    mock_failed_thread_create_index = 1U;
    InitTaskTest_Run();

    assert(task_runtime.init_status == TASK_INIT_MOTOR_THREAD_FAILED);
    assert(task_runtime.msgq.dr16_state == NULL);
    assert(mock_queue_delete_count == 1U);
    assert(mock_thread_terminate_count == 0U);
    assert(mock_motor_chassis_init_count == 1U);
}

/**
 * @brief 验证 DR16 任务创建失败后终止底盘任务并删除邮箱
 *
 * @return 无返回值
 */
static void InitTaskTest_CleansUpAfterDr16CreationFails(void)
{
    InitTaskTest_ResetMocks();
    mock_failed_thread_create_index = 2U;
    InitTaskTest_Run();

    assert(task_runtime.init_status == TASK_INIT_DR16_THREAD_FAILED);
    assert(task_runtime.thread.motor_chassis == NULL);
    assert(task_runtime.msgq.dr16_state == NULL);
    assert(mock_queue_delete_count == 1U);
    assert(mock_thread_terminate_count == 1U);
    assert(mock_motor_chassis_init_count == 1U);
}

/**
 * @brief 验证底盘控制链初始化失败后不创建任何业务对象
 *
 * @return 无返回值
 */
static void InitTaskTest_StopsWhenMotorChassisInitializationFails(void)
{
    InitTaskTest_ResetMocks();
    mock_motor_chassis_init_result = false;
    InitTaskTest_Run();

    assert(task_runtime.init_status == TASK_INIT_MOTOR_CHASSIS_FAILED);
    assert(mock_motor_chassis_init_count == 1U);
    assert(mock_queue_create_count == 0U);
    assert(mock_thread_create_count == 0U);
    assert(mock_queue_delete_count == 0U);
    assert(mock_thread_terminate_count == 0U);
}

/**
 * @brief 验证内核锁定失败后不创建任何业务对象
 *
 * @return 无返回值
 */
static void InitTaskTest_StopsWhenKernelLockFails(void)
{
    InitTaskTest_ResetMocks();
    mock_kernel_lock_result = -1;
    InitTaskTest_Run();

    assert(task_runtime.init_status == TASK_INIT_KERNEL_LOCK_FAILED);
    assert(mock_queue_create_count == 0U);
    assert(mock_thread_create_count == 0U);
    assert(mock_motor_chassis_init_count == 1U);
}
