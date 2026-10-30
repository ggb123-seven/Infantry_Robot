# INS 全零与启动堆不足排查

## 板上证据

- 通过 EmberProbe 已运行的 OpenOCD TCL 端口进行只读检查，没有停核、复位或烧录
- 目标为 STM32F407，状态为 running；两个 Flash 装载段共 131148 字节与修改前 Debug ELF 完全一致
- `INS` 位于 `0x20005e0c`，292 字节全部为零，包含生命周期、状态码、四元数和更新计数
- `task_runtime` 位于 `0x20006918`，全部业务线程和邮箱句柄为零，`init_status = 0x1b`
- `0x1b` 对应 `TASK_INIT_SHOOT_THREAD_FAILED`，Task_Init 创建射击线程失败后执行现有回收流程，INS 在调度解锁前被删除
- `configTOTAL_HEAP_SIZE = 15360`，`xMinimumEverFreeBytesRemaining = 0x340 = 832` 字节，不足以分配 1024 字节任务栈
- 回收后 `xFreeBytesRemaining = 0x2910 = 10512`，不能用回收后的堆余量否定启动峰值不足
- 因此 `INS.status = INS_STATUS_OK(0)` 在这里是全局实例的零初始化结果，不代表已完成初始化或采集

## 修复

- 仅修改 `User/task/user_task.c`，为创建顺序末尾的 DR16、射击和拨弹任务提供静态控制块和静态栈
- 使用既有 `StaticTask_t` 与 `StackType_t`，没有定义结构体或修改实例字段
- 每个栈继续为 1024 字节，按 8 字节对齐；控制块各为 92 字节，总静态存储 3348 字节
- CMSIS-RTOS2 的 osThreadNew 在 cb_mem、cb_size、stack_mem、stack_size 完整时调用 xTaskCreateStatic
- 保留集中创建、返回值检查、失败回收与默认安全输出；没有修改 .ioc、FreeRTOSConfig.h、设备驱动或控制参数

## 验证

- `cmake --build --preset Debug`：2/2 成功，无警告，RAM 47048 B，FLASH 131148 B
- `python .auto-embedded/scripts/check.py`：ARCH、HW、SPEC 全部 PASS
- `git diff --check`：无差异格式错误
- 通过 Python 标准库解析 ELF，逐一核对三个属性对象的控制块和栈地址、大小、8 字节栈对齐、SRAM 范围和存储区域互不重叠，全部通过
- 源码扫描：120 列、行尾空白与 Allman 花括号全部通过；无新增函数、循环、ISR 共享状态或临界区
- 复核 RTOS 创建和删除路径：创建结果检查仍在 Task_Init，静态任务由 FreeRTOS 按静态分配类型回收内核状态，不释放静态存储
- 新增判空、isfinite、范围检查、保护性 return：无；新增重复校验及删除项：无
- 验证代码使用标准输入运行，没有生成临时测试源码或可执行文件

## 验证边界与下一步

- 已实测确认旧固件的启动失败原因；修复只完成构建和静态验证，尚未烧录，不能宣称真实 IMU 已恢复
- 烧录后重新加载对应 ELF，核对 task_runtime.init_status 为 TASK_INIT_OK、INS.module_state 为 RUNNING、INS.UpdateCount 持续增加
- 静止初始化完成后观察重力加速度和温度，再转动板子检查三轴角速度及欧拉角；任务栈高水位与运行堆余量仍需实测
