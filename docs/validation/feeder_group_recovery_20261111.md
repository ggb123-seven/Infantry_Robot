# 拨弹五齿完成撤驱与未完成组恢复重计

日期：2026-11-11

## 已确认行为

用户以拨盘实际完成五齿作为五连发完成依据，并选择反馈恢复后未完成的组自动从当前位置重新拨五发，不要求先回中

- 右拨杆下位直接请求一组，默认五齿；每齿均须满足原有到位判据才推进下一齿
- 最后一齿误差不超过 0.5°、速度不超过 1 rpm 并连续保持 20 ms 后，立即进入完成等待释放阶段与 FEEDER_RELAX，DumpOutput 输出零
- 整组完成不等待下一齿启动间隔，持续下位、位置扰动、反馈无效或反馈超时及恢复都不重新启动已完成组
- 未完成组遇到角度无效或反馈超时撤驱；反馈恢复且下位请求仍有效时捕获当前位置，重新计完整五齿
- 回中立即取消当前组并撤驱，不继续位置保持；再次下位从新位置重新计发
- 单发同样完成一齿后撤驱；持续连发保留速度环接口和倍率语义
- 模式或有效环路切换取消旧组并重新建立当前模式，配置参数变更本身不重置已完成组
- 反馈帧龄恢复原有 10 ms 信任边界，低 32 位时间差按无符号数计算以支持回绕

这项恢复策略允许一次下位操作在多次未完成恢复中累计超过五齿；这是用户选择的“未完成就重新五发”行为，不承诺反馈反复异常时总共只转五齿。组完成状态一旦建立，反馈异常不清除它

## 实现范围

生产实现修改 User/module/feeder.c，接口与观测说明同步至 feeder.h、cmd.h、User/task/motor_trigger.h。既有 cmd.c 的下位直接提交请求与 feeder 进入时直接响应请求作为本步骤相关基础保留；配置、设备方向、任务栈与其他现有改动不纳入本步骤

复用全部现有结构体、阶段枚举和组内计数，没有新增结构体字段、RTOS 对象或 ISR 共享状态。单角度环仍默认开启、速度环仍默认关闭，未增加单齿执行超时或故障锁存

config.c 运行前后 SHA-256 均为 `142B7B57295F737E2C7A74CE0EAF2DE123B8818E2CBF074C39B12A527B4D354D`。RAM config 副本、Control 周期快照和 PID 对持久快照的引用保持生效

## 验证证据

真实 cmd.c、feeder.c、config.c、pid.c、filter.c、user_math.c 主机回归编译与运行退出码均为 0，`PASS checks=52512 completed_groups=13`

主机仅提取正式 BSP_CAN_t 枚举隔离 HAL，使用正式业务结构体；直接包含真实 feeder.c 观察私有执行阶段，没有复制状态机。编译参数为 `-std=gnu11 -fgnu89-inline -O0 -g -Wall -Wextra -Werror`，主机禁用云台正弦测试

覆盖：单角度和串级的单发/五发、连续下位不重复、回中取消与扰动零输出、未完成反馈恢复重计八次、第五齿刚启动但尚未完成的恢复、最后一齿无需等待十秒组内间隔、已完成后角度无效和断帧恢复、在线改参不复活完成组、10 ms 边界和超时恢复、源时间回绕与启动未来帧、持续 3.2 s 新反馈但未到位不追加目标、四种环路开关组合、速度连发、持久参数引用、非法配置/周期/状态、缺命令、真实遥控档位组合与过期输入

ARM 三文件严格语法检查采用当前 compile_commands.json 的配置，添加 `-fsyntax-only -Wextra -Werror`：cmd.c、feeder.c、motor_trigger.c 均退出 0

通过 aemb-build-cmake 脚本执行 `cmake --preset Debug -S F:\RM\Infantry_Robot` 与 `cmake --build --preset Debug`，构建成功。产物为 `build/Debug/Infantry_Robot.elf`，SHA-256 为 `B78018FF2E33D1518B583EB0CD4B039A53B36DDD6736710EB1B7536DDA488491`

ELF section 汇总：RAM `.data + .bss + ._user_heap_stack` 为 50416 B，FLASH 含初始化数据为 130512 B

## 栈与三层复核

- 拨弹任务静态栈仍为 1024 B，任务入口 .su 为 192 B，Feeder_Control 为 216 B，Feeder_UpdateSingle 为 32 B，PID_Calc 为 72 B，Feeder_UpdateFeedback 为 48 B
- Control 的普通位置计算路径到 PID 前为 192+216+72=480 B；该数值没有包含数学库、滤波子调用和中断上下文，不能代替完整峰值或运行时高水位
- 本轮没有成功建立板上读取连接，无运行时栈高水位或新固件实测，栈与机械响应均未完成板上验收
- 机械门禁采用进程级 PSExecutionPolicyPreference=Bypass 后运行 python .auto-embedded/scripts/check.py，ARCH/HW/SPEC 全部通过
- 分层保持 task → module → component/device → bsp，CAN 仍由通信任务统一输出，main.c、HAL、CubeMX 和硬件资源未改
- 执行阶段归拨弹模块独占；无新增中断共享变量、临界区、任务或队列；最终候选电流只在 Feeder_DumpOutput 根据运行状态放行
- 逐文件重读 AGENTS.md 与 agent.md，检查 Allman 花括号、120 列、中文目的注释与行尾空白，源码扫描及 git diff --check 通过

## 校验自查

1. 恢复 `age_us <= FEEDER_FEEDBACK_TIMEOUT_US` 与 `now_us >= age_us`：属于外部 CAN 反馈和时间信任边界，实际场景为反馈停止更新或启动阶段源时间晚于当前时间；驱动/通信任务没有检查同一源帧在本控制周期的缓存年龄，仅此入口检查
2. 恢复帧超时时角度失效和速度 NaN 观测赋值：是上述同一检查结果的发布，不在 Control 或 DumpOutput 重复检查源时间
3. 新增整组完成及释放条件属于业务阶段转换，没有新增判空、isfinite、范围兜底或保护性 return；完成后的退出复用原有非运行状态 return
4. 删除 UpdateSingle 内释放时的目标捕获与两次 PID_Reset：释放在 Control 转换到 RELAX 时统一处理，避免回中仍执行位置环和重复重置
5. 删除 INTERVAL 的剩余齿数重复判断：最终一齿到位直接进入 WAIT_RELEASE，INTERVAL 的入口契约保证仍有下一齿

## 验证边界

本轮未烧录、复位或写目标 RAM，没有调整 config.c 参数或设备方向。板上只读连接此前两次返回 OPENOCD_START_FAILED，未继续重复尝试。上述电流均为软件输出请求，软件回归不代表真实相电流、负载响应或弹丸出膛次数已验证

默认单角度环的开火过程过冲与到位振荡未做实物验收；本次处理回中/整组完成仍保持电流以及完成状态被反馈恢复清除的问题

临时回归源码、替身头文件和可执行程序在记录结果后清理，不保留为仓库正式测试资产
