# 编码与协作约定 spec（conventions）

> 本层是"怎么写才安全"的可执行约定。EXECUTE/REVIEW 必读。

## 证据优先（铁律）

没有以下任一证据，**禁止**宣称"已修好/应该没问题"：
代码位置 · 编译输出 · 测试结果 · 串口日志 · 数据手册依据 · 网表依据 · 实测波形/寄存器状态。
完成声明必须在当前回复内附"命令 + 输出 + 与验证标准的对照"。

## 复用优先

本地离线索引 → 官方文档/Context7 → 开源驱动 → 最后才自己写。不凭记忆猜接口。

## 代码规范

- 模块化：`.c` + `.h` 成对；公共 API 走 `.h`，内部函数 `static`。
- 命名前缀按层（halport_/bsp_/drv_/mw_/svc_/app_）。
- `volatile`：ISR 与主循环共享变量、MMIO 必须 `volatile`；不滥用。
- 临界区：多字节共享状态读写用临界区/原子；Cortex-M0+ 无 LDREX 注意。
- ISR 纪律：ISR 体 ≤ 20 行，只置标志/搬数据，重活交主循环；ISR 内禁 `printf`/阻塞/动态内存。
- 魔数写回注释来源（`@datasheet p.XX` / `@netlist`）。

## Git 快照

EXECUTE 每完成一个清单项 + 用户确认 → 本地 `git add <具体文件>`（**不用 -A**）→ commit；
**绝不自动 push**。敏感文件（.env/.key/.pem/*secret*/*token*/id_rsa*）暂停存档。

## 沉淀（promote 回流）
> REVIEW 阶段把每次的设计决策/约定/坑/gotcha 沉淀于此，下次自动注入。
- [约定] Keil ARMCC 工程使用 FreeRTOS V10.3.1 时，必须包含 portable/RVDS/ARM_CM4F/port.c 与 portmacro.h，并将实际启用的外设初始化源文件、HAL 驱动源文件和 HAL 时基源文件纳入 .uvprojx。
- [约定] VS Code 的 STM32Cube J-Link 调试出现 JLinkGDBServerCL 被 SIGTERM 终止时，该行通常是调试适配器失败后的清理结果；应先检查此前的 GDB Remote 握手错误。若 RTOS 代理链路不稳定，可在 launch.json 中显式设置 serverRtos.enabled=false，使 GDB 直连 J-Link GDB Server。
- [坑/gotcha] RoboMaster RM 电机参数中的 id 使用反馈 CAN ID（0x200 + 电调 ID）；C620 电调 ID 1 应填 0x201，而不是 1。MOTOR_RM_SetOutput、MOTOR_RM_SetTorqueCurrent 和 MOTOR_RM_Relax 只更新发送缓存，随后必须调用 MOTOR_RM_FlushGroup 或 MOTOR_RM_FlushCAN 才会真正下发。
- [约定] RoboMaster C620/M3508 首次接入时，由唯一底盘任务拥有 0x200 控制帧发送权；上电默认持续发送零电流，先确认 0x201~0x208 反馈、物理 ID 和安装方向，再通过明确使能条件开放非零输出。
- [坑/gotcha] J-Link 调试现象与源码不一致时，先比较目标 Flash 0x08000000 的初始 MSP/Reset 向量和待调试 ELF 的 .isr_vector；VS Code 的自动二进制选择可能下载旧 AXF/ELF，应将 imageFileName 与 symbolFileName 显式固定到当前 CMake 产物。
- [可复用模式] CMake 工具链文件先查当前环境，再查显式根目录和 STM32Cube Bundle；同一工具链的 gcc、g++、objcopy 与 size 必须来自同一 bin 目录。
- [坑/gotcha] Windows PowerShell 5 会把无 BOM 的 UTF-8 脚本按系统代码页读取；含中文的兼容脚本必须保存为 UTF-8 BOM，并避免依赖新版 .NET API。
