# 校验边界修复记录

基线：main / ac3a7c8，日期：2026-10-30

用户授权按前述范围修复，SMC 不处理；保留 config.c 全部内容和 EmberProbe 可写实例 config。优先复用已有变量，确需新增时按现有命名集中注释并在交付中说明，不再逐项确认

## 实施清单

1. BSP 时间：从运算起点使用 64 位，扩展 32 位 tick 回绕，补偿挂起 SysTick，最小中断临界区保护共享计时状态
2. 启动入口：检查 LED 线程与 Task_Init 创建结果，失败进入现有 Error_Handler
3. 四个电机模块：Control 快照并集中校验，参数指针继续引用持久 applied 参数；DumpOutput 仅按状态导出
4. 任务与命令链：删除固定对象生命周期重复检查，编译期验证固定节拍约束，保留实际消息失败、周期和输入时效检查
5. 同步 EmberProbe 名称，逐项记录新增与删除检查、构建和复现结果，清理临时测试并本地提交

## 步骤 1～2：时间与启动入口

修改 User/bsp/time.c 与 Src/freertos.c 用户代码区，无 .ioc 或 FreeRTOS 配置修改

- 时间在约 71.6 分钟边界由 4294966500 正确推进至 4294967500 us
- 32 位 tick 回绕由 4294967295500 正确推进至 4294967296500 us
- SysTick 挂起时补记一拍，处理完中断后读取连续；100000 次样本非递减，原 PRIMASK 为 0 或 1 均准确恢复
- 相邻读取必须小于一圈 32 位 tick，内核节拍必须已启动；本工程周期任务持续调用满足该契约。临界区只采样寄存器和更新高位，64 位单位换算在恢复中断后执行
- 实际启动函数四个线程创建失败注入和正常启动均通过，失败进入现有 Error_Handler，不新增运行时字段

验证：

```text
python build/validation_boundary_fix/run.py time       PASS
python build/validation_boundary_fix/run.py startup    PASS
python build/validation_boundary_fix/run.py strict     PASS（本步骤两个 C 文件）
python .auto-embedded/scripts/check.py                 ARCH/HW/SPEC PASS
python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug
Debug 构建 PASS
git diff --check -- User/bsp/time.c Src/freertos.c      PASS
```

主机测试直接编译生产 time.c / freertos.c，替身提供 SysTick、SCB、内核接口、线程创建和 Error_Handler。启动测试替身首轮缺少弱符号及 USB/延时桩，补齐后重新通过；生产启动代码既有弱任务参数未使用，严格检查仅对 freertos.c 关闭 unused-parameter，未关闭其他警告

本步骤新增检查：SysTick 挂起标志属于硬件时间采样边界；tick 低位回绕比较属于时间边界，均无上游同条件检查。启动线程判空属于对象创建边界，此前没有检查。原内核状态下禁止等待的返回保留，没有新增内部实例判空、isfinite 或参数兜底

本步骤删除：原时间采样两次比较的分支由原子采样和明确挂起补偿替代；固定换算要求改为编译期断言。未上板，不以主机寄存器替身证明真实 IRQ 延迟或硬件调度时序
