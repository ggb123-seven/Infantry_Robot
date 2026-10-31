# CRC8 / CRC16 移植评审

日期：2026-10-31

> 本文前半部分保留授权前评审；用户随后明确授权移植，最终实施与先前建议的差异以文末“授权后实施结果”为准，数学库依赖按用户要求保留

## 当前结论与授权边界

- 已只读对比 `F:/RM/未命名文件夹/User/component/crc8.[ch]`、`crc16.[ch]` 及参考工程裁判系统、自瞄调用点
- 当前工程 `User` 中没有 CRC8 / CRC16 实现或调用，CMake、Keil 也未登记这两个组件；本次是增加可复用能力，不能声称已经删除当前工程的大量重复函数
- 建议保留 `CRC8_Calc`、`CRC8_Verify`、`CRC16_Calc`、`CRC16_Verify` 四个公共函数，将参考实现仅由 Calc 使用的 `CRC16_Byte` 合并到计算循环中
- 根目录 `AGENTS.md` 限制默认源码修改范围为 `User/task`；新增 `User/component/crc8.[ch]`、`crc16.[ch]` 需要用户明确同意，当前未修改生产源码、CMake 或 Keil
- 建议授权范围为上述四个新文件及 `CMakeLists.txt`、`MDK-ARM/Infantry_Robot.uvprojx` 的源码登记；不涉及结构体设计、设备协议接入、参数配置和 CubeMX

## 移植方案

1. 保留两张只读查找表及带初值的 Calc 接口，支持不同初值与分段计算
2. `crc8.h`、`crc16.h` 仅包含 `stdbool.h`、`stddef.h`、`stdint.h`；删除参考 `crc16.h` 对 `user_math.h` 的依赖，避免继续引入参考工程的 STM32 头文件
3. 校验按完整帧传入，Calc 长度不含尾部 CRC；协议长度、SOF、命令字段和接收缓冲区容量由设备解析入口检查，CRC 组件不认识具体设备
4. 不增加 Append、Init、Reset、逐字节包装和实例结构体；发送端直接将 Calc 结果写入已有协议字段，字节流形式发送时由协议层按字节序序列化
5. CRC16 尾部按低字节在前显式组合，替代 `uint8_t *` 到 `uint16_t *` 的强转，消除缓冲区对齐、宿主字节序及类型访问问题
6. 保持参考边界语义：CRC8_Verify 的完整帧至少 2 字节，CRC16_Verify 至少 2 字节；后者保留仅含两字节校验值的空负载帧行为
7. 移植时统一 Allman、120 列和简体中文 Doxygen 注释，不直接保留参考实现的英文循环注释和单行控制块

公共接口保持如下：

```c
uint8_t CRC8_Calc(const uint8_t *buf, size_t len, uint8_t crc);
bool CRC8_Verify(const uint8_t *buf, size_t len);
uint16_t CRC16_Calc(const uint8_t *buf, size_t len, uint16_t crc);
bool CRC16_Verify(const uint8_t *buf, size_t len);
```

候选实现的 CRC16 尾部读取表达式如下：

```c
return expected == (uint16_t)((uint16_t)buf[len - 2] | ((uint16_t)buf[len - 1] << 8));
```

这一步减少参考实现的一个私有函数，不合并 CRC8 与 CRC16 为运行时通用引擎，避免为不同位宽引入分派与配置接口

## 实际验证

### 工程基线

- `python .auto-embedded/scripts/check.py`：ARCH、HW、SPEC 全部 PASS
- `python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --list-presets`：列出 Debug / Release
- `python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug`：配置和构建成功，产物 `build/Debug/Infantry_Robot.elf`
- 基线构建未包含候选 CRC，不表示移植后的 ARM 固件编译已经通过

### 参考代码主机对照

通过 Python 标准输入提取实际参考 C 文件，去除其 include，由标准 C 头文件提供类型，再将真实查表实现与独立逐位参考实现一起送入 GCC 标准输入编译

编译参数：`gcc -x c - -std=c11 -O2 -Wall -Wextra -Wcast-align=strict -o <临时程序>`

- CRC8 对照 67588 次、CRC16 对照 67588 次、分段对照 4104 次、有效帧和损坏帧校验 8208 次、短帧边界 4 次，全部通过
- `123456789` 在参考初值 `0xFF` / `0xFFFF` 下分别为 `0x0B` / `0x6F91`
- 编译器在原 `CRC16_Verify` 的 `uint16_t *` 强转处报告 `cast increases required alignment of target type`

### 精简候选实现主机对照

在内存中合并 `CRC16_Byte`，改用字节组合读取 CRC16，未向项目写入候选生产源码

编译参数：`gcc -x c - -std=c11 -O2 -Wall -Wextra -Werror -Wcast-align=strict -fstrict-aliasing -o <临时程序>`

输出：`PASS calc=135176 splits=4104 frames=4104 tail_bit_flips=32832 short=4`

- 覆盖所有 CRC8 单字节初值与字节组合，以及全部 65536 个 CRC16 初值的单字节样本
- 覆盖 0～512 字节负载、起始偏移 0～3、奇偶长度、分段计算与 CRC16 尾部逐位破坏
- 编译和运行返回码均为 0，严格警告检查无诊断
- 原实现另有 CRC8 损坏帧测试通过；候选仅修改 CRC16，CRC8 函数体保持参考实现
- 临时可执行文件均由 TemporaryDirectory 在测试结束时清理，未保存临时测试源码

## 三层人工复核与校验自查

- 验证门：工程基线和候选主机测试已有上述证据；正式移植后的目标编译、协议调用联调、上板收发均未执行
- 硬件合规：门禁通过；候选为纯 C 只读查表计算，不访问寄存器、外设、引脚、DMA 或中断资源
- 代码质量：候选不改 main.c，不新增共享可写状态、任务对象或临界区；数学库与 STM32 依赖可直接移除；正式文件尚待授权后按项目风格整理
- 新增生产判空、isfinite、范围检查和保护性 return：无，本轮未修改生产源码
- 计划保留的长度检查位于 Verify 的外部字节流入口，在减去尾部长度前执行；Calc 不重复检查，调用方保证缓冲区和长度有效
- 计划新增判空与浮点检查：无；删除的生产检查：无

## 后续实施验证

获得上述目录授权后，新增四个独立组件文件并登记两套构建，执行真实组件的主机回归、目标 ARM 严格编译、Debug 构建及 ARCH/HW/SPEC，再记录证据并仅提交本次相关文件

## 授权后实施结果

### 已落实的范围与差异

- 用户确认移植 CRC 文件，要求 CRC8 / CRC16 分开，并在函数内说明额外改动
- 新增 `User/component/crc8.c/.h` 与 `crc16.c/.h`；两张查找表各 256 项，逐项与参考工程比对完全一致
- 按用户要求保留 `crc16.h` 对 `user_math.h` 的依赖，不修改数学库文件；此前移除该依赖的建议未实施
- `CRC16_Calc` 内用简体中文说明将原 `CRC16_Byte` 合入计算循环，四个公共接口保持原名与签名
- `CRC16_Verify` 内用简体中文说明按低字节在前组合帧尾，替代 `uint16_t *` 强转；两种 CRC 均保留原初值、计算范围和最短帧行为
- 同步登记 CMake、Keil 和组件配置清单，CRC16 清单如实记录数学库头文件依赖
- 新增文件按 UTF-8、Allman 和 120 列整理，补齐简体中文 Doxygen；没有新增结构体、设备调用、配置参数或 CubeMX 变更
- 每个源码文件落盘后均重新读取根目录 AGENTS.md、agent.md，并逐文件执行行宽、花括号、行尾空白和注释末尾检查

### 最终验证证据

- `python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --list-presets`：Debug / Release 预设可用
- `python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug`：实际新增 CRC 源文件参与构建，配置与链接成功，产物 `build/Debug/Infantry_Robot.elf`
- 从 Debug compile_commands.json 逐项取得真实 ARM 编译器与工程参数，对两个 CRC 源码追加 `-Wall -Wextra -Werror -Wcast-align=strict -Wstrict-prototypes -fsyntax-only`：均返回 0，无警告
- 主机使用实际新增的两个 C 文件、真实数学库头文件，以及从参考文件提取并重命名的原始实现进行对照，另以独立逐位实现交叉验证；编译参数含 `-O2 -Wall -Wextra -Werror -Wcast-align=strict -Wstrict-prototypes -fstrict-aliasing`
- 主机输出：`PASS calculations=147474 segments=16400 valid_frames=16400 corrupt_frames=213184 short_frames=4 known_vectors=2`，运行返回 0
- 帧行为检查计数沿用测试输出 valid_frames 标签，包含 CRC8 零负载帧按原约束被拒绝的情况，不表示所有样本都被接受
- 覆盖 CRC8 全部初值与单字节组合、CRC16 全部初值的单字节样本，0～1024 字节负载、起始偏移 0～7、奇偶长度、分段计算、CRC 尾部每一位破坏及非空负载位破坏；另覆盖 65537 字节长度，确认未截断为 16 位长度
- 头文件分别通过 C11 / C++17 独立包含严格语法检查；Keil XML 解析、两个文件唯一登记及实际源路径检查通过
- `python .auto-embedded/scripts/check.py`：ARCH、HW、SPEC 全部 PASS；`git diff --check` 通过
- 完整命令和返回值记录于 `docs/validation/crc_port_20261031.json`；临时参考源码、对象文件和测试程序已由 TemporaryDirectory 清理

### 最终人工复核与校验自查

- 分层：组件不访问任务、设备、HAL 或 RTOS；CRC16 按用户选择保留同层数学库头文件依赖，未修改其函数或宏
- 并发与硬件：查找表为 static const，工作状态均为调用局部变量；无 ISR 共享可写状态、临界区、对象创建或硬件资源变更
- 行为边界：坏帧和过短帧只返回 false，不写输入缓冲区；组件不驱动执行器，不新增 DumpOutput 或业务状态机
- 判空：未新增；isfinite：未新增；范围检查与保护性 return：CRC8_Verify 的 len < 2U、CRC16_Verify 的 len < sizeof(uint16_t)，各自返回 false
- 上述两项均为继承参考实现的外部字节流信任边界，实际触发场景是截断帧；当前尚无上游调用者，不存在上游重复检查，Calc 不重复检查
- 删除的重复校验：无；移除的私有包装：CRC16_Byte，其更新表达式已合入 CRC16_Calc 并在函数内说明
- 未执行 Keil 编译、烧录及真实协议收发；本轮完成组件移植和软件验证，不表示设备协议已接入
