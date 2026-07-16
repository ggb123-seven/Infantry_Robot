# CMake 工具链发现与自动门禁修复反思

**日期**: 2026-07-16
**提交类型**: bugfix
**会话时长**: 90 分钟
**修改文件数**: 19 个文件

## 1. 概述

修复普通终端无法自动找到 STM32Cube Bundle GNU Arm 工具链的问题，并恢复 auto-embedded 的 ARCH/HW/SPEC 自动门禁，使检查结果反映真实工程约束而不是工具兼容性或规则误报。

## 2. 修改内容

### 修改的文件

- `cmake/gcc-arm-none-eabi.cmake`：自动发现同一目录中的 GNU Arm 编译、链接和产物工具。
- `.auto-embedded/scripts/arch-check.ps1`：修复 UTF-8、PowerShell 5、相对路径和构建目录扫描问题。
- `.auto-embedded/scripts/arch-check.sh`、`aemb_core.py`、`check.py`：同步 ARCH-2 与硬件资源判重规则。
- `.auto-embedded/spec`、`.auto-embedded/refs`、`.agents/skills`：同步长期规则和操作说明。
- 当前任务日志：补充构建、产物和门禁验证证据。

### 主要变更

- CMake 依次查找当前环境、`ARM_GNU_TOOLCHAIN_ROOT`、`CUBE_BUNDLE_PATH` 和本机 STM32Cube Bundle。
- `gcc`、`g++`、`objcopy` 与 `size` 限定在同一个工具链目录，避免版本混用。
- ARCH-2 不统计 CubeMX/HAL/RTOS 生成的初始化与调度调用，只限制自定义顶层调用。
- HW 门禁按 pin、DMA stream、IRQn 和 timer 标识判重，不再把相同中断优先级视为资源冲突。

## 3. 遇到的错误

1. 普通 PowerShell 中 `arm-none-eabi-gcc` 不在 `PATH`，CMake 无法完成首次配置。
2. Windows PowerShell 5 按系统代码页读取无 BOM 的 UTF-8 脚本，中文损坏后触发级联语法错误。
3. Windows PowerShell 5 使用的 .NET Framework 不提供 `Path.GetRelativePath()`。
4. ARCH 扫描包含 `build/` 中的编译器识别文件和历史构建快照，产生假违规。
5. HW 检查错误要求每个 IRQ 优先级组合唯一。
6. ARCH-2 把生成的外设初始化数量当作架构复杂度，导致 CubeMX 配置变化后阈值失效。

## 4. 根本原因分析

### 为什么会出现这些错误?

- 工具链文件依赖外部环境注入可执行文件路径，没有项目内的后备发现逻辑。
- PowerShell 脚本按 PowerShell 7 编写和验证，但实际门禁优先调用 Windows PowerShell 5。
- 硬件规则混淆了“资源标识重复”和“优先级相同”两个概念。
- ARCH-2 使用固定总调用数近似判断入口复杂度，没有区分生成代码和自定义业务调用。

### 是什么导致编写时出现这些错误?

- 只验证了已有构建目录，没有先验证无 `PATH` 的全新配置。
- 缺少 Windows PowerShell 5 与 Git Bash 的双端语法检查。
- 将通用阈值直接套用于 CubeMX 生成入口，没有结合生成代码的稳定命名规则。

## 5. 调试过程

### 调查步骤

1. 在普通终端复现编译器不可见，并定位 STM32Cube Bundle 14.3.1 的实际安装目录。
2. 删除 `build/Debug`，验证工具链自动发现能完成从零配置和构建。
3. 用 PowerShell AST 解析器定位 ARCH 脚本语法与编码问题。
4. 恢复脚本执行后，根据真实输出继续定位相对路径 API 和 `build/` 误扫描。
5. 为 HW 规则验证“同优先级不同 IRQ 放行、重复 IRQn 拦截”。
6. 将 ARCH-2 改为只统计自定义顶层调用，并同步 PowerShell、Bash 和 spec。

### 迭代过程

- 第一次只补工具链 `PATH`，构建成功但终端重开后会失效。
- 第二次改为 CMake 自动发现，干净构建通过。
- ARCH 脚本依次暴露正则转义、UTF-8 BOM、旧版 .NET API 和生成目录扫描四层问题。
- 固定把 ARCH-2 上限改为 7 被否决，最终改为按调用语义分类。

### 耗时统计

- 调查: 20 分钟
- 实现: 45 分钟
- 测试: 25 分钟

## 6. 经验总结

### 核心洞察

- 嵌入式构建必须从空构建目录验证，已有缓存会掩盖工具链发现问题。
- 自动门禁本身也需要跨运行时兼容测试，否则工具故障会伪装成工程违规。
- NVIC 优先级相同是调度策略，不是 IRQ 资源冲突。
- 生成初始化调用数量由硬件配置决定，不能作为业务复杂度的固定代理指标。

### 预防策略

- 提交前至少执行一次无手工 `PATH` 的 Debug 干净构建。
- PowerShell 兼容脚本保存为 UTF-8 BOM，并限制使用 Windows PowerShell 5 可用 API。
- 资源锁只对唯一资源标识做机械判重，优先级合理性由硬件人工审查。
- 架构门禁识别生成代码模式，只约束开发者能够控制的业务调用。

### 识别的最佳实践

- 同一工具链的所有程序从同一 `bin` 目录解析。
- PowerShell 与 Bash 实现同步修改并分别做语法检查。
- 工具规则变更同步更新 spec、refs、skills 和任务验证记录。

## 7. 知识提炼

### 可复用模式

- 环境优先、显式根目录次之、工具供应商 Bundle 兜底的工具链发现顺序。
- 资源标识判重与配置策略审查分离。
- 生成启动编排白名单与自定义调用预算分离。

### 应避免的反模式

- 依赖 IDE 扩展临时注入的环境变量。
- 为通过门禁而修改真实中断优先级或简单放宽固定阈值。
- 扫描构建产物、快照和厂商源码后把结果归因于项目业务代码。

### 类似任务检查清单

- [ ] 确认编译器不在 `PATH` 时能否从零配置。
- [ ] 检查工具链程序是否来自同一版本目录。
- [ ] 分别验证 PowerShell 5、PowerShell 7 和 Git Bash 语法。
- [ ] 排除 `build/` 和厂商目录。
- [ ] 为允许场景和禁止场景各保留一个回归用例。
- [ ] 规则变化同步回写长期 spec。

## 8. 测试与验证

### 测试用例

- `python -B .auto-embedded/scripts/check.py --json`：ARCH、HW、SPEC 全部通过。
- 无 `arm-none-eabi-gcc` PATH 条件下删除 `build/Debug` 并重新构建：通过。
- HW 回归：不同 IRQ 使用相同优先级通过，重复 IRQn 被检出。
- ARCH-2 分类：生成初始化调用放行，自定义业务调用仍计数。
- PowerShell AST 与 Git Bash `bash -n`：语法通过。

### 验证步骤

1. `cmake_builder.py --source . --preset Debug --clean -j 8` 返回 0。
2. 生成 `build/Debug/Infantry_Robot.elf`，SHA-256 为 `1DB551696A611C476CF5C94F1FB50F5289761EB91F6EBA966A81B2D8F9C1EE09`。
3. 固件占用为 FLASH 49264 B、RAM 39064 B。
4. `ctest` 未发现测试项。
5. 未进行开发板烧录与实机运行验证。

## 9. 参考资料

- `docs/workflows/2026-07/12_reflection_bugfix_cubemx_build.md`
- `.auto-embedded/spec/architecture/index.md`
- `.auto-embedded/spec/hardware/index.md`
- `.auto-embedded/refs/arch-gate.md`

## 10. 指标

- 总错误数: 6
- 严重错误数: 0
- 调试迭代次数: 7
- 成功率: 100%
- 代码变动: +286 -64 行

---
**生成工具**: Codex
**技能**: commit-with-reflection v3.0
