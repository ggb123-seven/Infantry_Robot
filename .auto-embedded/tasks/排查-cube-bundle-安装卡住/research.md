# 研究发现

| 关键词 | 来源 | 摘要 | 可信度 | 状态 |
|---|---|---|---|---|
| Cube bundle 清单 | `.settings/bundles.store.json`、`.settings/bundles-lock.store.json` | 已解析 CMake 4.3.1、Ninja 1.13.2、GNU Tools for STM32 14.3.1、ST Arm Clangd 21.1.0、J-Link GDB Server 9.24.0。 | 高 | 已验证 |
| 工具链实际路径 | `build/Debug/CMakeFiles/4.3.1/CMakeCCompiler.cmake` | C 编译器来自 `%LOCALAPPDATA%/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin/arm-none-eabi-gcc.exe`。 | 高 | 已验证 |
| Debug 构建 | `py .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug -j 8` | 配置和构建成功，产物为 `build/Debug/Infantry_Robot.elf`，大小约 1054.5 KB。 | 高 | 已验证 |
| 项目门禁 | `py .auto-embedded/scripts/check.py` | HW 与 SPEC 通过；ARCH 因 `arch-check.ps1` 第 279 行附近的 PowerShell 解析错误未能执行，不是本次源码编译错误。 | 高 | 工具故障 |
