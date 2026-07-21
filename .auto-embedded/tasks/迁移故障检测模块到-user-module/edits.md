# 编辑清单

| 文件 | 改动 | 验证标准 | 结果 | commit |
|---|---|---|---|---|
| User/module/fault_detect.h、User/module/fault_detect.c | 从 `User/task` 迁移到 `User/module`，源码 include 改为 `module/fault_detect.h`。 | 不存在 `User/task/fault_detect.*`；构建通过。 | 通过：`Test-Path User/task/fault_detect.*` 为 false，`Test-Path User/module/fault_detect.*` 为 true，CMake Debug 构建通过。 |  |
| User/task/motor_chassis.c、User/task/ozone_debug.h | 将 include 从 `task/fault_detect.h` 改为 `module/fault_detect.h`。 | `rg task/fault_detect` 无残留；task 层只保留调用编排。 | 通过：`rg fault_detect` 仅显示 `module/fault_detect` 引用，无 `task/fault_detect` 残留。 |  |
| CMakeLists.txt | 源文件清单从 `User/task/fault_detect.c` 改为 `User/module/fault_detect.c`。 | CMake Debug 构建可重新编译并链接。 | 通过：`python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source F:\RM\Infantry_Robot --preset Debug -j 4` 成功生成 ELF。 |  |
