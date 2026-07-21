# 编辑清单

| 文件 | 改动 | 验证标准 | 结果 | commit |
|---|---|---|---|---|
| User/task/fault_detect.h、User/task/fault_detect.c | 新增独立底盘故障检测快照、系统级故障位、逐电机故障位和汇总逻辑。 | CMake 编译通过；120 列和 Allman 风格检查通过。 | 通过：`cmake --build --preset Debug -j 4`、`rg` 格式扫描、ARCH/HW/SPEC 门禁通过。 |  |
| User/task/motor_chassis.c | 在底盘周期和初始化后调用故障检测，并发布 Ozone 快照。 | 底盘周期仍只编排消息和模块调用，不直接访问底层对象。 | 通过：底盘任务只调用公开快照接口，增量构建重新编译并链接成功。 |  |
| User/task/ozone_debug.h、User/task/ozone_debug.c | 增加 `g_fault_detect_monitor` 和故障检测发布接口。 | Ozone 全局快照字段完整可观察。 | 通过：`g_fault_detect_monitor` 发布完整快照，CMake 链接成功。 |  |
| CMakeLists.txt | 将 `User/task/fault_detect.c` 加入 CMake 源文件清单。 | CMake 构建能链接新增模块。 | 通过：`Infantry_Robot.elf` 生成成功。 |  |
