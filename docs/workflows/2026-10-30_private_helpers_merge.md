# 输入整形与角度展开合回模块

## 授权与范围

- 用户确认将两个小组件合回其唯一使用模块，不再独立暴露源码文件
- 用户确认仅 `User/module/gimbal.c` 使用 900 行上限，其余 C 文件保持 800 行
- 基线为 `8b8d269`；本轮不改变结构体名称、字段及布局，不修改 config.c、驱动或 CubeMX 配置

## 结果

- input_shaping 两个函数合入 cmd.c，改为私有 Cmd_NormalizeChannel 与 Cmd_SmoothRate
- angle_unwrap 三个函数合入 gimbal.c，改为私有 Gimbal_WrapAngle、Gimbal_ResetAngleUnwrap 与 Gimbal_UpdateAngleUnwrap
- AngleUnwrap_t 原有定义迁入 gimbal.h，字段、类型和顺序保持一致，集中说明同步为云台使用的度及 360 度周期
- 删除四个独立源码及头文件，同步移除 CMake 和 Keil 登记
- 云台文件现为 878 行，PowerShell 与 Bash 门禁按精确路径应用 900 行例外，架构规范同步记录

## 验证证据

- `python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug` 配置与构建通过
- 从 build/Debug/compile_commands.json 提取实际 ARM 命令，移除 -o、-c、-fstack-usage，追加 -Wall -Wextra -Werror -fsyntax-only，cmd.c 与 gimbal.c 均通过
- 标准输入执行 Python 对比脚本，23 个原有模块函数在调用名替换后函数体一致；5 个迁入函数只改名称、static 和下节明确删除的检查；AngleUnwrap_t 定义逐字段一致
- 临时主机 GCC C11 程序以 -O0 -Wall -Wextra -Werror 编译，对比基线真实函数与迁入真实函数：全部 65536 个 int16_t 通道值、100000 周期速度整形、100000 周期环绕及连续角展开逐浮点字节一致
- 额外检查正负零、极小负值、正负整圈、跨零点、半圈歧义及非有限累计结果；半圈与非有限结果均清除参考并返回失败
- `python .auto-embedded/scripts/check.py` 的 ARCH/HW/SPEC 全部通过；Git Bash 执行 arch-check.sh --all 通过
- 两套门禁使用临时文件验证边界：gimbal.c 900 行、其他 C 文件 800 行通过；gimbal.c 901 行或其他 C 文件 801 行准确报错
- 三个修改源码文件完成 Allman、控制块花括号、120 列、尾空白和编码扫描；人工复核目的注释、Doxygen、状态与字段集中说明，未引入跨周期业务状态分支
- Keil XML 解析、全部已登记 User 源文件路径存在性、删除文件与登记无残留、git diff --check 均通过
- main.c、RTOS/HAL 对象及返回值处理、ISR 共享状态、临界区、配置快照与安全输出路径未改变；私有函数保留在业务模块内，不引入反向依赖

## 防御性检查自查

- 新增判空、范围检查、isfinite 和保护性 return 均为零
- 删除原 Reset 的实例判空及 return：现为私有调用，所有调用方传入实例内 unwrap 地址
- 删除原 Update 的实例、输出指针判空及 return：唯一调用方传入实例成员和局部输出地址，调用方保证非空
- 删除原 Update 的周期与单圈输入合法性检查及失败复位：周期固定为 360，编码器在 Gimbal_UpdateAxisFeedback 信任边界已检查范围，零点来自最近有效配置快照，输入经 Gimbal_WrapAngle 处理
- 保留最短路径分支、首次参考分支与半圈歧义判断，属于角度展开计算本身；保留 next_angle 的 isfinite，属于新增累计计算链出口，上游无法保证加法结果有限
- 输入整形死区、零速换向和收敛条件原样迁入，属于计算规则，未增加下游重复校验

## 验证边界与清理

本轮未执行 Keil 编译、烧录或实物测试，主机数值回归不替代整车验收
临时回归源码、可执行文件和门禁夹具通过 TemporaryDirectory 在验证后清理；没有删除正式测试资产
原有底盘及云台实物方向、失联、任务耗时与栈余量待验收事项继续按 agent.md 执行
