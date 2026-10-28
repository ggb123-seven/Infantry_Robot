# 命令层拆分与云台模块精简验证记录

日期：2026-10-28

实施依据：[方案](2026-10-28_cmd_gimbal_refactor_plan.md)

## 当前进度

- 步骤 1 已完成：公共输入整形组件、任务调用迁移、CMake 与 Keil 源文件登记
- 步骤 2 已完成；用户已确认命令上下文、单轴命令、内部设定值及配置版本字段，含 rejected_version
- 用户选择正弦测试留在原任务并使用编译开关隔离
- 当前代码以 `8d49637` 为起点，保留此前默认配置接口精简；行为对比以 `f6801cf` 为基准
- 未修改 INS、governor、底层驱动、生成代码或 `.ioc`，未推送、未烧录

## 步骤 1：公共输入整形

新增 `User/component/input_shaping.c/.h`，提供两个纯计算接口：

- `InputShaping_NormalizeChannel`：接收满量程和死区，替换底盘两个私有函数与云台一个私有函数
- `InputShaping_SmoothRate`：原样迁移云台速度整形，平滑时间与收敛阈值由调用方传入

满量程 660、5% 死区、满杆 6 rad/s、加减速 60000 度每平方秒、4 ms 平滑和 0.1 度每秒收敛阈值不变

新接口要求调用方提供合法参数，两个现有调用点继续使用原常量和原周期校验；未新增锁、等待或 RTOS 对象

### 构建与检查

```powershell
python .auto-embedded/scripts/check.py
python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug
git diff --check
```

- ARCH、HW、SPEC 全部 PASS
- CMake Debug 构建通过，产物 `build/Debug/Infantry_Robot.elf`
- 从 `build/Debug/compile_commands.json` 取得实际 ARM 编译命令，移除 `-o`、`-c`，追加 `-Wall -Wextra -Werror -fsyntax-only`，检查 `input_shaping.c`、`motor_chassis.c`、`motor_gimbal.c`，全部通过
- 三个源文件及新增头文件的 Allman 花括号、120 列与行尾空白扫描通过；人工复核阶段注释、参数契约、分层、原有并发与安全输出路径
- Keil 工程 XML 可解析，新源文件仅登记一次且路径存在；未执行 Keil 编译

### 主机行为对比

临时测试命令：`python build/cmd_refactor_step1/run.py`

编译器：`E:\movie\vscode+\c_c++\mingw64\bin\gcc.exe`，使用 `-std=gnu11 -O0 -fgnu89-inline`

测试直接从 `git show f6801cf:<文件>` 提取旧私有函数，编译当前公共组件及实际数学、滤波、PID 和角度展开组件：

- 全部 65536 个 `int16_t` 通道计数分别与旧底盘、旧云台归一化实现比较，结果精确相等，包含方案要求的 [-700, 700] 和 ±33 死区边界
- 200000 次固定种子随机请求与周期序列，加 10000 次换向、松杆及不对称加减速序列，逐次结果精确相等，最后归零

控制链对比独立编译旧、新两份 `gimbal.c` 与 `gimbal_governor.c`，提取各版本任务的实际遥控和正弦函数；邮箱读取与时间使用替身，发布失败处理取自实际任务代码，配置取当前未改动的云台默认值

18 组场景每组 1600 个周期，共 28800 个周期，覆盖：

- 上电上位不使能、中位确认、满杆与换向
- pitch 机械限位附近持续推杆及回拉
- 遥控离线、100000/100001 微秒帧龄边界、来源时间超前、缓存过期
- yaw 或 pitch 电机超时与恢复、pitch 非有限反馈、IMU 无效反馈
- 空邮箱、读取错误、空句柄、非法周期、下位停机及重新中位确认
- 正弦测试启停与前馈开关、ANGLE/SPEED 切换、输出发布失败
- 残差触发制动、HOLD、RESTORING、GUARDED、MONITOR，实际观测 governor 全部 6 个状态，覆盖掩码为 `0x3f`

额外断言验证：正常使能后两轴实际 RUNNING 且产生非零控制电流；yaw 超时时 yaw OFFLINE、pitch 仍 RUNNING，避免只比较停机轨迹

逐周期比较两轴输出电流、目标角、目标速度、轴状态、IMU 状态和 governor 状态，两份轨迹完全一致，无需浮点容差

轨迹 SHA256：`eae87ac1a41f95e912a4945ae90899ba34ec1a7cb4eadfeb259f61d7e01a15f0`

该摘要对应时间戳和覆盖断言修正后的最终有效测试轨迹；临时测试源码、程序及轨迹在记录后清理，不作为长期回归资产

### 验证边界与后续

本步确认计算行为保持一致，未测量 MCU 周期耗时、栈峰值或整车遥控手感，不能据此宣称 CPU 性能提升或整车验收完成

下一步需确认 `Cmd_Gimbal_t`、公开命令和内部设定值的名称及字段、正弦测试隔离方式，以及配置版本字段和非法配置拒绝策略，再实施命令层

## 步骤 2：命令层

- 新增 `module/cmd.c/.h`，将底盘遥控映射、云台使能状态机和速度整形移入模块；复用现有任务及邮箱
- 云台暂时保留任务内的目标角累加，以及失联、正弦接管时清除对应轴平滑的旧行为，供本步骤独立对比
- `Cmd_ParseGimbal` 返回快照与周期有效性，任务据此清除缓存标志，保留旧实现“周期异常后即使邮箱为空也不得沿用旧使能”的约束
- `Gimbal_HasLatchedFault` 为只读内联查询，任一轴 FAULT 或惯性源 FAULT 均要求命令 RELAX，OFFLINE 不触发人工复位
- 验证：ARCH/HW/SPEC、Debug 构建、实际 ARM 编译器 `-Wall -Wextra -Werror -fsyntax-only`、格式和 `git diff --check` 全部通过
- 主机命令：`python build/cmd_refactor/test.py 2`；相对 f6801cf 的 18 组共 28800 个周期，电流、目标和状态精确相等，governor 状态覆盖掩码 0x3f
- 底盘独立对比原任务解析函数：[-700,700]、6 个拨杆取值及 7 种缓存/在线/帧龄条件，共 58842 组，命令各字段精确相等；空指针保护通过
- 未改硬件资源、任务优先级和队列拓扑，新增状态由云台任务独占，无新增共享临界区；尚未上板

## 步骤 3：模块持有目标角

- 采用用户确认的单轴 `source/rate_rad_s/angle_deg` 命令及轴内 `setpoint`；`Gimbal_Control` 输入改为 const，任务删除目标角累加和轴状态清平滑逻辑
- 首次遥控使能仍将平滑速度置零，随后模块按“解析、姿态捕获、governor、控制”执行；增量以上周期实际限幅目标为基准
- 正弦留在原文件，以 yaw ABSOLUTE 来源提供目标，退出恢复 INCREMENT；`MOTOR_GIMBAL_ENABLE_SINE_TEST=0` 可编译移除整个实验
- 本步骤曾拆出反馈实现和私有头文件，后按用户要求合回 gimbal.c，当前结果及验证见下节
- governor 仅将输入命令改为模式和模块内部设定值，未改状态机、阈值、包络；公共接口完整说明集中于头文件，避免两份契约重复维护
- `python build/cmd_refactor/test.py 3`：20 组共 32000 周期，与 f6801cf 比较，状态全部相同，全部 6 个 governor 状态均覆盖
- 普通场景数值满足 `1e-6 × max(1, |旧值|, |新值|)`，误差来自公开命令改为 rad/s 后的度/弧度浮点换算；不改变旧满杆参数
- 超出上述容差的差异仅在方案允许的 yaw 超时恢复、pitch 超时恢复及正弦交接场景，各为 177、130、2117 个字段样本；对应命令层不再因单轴失联或正弦接管清空平滑历史
- 增加绝对角输入、非法来源和逐周期命令不回写断言，均通过；编译关闭正弦的实际 ARM 对象经 nm 确认不含测试函数和变量
- ARCH/HW/SPEC、Debug 构建、ARM 严格语法检查与格式检查通过；硬件和通信拓扑未改，未上板实测

## 按用户要求合回云台文件

- 将反馈实现和私有头文件内容合回 `User/module/gimbal.c`，删除两个文件，移除 CMake 与 Keil 源文件登记
- `Gimbal_ClearFeedback`、`Gimbal_StopAxis` 恢复为 static，公共接口保持不变；去除重复说明后 gimbal.c 为 800 行，三个源码文件合计减少 26 行
- 对照合并前提交 `124ea6e`，逐函数核对全部 19 个函数体的文本完全一致，未改计算、状态机、保护阈值、INS、硬件资源或 RTOS 拓扑
- `python .auto-embedded/scripts/check.py`：ARCH/HW/SPEC 全 PASS
- `python .auto-embedded/tools/build-cmake/scripts/cmake_builder.py --source . --preset Debug`：构建通过
- `python build/cmd_refactor/test.py 4 124ea6e`：参数 4 选择精确比较，不表示已实施计划步骤 4；20 组共 32000 周期的电流、目标、轴及 governor 状态逐项精确一致，governor 全部 6 个状态均覆盖
- 两份最终轨迹 SHA256 均为 `8a96a66508042d60799bb32fcb2ef120844751295864f33186e0ab9d982b3aea`
- 实际 ARM 编译器 `-Wall -Wextra -Werror -fsyntax-only`、正弦编译关闭检查、Allman/120 列/行尾空白检查、`git diff --check` 通过
- Keil XML 与工程路径检查通过，未执行 Keil 编译；ELF 确认两个内部函数为局部符号，源码及工程中无已删除文件引用
- 临时测试源码、程序与轨迹已在记录后清理；未烧录，未进行板上控制或耗时测量
- 本次仅合并文件，后续步骤尚未实施，不再按每个控制阶段新增函数
