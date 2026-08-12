# 编辑清单

| 文件 | 改动 | 验证标准 | 结果 | commit |
|---|---|---|---|---|
| `User/component/mixer.h` | 从参考工程迁移并整理通用 Mixer 公开模式、状态和 API，保留多底盘切换能力 | 花括号、120 列、行尾空白、依赖方向和 `git diff --check` | 通过 | `380756e` |
| `User/component/mixer.c` | 移植已有模式，修复分支贯穿缺陷，按当前轮序补全 X 型全向轮，加入整体缩放和失败清零 | `gcc -std=c11 -Wall -Wextra -Werror` 编译和主机纯函数测试 | 通过，输出 `mixer_host_test: PASS` | `380756e` |
| `mixer_host_test.c` | 增加任务内主机测试，覆盖纯前进、纯横移、纯转动、组合缩放、不支持模式和 NaN | 测试程序退出码为 0 | 通过后已按用户要求清理 | 不提交 |
| `User/module/chassis.h` | 将模块输入改为归一化运动向量和转速尺度，初始化接口增加 Mixer 模式，快照增加运动学诊断字段 | 结构体集中说明、依赖方向、120 列、行尾空白和 `git diff --check` | 通过 | `9086b93` |
| `User/module/chassis.c` | 模块私有状态持有 Mixer，初始化时检查模式与四路输出兼容性，PID 前整组解算并限制统一尺度 | 主机编译与运行测试 | 通过，输出 `chassis_host_test: PASS` | `9086b93` |
| `chassis_host_test.c` | 覆盖不支持模式、X 型初始化、前进、横移限速、逆时针转动和 NaN 安全清零 | 测试程序退出码为 0 | 通过后已按用户要求清理 | 不提交 |
| `User/task/ozone_debug.h` | 将在线参数改为 `vx`、`vy`、`wz` 和 `scale_rpm`，增加运动学状态与四轮解算结果监视字段 | 结构体集中说明、旧直控字段扫描和格式检查 | 通过 | `6363284` |
| `User/task/ozone_debug.c` | 默认关闭使能，组装归一化运动输入并发布 Mixer 与四轮解算诊断 | ARM 编译对象成功生成 | 通过 | `6363284` |
| `User/task/motor_chassis.c` | 初始化时选择 `MIXER_OMNICROSS`，保留任务层编排职责 | ARM 编译对象成功生成且任务层无运动学公式 | 通过 | `6363284` |
| `AGENTS.md`、`agent.md` | 增加每个任务完成并留存证据后清理临时测试程序的规则 | 全部仓库内规则文件覆盖检查 | 通过 | `750960c` |
| `CMakeLists.txt` | 将 `User/component/mixer.c` 纳入 Component 源文件列表 | Debug 全量编译与链接 | 通过，FLASH 83192 B，RAM 41248 B | `0658906` |

## REVIEW 验证证据

- `python .auto-embedded/scripts/check.py`：ARCH、HW、SPEC 全部通过
- `.auto-embedded/scripts/arch-check.ps1`：0 violations，仅报告第三方库 mega-header 提示
- `cmake --build --preset Debug --clean-first`：77 个目标完整重建并链接成功
- 最终资源占用：FLASH 83192 B（7.93%），RAM 41248 B（31.47%）
- 逐文件格式检查与 `git diff --check 5967c8c..HEAD`：通过
- 任务目录临时测试源码和测试可执行文件：已清理
- 待上板：按 ID1 左前、ID2 左后、ID3 右后、ID4 右前架空验证 `+vx`、`+vy`、`+wz`
