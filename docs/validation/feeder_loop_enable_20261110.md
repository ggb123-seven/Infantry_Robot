# 拨弹角度环与速度环使能接口

## 实现行为

- `feeder_angle_loop_enabled` 复位默认 `true`，`feeder_speed_loop_enabled` 复位默认 `false`
- 单发与五连发默认只运行角度环，位置误差输入为输出轴弧度，PID 输出直接作为电流请求 A
- 独立角度环输出限幅为 `min(position_pid.out_limit, max_current_a)`；RAM 配置副本不被改写，PID 长期引用模块的持久周期快照
- 同时开启角度环和速度环时，定量进给恢复角度 rad/s 到速度 rpm 到电流 A 的原串级
- 连续进给要求速度环开启；角度环关闭、速度环开启时，单发和五连发没有速度倍率，因此撤驱
- 两环都关闭时撤驱。环路结构变化清空控制历史、取消当前定量组并等待开火请求释放
- 遥控安全档、离线、故障和模块输出闸门保持原行为；未修改 `config.c`，未增加公开结构体字段

## 软件验证

- `cmake --build --preset Debug`：通过，RAM 50416 B，FLASH 130304 B
- `.auto-embedded/scripts/check.py`：ARCH、HW、SPEC 全部通过
- ARM `-fsyntax-only -Wextra -Werror`：`User/module/feeder.c` 和 `User/task/motor_trigger.c` 通过
- 真实 feeder、config、PID、滤波与数学组件的主机回归：2989 条断言通过
- 主机用例覆盖默认开关、安培直出、速度 PID 与速度前馈不参与、动态电流限幅、PID 积分跨周期保持、角度微分、串级恢复、四种开关组合、单发与五连发、模式切换取消、1 秒超时、故障解除、失联/空命令/非法配置/周期、反馈时间戳回绕和安全撤驱
- `User/module/config.c` SHA256 与实现前一致：`48A26D9EE250A2F3012D0D1CC21E927F7955264FAB6013C6653CC3EA5CD4DA7D`
- 源码格式扫描与 `git diff --check` 通过

## 验证边界

本次没有烧录固件或写入目标 RAM。角度环的电流请求、机构实际出力、齿位跟随效果、振动和到位超时仍需在安全架空状态下上板验证。默认角度 PID 参数原来用于 rad/s，启用单角度模式后输出单位变为 A，现场使用前须按新单位校准参数。

本次新增的配置范围检查位于 Feeder Init/Control 的配置快照入口，只在串级模式要求位置 PID 输出上限不超过速度上限换算值；该边界区分了串级 rad/s 与独立角度环 A 的量纲。角度环输出限幅应用于已校验的持久快照，没有在下游重复检查。未新增判空、`isfinite` 或保护性 `return`；原有检查保持原位置。
