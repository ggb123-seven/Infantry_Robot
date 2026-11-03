# 右拨杆下位启动摩擦轮

日期：2026-11-03

## 行为变更

- `Task_motor_trigger` 以已校验的 `feeder_command.mode == FEEDER_CMD_RUN` 作为右拨杆下位信号，摩擦轮不再依赖拨弹模块是否因摇杆输入进入运行态
- 右拨杆上位、中位、非法档位或遥控失效时，`Cmd_ParseFeeder` 不会给出运行档，摩擦轮命令保持失能
- 上一周期输出发布失败时仍禁止提交拨弹与摩擦轮运行命令

## 验证

- `cmake --build build/Debug --parallel`：通过，修改任务编译并完成 ELF 链接
- `python .auto-embedded/scripts/check.py`：ARCH、HW、SPEC 全部通过
- `powershell -ExecutionPolicy Bypass -File .auto-embedded/scripts/arch-check.ps1`：0 violations；未找到硬件资源表，硬件锁专项检测按脚本提示跳过
- 本次未烧录固件或运行真实摩擦轮，未验证实物方向、启动电流和转速响应
