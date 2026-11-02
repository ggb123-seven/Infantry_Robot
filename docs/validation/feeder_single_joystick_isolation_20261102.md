# 拨弹单发模式隔离摇杆输入修复

日期：2026-11-02

## 问题

- `feeder_fire_mode = 1` 时，右拨杆下位可正常触发一次 45° 单发
- 单发完成后推动右摇杆，既有逻辑会取消单发状态并进入纯速度环
- `feeder_fire_mode` 仍保持为 1，因此观察值与实际采用的控制路径不一致

## 修复

- 单发模式在 `Feeder_Control` 入口将右摇杆进给倍率隔离为零
- 单发模式只响应右拨杆下位的触发边沿，起发与执行固定使用角度速度串级
- 切回连发模式时终止可能残留的单发执行状态
- 未修改 `config.c`、结构体布局、电机参数、BSP、设备层或 CubeMX 配置

本记录替代旧验证文档中“单发档推动摇杆进入连续进给”的行为约定

## 验证

- 临时主机回归使用真实 `feeder.c`、PID、滤波和数学组件，覆盖模式 1、非零摇杆倍率、拨杆触发、45° 到位、
  保持拨杆下位再次推动摇杆的完整序列
- 回归结果：单发到位后 `state == FEEDER_RELAX`、`single_active == false`、`Feeder_DumpOutput() == 0 A`
- MinGW 严格编译：`-std=c11 -Wall -Wextra -Werror`，测试通过
- ARM 严格语法检查：`feeder.c`、`motor_trigger.c` 使用 `-Wall -Wextra -Werror -fsyntax-only`，通过
- Debug 构建与链接通过：RAM 47200 B / 128 KB，FLASH 127616 B / 1 MB
- ARCH 门禁通过，0 violations；HW 与 SPEC 门禁通过
- 四个修改文件的 120 列、Allman、行尾空白和 `git diff --check` 扫描通过
- 临时测试源码、类型桩和可执行文件已清理

## 防御性检查自查

- 本次未新增判空、`isfinite`、范围检查或保护性 `return`
- 本次未删除既有信任边界检查

## 未验证

- 未烧录固件，未在真实 M2006、拨弹盘和弹丸负载下复测摇杆隔离
- 单发 4° 到位窗口、0.3 s 超时与连续操作手感仍以实机结果为准
