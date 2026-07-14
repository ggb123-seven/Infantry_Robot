# 编辑清单

| 文件 | 改动 | 验证标准 | 结果 | commit |
|---|---|---|---|---|
| `User/task/app_usb_debug.c` | 新增低优先级 USB CDC 调试任务；任务启动时初始化 USB，100 Hz 读取电机反馈快照并发送 JustFloat 帧；USB 忙时丢帧并计数 | 不阻塞，发送缓冲区生命周期正确，断开主机不影响控制任务 | 待执行 | - |
| `User/task/app_usb_debug.h` | 定义调试任务 API、运行统计和帧字段约定 | 公共 API 简洁，注释为简体中文 Doxygen 格式 | 待执行 | - |
| `User/task/user_task.c` | 新增 USB 调试线程属性 | 低于底盘控制任务优先级，栈大小满足构建与实测水位 | 待执行 | - |
| `User/task/user_task.h` | 登记 USB 调试线程句柄、频率和运行统计 | 结构体字段与任务实现一致 | 待执行 | - |
| `User/task/init.c` | 创建 USB 调试任务 | 任务创建成功且不改变现有电机任务启动顺序 | 待执行 | - |
| `User/task/config.yaml` | 登记 USB 调试任务配置 | 配置与线程频率、栈大小一致 | 待执行 | - |
| `CMakeLists.txt` | 纳入 `User/task/app_usb_debug.c` | 全量 Debug 构建成功，ELF 保留 `MX_USB_DEVICE_Init` 和 USB 调试任务符号 | 待执行 | - |

## 生成代码约束

- 不修改 `Src/freertos.c` 中 CubeMX 生成的弱 `red_led_task()`；应用侧任务自行完成唯一一次 USB 初始化。
- 不修改 `Src/usbd_cdc_if.c`、`Middlewares/ST` 或 HAL PCD/LL USB 文件。
- Keil 工程缺失 RVDS FreeRTOS 端口的问题单独处理，本任务先以 CMake/GCC 构建链为准。
