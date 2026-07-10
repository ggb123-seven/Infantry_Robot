# RoboMaster 电机与电调 CAN 参考

> 适用于 M3508+C620、M2006+C610、GM6020 的选型、驱动开发和 CAN 调试。
> 原始手册保存在 `docs/datasheets/`；本文只蒸馏可复用的协议、电气参数和工程约束。

## 项目约束

- 本项目主控为 RoboMaster C 板（STM32F407IGH6）。
- 复用本文的 CAN 协议和设备参数时，CAN 外设、收发器、引脚、电源和终端电阻配置必须以 C 板原理图及本项目 CubeMX 配置为准。
- 不得照搬 RoboMaster A 板、其他开发板或 USB-CAN 示例中的引脚和供电配置。
- 以下设备均使用经典 CAN 标准帧，标称总线比特率为 `1 Mbps`。

## 快速选型

| 组合 | 额定电压 | 典型输出能力 | 机械参数 | 主要用途 |
|---|---:|---|---|---|
| M3508 + C620 | 24 V | C620 持续 20 A；组合最大连续转矩 3 N·m、额定转速 469 rpm | 减速比 `3591/187 ≈ 19.203`，365 g | 底盘、摩擦轮、较大负载机构 |
| M2006 + C610 | 24 V | C610 持续 10 A；组合最大连续转矩 1 N·m、额定转速 416 rpm | 减速比 36:1，90 g | 拨弹、供弹、小型执行机构 |
| GM6020 | 24 V | 最大连续转矩 1.2 N·m、额定电流 1.62 A | 直驱中空轴，约 468 g，13 bit 定位精度 | 云台、角度控制机构 |

来源：

- `docs/datasheets/RoboMaster_C620_ESC_User_Manual_V1.01.pdf`，PDF 第 35 页。
- `docs/datasheets/RoboMaster_M3508_User_Manual_V1.0.pdf`，PDF 第 12–13 页。
- `docs/datasheets/RoboMaster_C610_ESC_User_Manual.pdf`，PDF 第 9 页。
- `docs/datasheets/RoboMaster_M2006_P36_User_Manual.pdf`，PDF 第 8–9 页。
- `docs/datasheets/RoboMaster_GM6020_User_Manual.pdf`，PDF 第 12 页。

## C620 + M3508

### C620 控制帧

- 帧类型：标准数据帧，DLC=8。
- `0x200` 控制电调 ID 1–4，`0x1FF` 控制电调 ID 5–8。
- 每台电调占连续 2 字节，使用有符号 16 位数，高字节在前。

| 电调 ID | 控制 CAN ID | 字节 |
|---:|---:|---|
| 1 | `0x200` | D0–D1 |
| 2 | `0x200` | D2–D3 |
| 3 | `0x200` | D4–D5 |
| 4 | `0x200` | D6–D7 |
| 5 | `0x1FF` | D0–D1 |
| 6 | `0x1FF` | D2–D3 |
| 7 | `0x1FF` | D4–D5 |
| 8 | `0x1FF` | D6–D7 |

控制原始值范围为 `-16384..16384`，对应转矩电流 `-20..20 A`：

```text
current_A = raw * 20 / 16384
raw = clamp(round(current_A * 16384 / 20), -16384, 16384)
```

来源：`docs/datasheets/RoboMaster_C620_ESC_User_Manual_V1.01.pdf`，PDF 第 31–32 页。

### C620 反馈帧

- CAN ID：`0x200 + ESC_ID`，即 `0x201..0x208`。
- 默认发送频率：1 kHz，可通过 RoboMaster Assistant 修改。

| 字节 | 含义 | 解析 |
|---|---|---|
| D0–D1 | 转子机械角度 | 无符号 16 位，大端；有效范围 `0..8191` 对应 `0..360°` |
| D2–D3 | 转子转速 | 有符号 16 位，大端，单位 rpm |
| D4–D5 | 实际转矩电流 | 有符号 16 位，大端 |
| D6 | 电机温度 | ℃ |
| D7 | 保留 | 忽略 |

来源：`docs/datasheets/RoboMaster_C620_ESC_User_Manual_V1.01.pdf`，PDF 第 33 页。

### 电气和操作约束

- 额定电压 24 V，持续电流上限 20 A，工作环境 0–50 ℃。
- CAN 口带可切换的 120 Ω 终端电阻。
- CAN 与 PWM 端口不得同时连接；切换控制方式必须先断电。
- 更换电机或电调后可执行校准，以获得更好的适配参数；校准时电机会转动，应空载并远离机构。

来源：`docs/datasheets/RoboMaster_C620_ESC_User_Manual_V1.01.pdf`，PDF 第 22、24、31、35 页。

### M3508 关键参数

- 额定电压 24 V。
- 配 C620 时：空载 482 rpm，额定 469 rpm，最大连续转矩 3 N·m，额定输入电流 10 A。
- 转矩常数 0.3 N·m/A，减速比 `3591/187`，极对数 7。
- 工作环境 0–50 ℃，绕组最高允许温度 125 ℃。

来源：`docs/datasheets/RoboMaster_M3508_User_Manual_V1.0.pdf`，PDF 第 12–13 页。

## C610 + M2006

### C610 控制帧

C610 的控制分组、DLC 和大端字节布局与 C620 相同：

- `0x200` 控制 ID 1–4，`0x1FF` 控制 ID 5–8。
- 原始值 `-10000..10000` 对应转矩电流 `-10..10 A`。

```text
current_A = raw * 10 / 10000
raw = clamp(round(current_A * 10000 / 10), -10000, 10000)
```

来源：`docs/datasheets/RoboMaster_C610_ESC_User_Manual.pdf`，PDF 第 8 页。

### C610 反馈帧

- CAN ID：`0x200 + ESC_ID`，即 `0x201..0x208`。
- 默认发送频率：1 kHz。

| 字节 | 含义 |
|---|---|
| D0–D1 | 转子机械角度，大端，范围 `0..8191` |
| D2–D3 | 转速，大端有符号数，单位 rpm |
| D4–D5 | 实际输出转矩原始值；原手册未给出换算比例 |
| D6–D7 | 保留 |

来源：`docs/datasheets/RoboMaster_C610_ESC_User_Manual.pdf`，PDF 第 8–9 页。

### 电气和操作约束

- 额定电压 24 V，持续电流上限 10 A，工作环境 0–55 ℃。
- ID 范围 1–8；同一总线 ID 重复时冲突设备会关闭输出。
- 初次使用，或更换电机/电调后，必须执行电机校准。

来源：`docs/datasheets/RoboMaster_C610_ESC_User_Manual.pdf`，PDF 第 7、9 页。

### M2006 关键参数

- 额定电压 24 V。
- 配 C610 时：空载 500 rpm，额定 416 rpm，最大连续转矩 1 N·m，额定输入电流 3 A。
- 转矩常数 0.18 N·m/A，减速比 36:1，极对数 7，工作环境 0–55 ℃。

来源：`docs/datasheets/RoboMaster_M2006_P36_User_Manual.pdf`，PDF 第 8–9 页。

## GM6020

### ID 与经典电压控制协议

2018 版原厂手册只明确描述电压控制模式：

- 电机 ID 1–7；拨码值 0 为无效。
- `0x1FF` 控制 ID 1–4。
- `0x2FF` 控制 ID 5–7，D6–D7 保留。
- 每台电机占连续 2 字节，有符号 16 位，高字节在前。
- 电压控制原始值范围 `-30000..30000`。

反馈帧：

- CAN ID：`0x204 + Motor_ID`，即 `0x205..0x20B`。
- 默认频率：1 kHz。

| 字节 | 含义 |
|---|---|
| D0–D1 | 机械角度，大端，范围 `0..8191` |
| D2–D3 | 转速，大端有符号数，单位 rpm |
| D4–D5 | 实际转矩电流，大端有符号数 |
| D6 | 电机温度，℃ |
| D7 | 保留 |

来源：`docs/datasheets/RoboMaster_GM6020_User_Manual.pdf`，PDF 第 7–8 页。

### 版本敏感提醒

- 本地 2018 原厂手册没有给出 `0x1FE` 电流模式定义。
- 不得仅凭网络代码片段把 `0x1FE` 当作所有 GM6020 固件都支持的固定协议。
- 若项目需要电流模式，先通过 RoboMaster Assistant 确认设备固件、控制模式和量程，再把实测结果写入项目驱动。

## 通用打包与解析

### 大端 16 位

```c
static inline int16_t be_i16(const uint8_t hi, const uint8_t lo)
{
    return (int16_t)((uint16_t)hi << 8 | lo);
}

static inline void put_be_i16(uint8_t data[2], int16_t value)
{
    data[0] = (uint8_t)((uint16_t)value >> 8);
    data[1] = (uint8_t)value;
}
```

### 多圈角度

RoboMaster 反馈角度是单圈 `0..8191`。多圈累计时，应以相邻采样差判断过零：

```text
delta = current - previous
if delta > 4096:  delta -= 8192
if delta < -4096: delta += 8192
total_ecd += delta
```

首次收到反馈时只初始化基准，不累计圈数；设备掉线重连后需要重新建立基准。

## CAN 总线调试清单

1. C 板与所有设备统一配置为经典 CAN、标准帧、1 Mbps。
2. 总线只在物理两端各接一个 120 Ω 终端电阻；断电测 CANH-CANL 应接近 60 Ω。
3. 逐台设置唯一 ID，再并入总线。
4. 先只监听反馈，确认 ID、DLC、周期和字节序。
5. 初次下发控制量从 0 开始，缓慢增加并做软件限幅。
6. 停机、超时或急停时显式发送 0 控制量，不能只停止刷新应用层目标。
7. 多台设备默认 1 kHz 反馈会快速占用总线带宽；共享总线时应核算负载，并按需降低可配置设备的反馈频率。
8. C620/C610 的反馈 ID 区间相同；如果共用一条总线，必须统一规划 ID。

## 机械与线束维护

- M3508 每天工作不超过 8 小时时，原厂建议使用至 3 个月检查并补充或更换润滑脂。
- M3508 闲置超过 2 个月，再次使用前应检查和保养减速箱。
- 出现撞击声、轴向游隙、轴承异响、传动阻力突增或转速持续明显下降时，应停止使用并检查。
- M3508 附件终端板的 8-Pin 端口和单路 XT30 端口标称最大持续 20 A，XT60 总电源线标称最大持续 40 A。
- 面向 M3508 输出轴观察时，原厂混控教程定义逆时针为正转；实际闭环仍需低速测试确认安装方向和反馈符号。

来源：

- `docs/datasheets/RoboMaster_M3508_Maintenance_Manual.pdf`，PDF 第 2–3 页。
- `docs/datasheets/RoboMaster_M3508_Accessory_Kit_User_Manual_V1.0.pdf`，PDF 第 4–5 页。
- `docs/datasheets/RoboMaster_M3508_Mixed_Control_Tutorial_V1.0.pdf`，PDF 第 2–5 页。
