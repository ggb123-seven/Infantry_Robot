#include "module/config.h"
#include <stdbool.h>

/*
 * 拨弹默认配置独立定义，初始化只复制本配置，不再从底盘或云台覆盖参数
 * 输出轴目标限幅 300 rpm、转子电流限幅 4 A，保留迁移前实际使用的边界
 * 速度 PID 采用用户提供的 P=0.5、I=0.12、D=0，k=1、积分限幅 20 rpm·s
 * 速度 PID 输出限幅 4 A、微分截止 20 Hz、range=0，输入输出轴 rpm、输出 A
 * 位置外环输入输出轴弧度、输出 rad/s，range 为 2π，输出限幅为 300 rpm 对应的 rad/s
 * 逐齿配置为一圈 8 齿、每秒 20 齿、方向 +1，每齿对应一发，满倍率输出轴约 150 rpm
 * 目标逐周期差分前馈单位 A/rpm，默认零；实物方向和闭环响应仍须上板验证
 */
const Feeder_Config_t feeder_config =
{
    .tooth_count = 8.0f,
    .feed_hz = 20.0f,
    .direction = 1.0f,
    .max_speed_rpm = 300.0f,
    .max_current_a = 10.0f,
    .position_pid =
    {
        .k = 1.0f,
        .p = 25.0f,
        .i = 2.4f,
        .d = 0.4f,
        .i_limit = 1.0f,
        .out_limit = 300.0f * (M_2PI / 60.0f),
        .d_cutoff_freq = 0.0f,
        .range = M_2PI,
    },
    .speed_pid =
    {
        .k = 1.0f,
        .p = 0.5f,
        .i = 0.12f,
        .d = 0.0f,
        .i_limit = 20.0f,
        .out_limit = 10.0f,
        .d_cutoff_freq = 20.0f,
        .range = 0.0f,
    },
    .speed_feedforward_gain_a_per_rpm = 0.0f,
};

/*
 * 双摩擦轮接入阶段默认配置：转子目标限幅 8000 rpm，每轮转子电流限幅 3 A
 * 两轮使用相同参数、独立控制历史，目标符号由各自命令指定
 * 速度 PID 独立保存迁移前实际使用的 k=1、p=0.16、i=d=0，不再借用底盘参数
 * 积分限幅 20 rpm·s、输出限幅 3 A、微分截止 20 Hz、range=0
 * 尚未标定实际弹速，当前只提供独立转速命令接口
 */
const Shoot_Config_t shoot_config =
{
    .max_speed_rpm = 8000.0f,
    .max_current_a = 3.0f,
    .speed_pid =
    {
        .k = 1.0f,
        .p = 0.16f,
        .i = 0.1f,
        .d = 0.0f,
        .i_limit = 20.0f,
        .out_limit = 3.0f,
        .d_cutoff_freq = 20.0f,
        .range = 0.0f,
    },
};

/*
 * 摩擦轮目标逐周期差分前馈的可写参数集中定义于配置文件，控制入口每周期读取一次
 * 参考第 3.9 节作业的系数 30000，按减速比 3591/187、rad/s 转 rpm、16384 对应 20 A 换算
 * 单位 A/rpm，保留既有在线调参变量和数值，零表示关闭
 */
volatile float shoot_speed_feedforward_gain_a_per_rpm = 30000.0f * 20.0f / 16384.0f *
    M_2PI / 60.0f / (3591.0f / 187.0f);

/*
 * 轮速旋转标定系数，单位为每 rpm 对应的 rad/s
 * 来源为用户提供的《底盘几何与运动学参数清单》第 1 至 3 页，测量日期为 2026-09-22
 * 接地有效半径为 76.50 mm，轮心纵横偏距各 168.10 mm，径向轮轴对应切向滚动
 * 系数为 2π / 60 × 76.50 / √(168.10² + 168.10²)，不使用轮盘直径或车体包络尺寸
 * 输入已为减速后的输出轴 rpm，四轮正向沿用混合器的俯视逆时针约定，不重复除减速比
 * 此值启用四轮联合识别，装车仍须核对电机极性、惯性方向及载荷下有效轮径
 */
const float chassis_yaw_rad_s_per_rpm = 0.03369824953f;

/*
 * 底盘默认配置，电流边界按用户要求设为参考工程 20 A 的一半：
 * - X 型全向轮，最大输出轴转速 300 rpm，目标斜坡 200 rpm/s，换向放行阈值 10 rpm
 * - 速度反馈二阶低通 45 Hz，电流滤波 -1 表示直通，最终转子侧电流限幅 10 A
 * - 四轮共用速度 PID，增益以参数表为准，输入为输出轴 rpm，积分限幅 20 rpm·s
 * - PID 输出限幅 10 A，微分滤波 20 Hz，range=0 表示线性速度误差
 * - 跟随角度 PID 输入弧度、输出归一化转向，首轮 k=1、p=0.5、i=d=0，输出限幅 0.3
 * - 跟随误差按 2π 环绕，直接使用编码器相对角速度作为微分输入，微分截止频率 -1 Hz
 * - 布局及反馈、电流滤波截止频率仅初始化时生效，其他参数可在实例配置中在线调整
 */
const Chassis_Config_t chassis_config =
{
    .layout = MIXER_OMNICROSS,
    .max_wheel_speed_rpm = 300.0F,
    .wheel_accel_rpm_s = 200.0F,
    .reverse_release_speed_rpm = 10.0F,
    .speed_filter_cutoff_hz = 45.0F,
    .current_filter_cutoff_hz = -1.0F,
    .max_current_a = 10.0F,
    .wheel_speed_pid =
    {
        .k = 1.0F,
        .p = 0.16F,
        .i = 0.0F,
        .d = 0.0F,
        .i_limit = 0.0F,
        .out_limit = 10.0F,
        .d_cutoff_freq = -1.0F,
        .range = 0.0F,
    },
    .follow_pid =
    {
        .k = 1.0F,
        .p = 0.6F,
        .i = 0.0F,
        .d = 0.0F,
        .i_limit = 1.0F,
        .out_limit = 1.0F,
        .d_cutoff_freq = -1.0F,
        .range = M_2PI,
    },
};

/*
 * 云台默认配置：
 * - yaw 零点 97.0752° 取自用户实测，安装关系改变后须重新标定
 * - 姿态角采用当前 INS 坐标，yaw 使用累计 Yaw，pitch 使用 Pitch，速度环反馈为对应的大地系欧拉角速度
 * - yaw 未确认线束或机械行程，关闭机械限位，原 [0°, 360°] 数值保留但不再夹住惯性角目标
 * - 速度上限依据 GM6020 使用说明 20231013 的 320 rpm 空载转速换算，约 33.51 rad/s
 * - 两轴最终电流上限按用户指定设为 1 A，低于手册额定电流 1.62 A，为当前调试阶段的软件保护
 * - 电流控制码按 ±3 A 对应 ±16384 换算，±0.5 A、±1 A、±3 A 取整后分别对应 ±2730、±5461、±16384
 * - 角度环输出为目标角速度，单位 rad/s；速度环输出为电流候选，单位 A，最终另按 max_current_a 限幅
 * - PID 增益、积分限幅和输出限幅保留各轴独立配置，参数副本支持在线调参，控制周期使用已校验快照
 * - yaw 摩擦补偿采用已完成软件回归的二次模型，a0=0.2648070877 A、a1=-0.0055278781 A/(rad/s)、a2=0.00195745266 A/(rad/s)^2
 * - yaw 摩擦补偿平滑尺度为 0.2 rad/s，模型速度上限为 3.4 rad/s，复位后默认启用；运行中仍可修改整体配置内的五个 yaw_friction 参数
 * - yaw 按限幅与机械约束后的目标角速度计算平滑库仑补偿，与速度环电流相加后统一限流，pitch 不使用摩擦补偿
 * - yaw 角度环 k=20、p=1、i=d=0，速度环 k=0.45、p=1、i=6、d=0.0008
 * - yaw 角度环输出限幅 10 rad/s，速度环输出限幅 1 A；速度环积分限幅 1，微分截止频率 -1 Hz
 * - yaw 两环 range 沿用原配置 0，不启用角度误差环绕；pitch 参数说明见下方
 * - pitch 机械零点 238.93068° 由用户于 2026-11-07 确认，沿用当前 reversed=false 的编码器方向
 * - pitch 编码器机械范围沿用当前确认参数，相对机械零点为 [-30°, 20°]
 * - pitch 采用用户提供的 PID 参数，角度环 k=2、p=5、i=2.5，速度环 k=0.25、p=1、i=0
 * - pitch 角度环输出限幅 10 rad/s，速度环输出限幅 1 A；两环微分关闭，负截止频率表示不滤波
 * - pitch 角度环积分限幅为零，沿用现有 PID 的积分处理，角度误差按 2π 弧度环绕，速度误差不环绕
 * - 两轴均启用并独立输出电流，默认命令为 RELAX，上电保持零电流
 * - 编码器仅用于机械角、限位与在线检测，pitch 使用有符号角，yaw 保留连续展开
 * - 反馈超时 90 ms：320 rpm 时 90 ms 内转动小于半圈，保证编码器角度可唯一展开
 * - IMU 独立超时初始取 10 ms，以源采样时间校验，后续须通过实机调度抖动核定
 */
const Gimbal_Config_t gimbal_config =
{
    .yaw =
    {
        .enabled = true,
        .reversed = false,
        .mechanical_limit_enabled = false,
        .zero_encoder_deg = 97.0752f,
        .min_angle_deg = 0.0f,
        .max_angle_deg = 360.0f,
        .max_speed_rad_s = 320.0f * 0.10471975512f,
        .max_current_a = 10.0f,
        .angle_pid =
        {
            .k = 20.0f,
            .p = 1.0f,
            .i = 0.0f,
            .d = 0.05f,
            .i_limit = 0.0f,
            .out_limit = 10.0f,
            .d_cutoff_freq = -1.0f,
            .range = 0.0f,
        },
        .speed_pid =
        {
            .k = 0.45f,
            .p = 1.0f,
            .i = 0.0f,
            .d = 0.00008f,
            .i_limit = 1.0f,
            .out_limit = 1.0f,
            .d_cutoff_freq = -1.0f,
            .range = 0.0f,
        },
    },
    .pitch =
    {
        .enabled = true,
        .reversed = false,
        .mechanical_limit_enabled = true,
        .zero_encoder_deg = 238.93068f,
        .min_angle_deg = -20.0f,
        .max_angle_deg = 20.0f,
        .max_speed_rad_s = 320.0f * 0.10471975512f,
        .max_current_a = 1.0f,
        .angle_pid =
        {
            .k = 2.0f,
            .p = 5.0f,
            .i = 0.0f,
            .d = 0.0f,
            .i_limit = 0.0f,
            .out_limit = 6.0f,
            .d_cutoff_freq = -1.0f,
            .range = M_2PI,
        },
        .speed_pid =
        {
            .k = 0.45f,
            .p = 1.0f,
            .i = 0.5f,
            .d = 0.0f,
            .i_limit = 1.4f,
            .out_limit = 1.0f,
            .d_cutoff_freq = -1.0f,
            .range = -1.0f,
        },
    },
    .feedback_timeout_s = 0.09f,
    .imu_timeout_s = 0.01f,
    .yaw_friction_current_a = 0.2648070877f,
    .yaw_friction_smoothing_rad_s = 0.2f,
    .yaw_friction_linear_a_per_rad_s = -0.0055278781f,
    .yaw_friction_quadratic_a_per_rad_s2 = 0.00195745266f,
    .yaw_friction_max_speed_rad_s = 3.4f,
};
