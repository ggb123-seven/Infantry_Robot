#include "module/config.h"

/*
 * 拨弹接入阶段默认配置：输出轴目标限幅 300 rpm，转子电流限幅 2 A
 * 速度 PID 的 k=1、p/i/d=0，未调参时即使命令使能也保持零电流
 * 积分限幅 20 rpm·s、输出限幅 2 A、微分截止 20 Hz、range=0
 * 这些限幅仅作为初始调试边界，不表示已完成真实机构整定
 */
const Trigger_Config_t trigger_config =
{
    .max_speed_rpm = 300.0f,
    .max_current_a = 2.0f,
    .speed_pid =
    {
        .k = 1.0f,
        .p = 0.0f,
        .i = 0.0f,
        .d = 0.0f,
        .i_limit = 20.0f,
        .out_limit = 2.0f,
        .d_cutoff_freq = 20.0f,
        .range = 0.0f,
    },
};

/*
 * 双摩擦轮接入阶段默认配置：转子目标限幅 8000 rpm，每轮转子电流限幅 3 A
 * 两轮使用相同参数、独立控制历史，目标符号由各自命令指定
 * 速度 PID 的 k=1、p/i/d=0，未调参时保持零电流
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
        .p = 0.0f,
        .i = 0.0f,
        .d = 0.0f,
        .i_limit = 20.0f,
        .out_limit = 3.0f,
        .d_cutoff_freq = 20.0f,
        .range = 0.0f,
    },
};

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
 * 底盘默认配置，沿用底盘重构方案确认的原参数：
 * - X 型全向轮，最大输出轴转速 300 rpm，目标斜坡 200 rpm/s，换向放行阈值 10 rpm
 * - 速度反馈二阶低通 45 Hz，电流滤波 -1 表示直通，最终转子侧电流限幅 4 A
 * - 四轮共用原 ID4 速度 PID：k=1、p=0.16、i=0、d=0，积分限幅 20 rpm·s
 * - PID 输出限幅 4 A，微分滤波 20 Hz，range=0 表示线性速度误差
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
    .max_current_a = 4.0F,
    .wheel_speed_pid =
    {
        .k = 1.0F,
        .p = 0.16F,
        .i = 0.0F,
        .d = 0.0F,
        .i_limit = 20.0F,
        .out_limit = 4.0F,
        .d_cutoff_freq = 20.0F,
        .range = 0.0F,
    },
};

/*
 * 云台默认配置：
 * - yaw 零点 97.0752° 取自用户实测，安装关系改变后须重新标定
 * - 姿态角采用当前 INS 坐标，yaw 使用累计 Yaw，pitch 使用 Pitch，速度环反馈为对应的大地系欧拉角速度
 * - yaw 未确认线束或机械行程，关闭机械限位，原 [0°, 360°] 数值保留但不再夹住惯性角目标
 * - 速度上限依据 GM6020 使用说明 20231013 的 320 rpm 空载转速换算，约 33.51 rad/s
 * - 电流上限 0.5 A，低于手册额定电流 1.62 A，为当前调试阶段的软件保护
 * - 速度环 Kp=0.06、Ki=0.177 为已整定值，积分限幅取电流上限除以 Ki，约 2.825 rad
 * - 两轴角度环统一采用当前 yaw 参数 Kp=15、Ki=0.005、Kd=0，积分限幅为 1.0 rad·s
 * - pitch 的两环参数、速度和电流上限与 yaw 对齐，参数副本独立保存，后续可分别整定
 * - pitch 水平零点 118.08016° 取自用户实测，沿用当前 reversed=false 的编码器方向
 * - pitch 编码器机械范围按用户指定设为水平零点两侧各 60°，即 [-60°, 60°]
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
        .max_current_a = 0.5f,
        .angle_pid =
        {
            .k = 1.0f,
            .p = 15.0f,
            .i = 0.005f,
            .d = 0.0f,
            .i_limit = 1.0f,
            .out_limit = 320.0f * 0.10471975512f,
            .d_cutoff_freq = 0.0f,
            .range = 0.0f,
        },
        .speed_pid =
        {
            .k = 1.0f,
            .p = 0.06f,
            .i = 0.177f,
            .d = 0.0f,
            .i_limit = 0.5f / 0.177f,
            .out_limit = 0.5f,
            .d_cutoff_freq = 0.0f,
            .range = 0.0f,
        },
    },
    .pitch =
    {
        .enabled = true,
        .reversed = false,
        .mechanical_limit_enabled = true,
        .zero_encoder_deg = 118.08016f,
        .min_angle_deg = -60.0f,
        .max_angle_deg = 60.0f,
        .max_speed_rad_s = 320.0f * 0.10471975512f,
        .max_current_a = 0.5f,
        .angle_pid =
        {
            .k = 1.0f,
            .p = 15.0f,
            .i = 0.005f,
            .d = 0.0f,
            .i_limit = 1.0f,
            .out_limit = 320.0f * 0.10471975512f,
            .d_cutoff_freq = 0.0f,
            .range = 0.0f,
        },
        .speed_pid =
        {
            .k = 1.0f,
            .p = 0.06f,
            .i = 0.177f,
            .d = 0.0f,
            .i_limit = 0.5f / 0.177f,
            .out_limit = 0.5f,
            .d_cutoff_freq = 0.0f,
            .range = 0.0f,
        },
    },
    .feedback_timeout_s = 0.09f,
    .imu_timeout_s = 0.01f,
};
