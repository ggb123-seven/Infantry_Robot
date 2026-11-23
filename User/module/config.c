#include "module/config.h"

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
 * - yaw 电流上限 3.0 A（高于额定 1.62 A 的竞技常用量级）、pitch 取 1.6 A：pitch 俯仰轴重力不平衡，
 *   水平保持约需 0.5 A，参考工作树实测重力前馈为 0.47 A，0.5 A 不足以稳定保持
 * - 速度环 Kp=0.06、Ki=0.177 为已整定值，yaw SPEED 调试路径沿用该环，积分限幅取 17 rad，约 3 A
 * - pitch 速度环积分限幅取 3.5 rad，约 0.62 A，覆盖重力保持电流并留余量
 * - yaw ANGLE 使用直出电流滑模，不读取角度环和速度环；两环参数仅用于 SPEED 调试与 pitch
 * - 两轴角度环统一采用当前 yaw 参数 Kp=15、Ki=0.005、Kd=0，积分限幅为 1.0 rad·s
 * - pitch 水平零点 239.5459° 为 2026-11-23 实测，即俯仰摆到水平位时的编码器读数
 * - pitch 机械行程按 2026-11-23 实测两端限位：上限位机械角 -35.16°、下限位 +27.51°，
 *   配置取 -33.0° 到 25.5°，两端各留约 2° 余量
 * - pitch 两环参数取自参考工作树实测值：角度环 k=2/p=5/out=6 rad/s，速度环 k=0.45/p=1/i=0.5/out=1.0
 * - pitch 俯仰轴重力不平衡，向下运动有重力助力、向上运动需对抗重力，后续可考虑增加重力前馈
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
        .max_current_a = 3.0f,
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
            .i_limit = 17.0f,
            .out_limit = 3.0f,
            .d_cutoff_freq = 0.0f,
            .range = 0.0f,
        },
    },
    .pitch =
    {
        .enabled = true,
        .reversed = false,
        .mechanical_limit_enabled = true,
        .zero_encoder_deg = 239.5459f,
        .min_angle_deg = -33.0f,
        .max_angle_deg = 25.5f,
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
            .d_cutoff_freq = 0.0f,
            .range = 0.0f,
        },
        .speed_pid =
        {
            .k = 0.45f,
            .p = 1.0f,
            .i = 0.5f,
            .d = 0.0f,
            .i_limit = 1.4f,
            .out_limit = 1.0f,
            .d_cutoff_freq = 0.0f,
            .range = 0.0f,
        },
    },
    .feedback_timeout_s = 0.09f,
    .imu_timeout_s = 0.01f,
};
