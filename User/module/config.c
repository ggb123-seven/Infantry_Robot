/*
 * 整机配置模块
 *
 * 集中定义各业务模块的只读默认配置与标定系数，模块初始化时复制到各自实例
 */

/* Includes ----------------------------------------------------------------- */
#include "module/config.h"
#include <stdbool.h>

/* Exported constants ------------------------------------------------------- */
/*
 * 拨弹默认配置，初始化只复制本配置，不再从底盘或云台覆盖参数
 *
 * tooth_count：一圈齿数，单发步距为 360 度除以齿数
 * feed_hz：单发重复时每秒进给齿数上限，单位：Hz，实际推进还须等待到位
 * direction：角度递增方向，取 +1，按用户要求反转拨弹进给方向
 * max_speed_rpm：输出轴速度目标上限，单位：rpm
 * max_current_a：转子电流上限，单位：A
 * position_pid：角度外环，输入输出轴弧度，range 为 2π，输出限幅按速度上限换算
 * speed_pid：速度内环，输入输出轴 rpm，输出 A，range 为零
 * speed_feedforward_gain_a_per_rpm：目标逐周期差分的电流前馈系数，单位：A/rpm，零表示关闭
 *
 * 逐齿配置为一圈 8 齿、每秒 20 齿、方向 +1，每齿对应一发，满倍率输出轴约 150 rpm
 * 速度环参数取用户提供的整定值，实物方向和闭环响应仍须上板验证
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
 * 双摩擦轮默认配置，两轮使用相同参数、独立控制历史，目标符号由各自命令指定
 *
 * max_speed_rpm：转子目标转速上限，单位：rpm
 * max_current_a：每轮转子电流上限，单位：A
 * speed_pid：两轮共用的速度环参数，输入转子 rpm、输出电流 A，range 为零
 *
 * 速度环参数独立保存迁移前实际使用的整定值，不借用底盘参数
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
 * 单位：A/rpm，保留既有在线调参变量和数值，零表示关闭
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
 * 底盘默认配置，电流边界按用户要求取参考工程电流边界的一半
 *
 * layout：底盘布局，当前为 X 型全向轮
 * max_wheel_speed_rpm：输出轴最大转速，单位：rpm
 * wheel_accel_rpm_s：目标转速斜坡上限，单位：rpm/s
 * reverse_release_speed_rpm：换向放行阈值，单位：rpm
 * speed_filter_cutoff_hz：速度反馈二阶低通截止频率，单位：Hz
 * current_filter_cutoff_hz：电流滤波截止频率，单位：Hz，负数表示直通
 * max_current_a：最终转子侧电流上限，单位：A
 * wheel_speed_pid：四轮共用速度环，输入输出轴 rpm，输出电流 A，range 为零
 * follow_pid：跟随角度环，输入弧度、输出归一化转向，误差按 2π 环绕
 *
 * 布局及两处滤波截止频率仅初始化时生效，其他参数可在实例配置中在线调整
 * 跟随环直接使用编码器相对角速度作为微分输入，负截止频率表示不滤波
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
 * 云台默认配置，按结构体字段逐项说明，字段定义和校验约束见 gimbal.h
 *
 * yaw、pitch：单轴配置 Gimbal_AxisConfig_t，两轴字段含义相同
 * - enabled：轴使能，false 时该轴始终零输出
 * - reversed：方向反转开关，true 时编码器角度、转速和输出电流同时取反
 * - mechanical_limit_enabled：是否按编码器机械角限制目标，yaw 行程未确认故关闭
 * - zero_encoder_deg：机械零点对应的方向修正后编码器角度，单位：度，范围 [0, 360)
 * - min_angle_deg、max_angle_deg：相对机械零点的行程上下限，单位：度，机械限位开启时生效
 * - max_speed_rad_s：目标角速度绝对值上限，单位：rad/s，按 320 rpm 空载转速折算
 * - max_current_a：输出电流绝对值上限，单位：A，控制链末端按它统一限幅
 * - angle_pid：角度外环，输入角误差 rad，输出目标角速度 rad/s
 * - speed_pid：速度内环，输入角速度误差 rad/s，输出电流 A
 *
 * angle_pid、speed_pid：PID 参数 KPID_Params_t，四个控制环字段含义相同
 * - k、p、i、d：并联结构增益和比例、积分、微分系数，等效增益 Kp=k·p、Ki=k·i、Kd=k·d
 * - i_limit、out_limit：积分项和输出绝对值的限幅，积分限幅为 0 表示不积分
 * - d_cutoff_freq：微分项低通截止频率，单位：Hz，负数表示不滤波
 * - range：误差环绕周期，单位：rad，大于 0 时按周期折算角度误差
 *
 * feedback_timeout_s：电机反馈超时，单位：s，超时后该轴断开连续角并零输出
 * imu_timeout_s：惯性反馈超时，单位：s，按 INS 源采样时刻校验
 *
 * yaw_friction_current_a、yaw_friction_linear_a_per_rad_s、yaw_friction_quadratic_a_per_rad_s2：
 *   yaw 二次摩擦模型的常数项 a0、一次项 a1、二次项 a2，单位：A、A/(rad/s)、A/(rad/s)^2
 * yaw_friction_smoothing_rad_s：零速附近线性过渡半宽，单位：rad/s
 * yaw_friction_max_speed_rad_s：模型速度上限，单位：rad/s，0 表示关闭摩擦补偿
 * 摩擦电流 i = a0 + a1·v + a2·v²，v 取限幅后的闭环目标角速度，仅 yaw 使用
 *
 * pitch_gravity_enabled：pitch 重力前馈开关，false 时前馈恒为零
 * pitch_gravity_current_a：重力前馈幅值 A，单位：A
 * pitch_gravity_angle_deg：重力前馈相位 θ0，即重力力矩最大处的大地系俯仰角，单位：度
 * pitch_gravity_bias_a：重力前馈常值项 i0，单位：A
 * 重力前馈电流 i = A·cos(θ - θ0) + i0，θ 取 IMU 大地系俯仰角，不用编码器机械角
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
    .pitch_gravity_enabled = true,
    .pitch_gravity_current_a = 0.47f,
    .pitch_gravity_angle_deg = 180.0f,
    .pitch_gravity_bias_a = 0.0f,
};
