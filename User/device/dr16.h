#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "device/device.h"

#define DR16_FRAME_LENGTH (18U)

/*
 * DR16 接收器参数：
 * - DR16_RECEIVER_BUFFER_SIZE：协议字节流缓冲区容量，实际可用容量比定义值少 1 字节
 * - DR16_RECEIVER_OFFLINE_TIMEOUT_US：连续未收到合法帧后的离线判定时间，单位微秒
 */
#define DR16_RECEIVER_BUFFER_SIZE (128U)
#define DR16_RECEIVER_OFFLINE_TIMEOUT_US (100000ULL)

/**
 * @brief DR16 接收器状态
 */
typedef enum
{
    DR16_RECEIVER_OK = 0,
    DR16_RECEIVER_ERROR = -1,
    DR16_RECEIVER_NULL_ERROR = -2,
    DR16_RECEIVER_NOT_INITIALIZED = -3,
    DR16_RECEIVER_OVERFLOW = -4,
} DR16_ReceiverStatus_t;

/**
 * @brief DR16 拨杆位置
 */
typedef enum
{
    DR16_SWITCH_ERROR = 0,
    DR16_SWITCH_UP = 1,
    DR16_SWITCH_DOWN = 2,
    DR16_SWITCH_MIDDLE = 3,
} DR16_SwitchPosition_t;

/**
 * @brief DR16 键盘按键位编号
 */
typedef enum
{
    DR16_KEY_W = 0,
    DR16_KEY_S,
    DR16_KEY_A,
    DR16_KEY_D,
    DR16_KEY_SHIFT,
    DR16_KEY_CTRL,
    DR16_KEY_Q,
    DR16_KEY_E,
    DR16_KEY_R,
    DR16_KEY_F,
    DR16_KEY_G,
    DR16_KEY_Z,
    DR16_KEY_X,
    DR16_KEY_C,
    DR16_KEY_V,
    DR16_KEY_B,
    DR16_KEY_COUNT,
} DR16_Key_t;

/*
 * DR16 完整合法帧的解码结果：
 * - ch_l_x、ch_l_y、ch_r_x、ch_r_y、wheel：减去中心值 1024 后的原始计数，典型范围 -660~660。
 * - sw_l、sw_r：左右拨杆位置，取值见 DR16_SwitchPosition_t。
 * - mouse_x、mouse_y、mouse_z：鼠标三个方向的原始有符号计数。
 * - mouse_left、mouse_right：鼠标左右键状态。
 * - key_mask：键盘 W~B 对应位 0~15，位编号见 DR16_Key_t。
 */
typedef struct
{
    int16_t ch_l_x;
    int16_t ch_l_y;
    int16_t ch_r_x;
    int16_t ch_r_y;
    int16_t wheel;
    DR16_SwitchPosition_t sw_l;
    DR16_SwitchPosition_t sw_r;
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    bool mouse_left;
    bool mouse_right;
    uint16_t key_mask;
} DR16_Data_t;

/*
 * DR16 接收器稳定状态：
 * - header：在线状态和最后合法帧时间，由 DR16 接收器维护
 * - valid_frame_sequence：每收到一帧完整合法数据递增一次，由 DR16 接收器维护
 * - data：最近一帧完整合法的协议解码结果
 */
typedef struct
{
    DEVICE_Header_t header;
    uint32_t valid_frame_sequence;
    DR16_Data_t data;
} DR16_State_t;

/*
 * DR16 接收器诊断快照：
 * - initialized：协议字节流缓冲区初始化成功时为 true
 * - valid_frame_count：累计提交的完整合法帧数量
 * - invalid_frame_count：重同步期间累计失败的候选帧数量
 * - resync_discarded_byte_count：逐字节重同步累计丢弃的字节数量
 * - ring_buffer_overflow_count：协议字节流缓冲区累计溢出次数
 */
typedef struct
{
    bool initialized;
    uint32_t valid_frame_count;
    uint32_t invalid_frame_count;
    uint32_t resync_discarded_byte_count;
    uint32_t ring_buffer_overflow_count;
} DR16_ReceiverDiagnostics_t;

/**
 * @brief 解码并校验一帧 DR16 数据
 *
 * 只有长度、五个通道和两个拨杆全部合法时才覆盖输出，失败时输出保持不变。
 *
 * @param[in] frame 待解码的原始字节数组
 * @param[in] length 原始字节数组长度，必须等于 DR16_FRAME_LENGTH
 * @param[out] output 完整合法帧的解码结果
 * @return 成功返回 DEVICE_OK，失败返回对应设备状态码
 */
int8_t DR16_Decode(const uint8_t *frame, size_t length, DR16_Data_t *output);

/**
 * @brief 将 DR16 状态恢复为离线零值
 *
 * @param[out] state 待复位的 DR16 状态，允许为 NULL
 * @return 无返回值
 */
void DR16_ResetState(DR16_State_t *state);

/**
 * @brief 初始化 DR16 协议字节流接收器
 *
 * @return 初始化成功返回 DR16_RECEIVER_OK，否则返回对应接收器状态
 */
int8_t DR16_ReceiverInit(void);

/**
 * @brief 将 UART 新增字节写入 DR16 协议字节流
 *
 * 本函数供单一 UART 中断生产者调用，只复制字节并记录溢出状态
 *
 * @param[in] data 本次新增字节段首地址
 * @param[in] length 本次新增字节数
 * @return 全部写入返回 DR16_RECEIVER_OK，否则返回对应接收器状态
 */
int8_t DR16_ReceiverFeed(const uint8_t *data, uint16_t length);

/**
 * @brief 解码全部候选帧并更新 DR16 在线状态
 *
 * @param[in] now_us 当前时间，单位微秒
 * @return 业务状态发生变化时返回 true，否则返回 false
 */
bool DR16_ReceiverUpdate(uint64_t now_us);

/**
 * @brief 强制 DR16 切换为离线安全状态
 *
 * @return 无返回值
 */
void DR16_ReceiverSetOffline(void);

/**
 * @brief 清空 DR16 协议字节流中的未验证数据
 *
 * 调用方必须先停止 UART 字节交付，避免生产者与消费者并发复位
 *
 * @return 复位成功返回 DR16_RECEIVER_OK，否则返回对应接收器状态
 */
int8_t DR16_ReceiverResetStream(void);

/**
 * @brief 获取 DR16 一致状态快照
 *
 * @param[out] state 待写入的 DR16 状态快照
 * @return 成功返回 DR16_RECEIVER_OK，否则返回对应接收器状态
 */
int8_t DR16_ReceiverGetState(DR16_State_t *state);

/**
 * @brief 获取 DR16 接收器诊断快照
 *
 * @param[out] diagnostics 待写入的接收器诊断快照
 * @return 成功返回 DR16_RECEIVER_OK，否则返回对应接收器状态
 */
int8_t DR16_ReceiverGetDiagnostics(DR16_ReceiverDiagnostics_t *diagnostics);

#ifdef __cplusplus
}
#endif
