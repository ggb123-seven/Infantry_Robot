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
 * DR16 任务发布的稳定状态：
 * - header：在线状态和最后合法帧时间，由 DR16 任务维护。
 * - valid_frame_sequence：每收到一帧完整合法数据递增一次，由 DR16 任务维护。
 * - data：最近一帧完整合法的协议解码结果。
 */
typedef struct
{
    DEVICE_Header_t header;
    uint32_t valid_frame_sequence;
    DR16_Data_t data;
} DR16_State_t;

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

#ifdef __cplusplus
}
#endif
