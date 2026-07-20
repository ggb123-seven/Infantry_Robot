#include "device/dr16.h"

#include <string.h>

/*
 * DR16 DBUS 协议约束：
 * - 完整帧固定为 18 字节，所有多字节字段采用小端序。
 * - 字节 0~5 保存四个 11 位摇杆通道和两个 2 位拨杆。
 * - 字节 6~15 保存鼠标三轴、鼠标按键和键盘位图。
 * - 字节 16~17 保存第五通道。
 * - 五个通道原始中心值为 1024，合法范围为 364~1684。
 * @datasheet DJI DR16 DBUS 协议
 */
#define DR16_CHANNEL_MIN (364U)
#define DR16_CHANNEL_CENTER (1024U)
#define DR16_CHANNEL_MAX (1684U)
#define DR16_CHANNEL_MASK (0x07FFU)

static uint16_t DR16_ReadUint16LittleEndian(const uint8_t *data);
static int16_t DR16_ReadInt16LittleEndian(const uint8_t *data);
static bool DR16_IsChannelValid(uint16_t channel);
static bool DR16_IsSwitchValid(DR16_SwitchPosition_t switch_position);

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
int8_t DR16_Decode(const uint8_t *frame, size_t length, DR16_Data_t *output)
{
    if (frame == NULL || output == NULL)
    {
        return DEVICE_ERR_NULL;
    }
    if (length != DR16_FRAME_LENGTH)
    {
        return DEVICE_ERR;
    }

    // 先解码到局部变量，确保非法帧不会污染调用方保存的上一份合法状态
    const uint16_t channel_raw[5] =
    {
        (uint16_t)(((uint16_t)frame[0] | ((uint16_t)frame[1] << 8U)) & DR16_CHANNEL_MASK),
        (uint16_t)((((uint16_t)frame[1] >> 3U) | ((uint16_t)frame[2] << 5U)) & DR16_CHANNEL_MASK),
        (uint16_t)((((uint16_t)frame[2] >> 6U) | ((uint16_t)frame[3] << 2U) |
                    ((uint16_t)frame[4] << 10U)) & DR16_CHANNEL_MASK),
        (uint16_t)((((uint16_t)frame[4] >> 1U) | ((uint16_t)frame[5] << 7U)) & DR16_CHANNEL_MASK),
        DR16_ReadUint16LittleEndian(&frame[16]),
    };
    DR16_Data_t decoded = {0};
    decoded.ch_r_x = (int16_t)channel_raw[0] - (int16_t)DR16_CHANNEL_CENTER;
    decoded.ch_r_y = (int16_t)channel_raw[1] - (int16_t)DR16_CHANNEL_CENTER;
    decoded.ch_l_x = (int16_t)channel_raw[2] - (int16_t)DR16_CHANNEL_CENTER;
    decoded.ch_l_y = (int16_t)channel_raw[3] - (int16_t)DR16_CHANNEL_CENTER;
    decoded.wheel = (int16_t)channel_raw[4] - (int16_t)DR16_CHANNEL_CENTER;
    decoded.sw_r = (DR16_SwitchPosition_t)((frame[5] >> 4U) & 0x03U);
    decoded.sw_l = (DR16_SwitchPosition_t)((frame[5] >> 6U) & 0x03U);
    decoded.mouse_x = DR16_ReadInt16LittleEndian(&frame[6]);
    decoded.mouse_y = DR16_ReadInt16LittleEndian(&frame[8]);
    decoded.mouse_z = DR16_ReadInt16LittleEndian(&frame[10]);
    decoded.mouse_left = frame[12] != 0U;
    decoded.mouse_right = frame[13] != 0U;
    decoded.key_mask = DR16_ReadUint16LittleEndian(&frame[14]);

    // 完整校验所有会影响帧边界判断的字段，拒绝通道越界或拨杆非法的数据
    for (uint32_t channel_index = 0U; channel_index < 5U; channel_index++)
    {
        if (!DR16_IsChannelValid(channel_raw[channel_index]))
        {
            return DEVICE_ERR;
        }
    }
    if (!DR16_IsSwitchValid(decoded.sw_l) || !DR16_IsSwitchValid(decoded.sw_r))
    {
        return DEVICE_ERR;
    }

    // 所有字段合法后一次性提交，向任务层提供一致的协议快照
    *output = decoded;
    return DEVICE_OK;
}

/**
 * @brief 将 DR16 状态恢复为离线零值
 *
 * @param[out] state 待复位的 DR16 状态，允许为 NULL
 * @return 无返回值
 */
void DR16_ResetState(DR16_State_t *state)
{
    if (state != NULL)
    {
        memset(state, 0, sizeof(*state));
    }
}

/**
 * @brief 按小端序读取无符号 16 位字段
 *
 * @param[in] data 两字节字段首地址，调用方保证非空且长度足够
 * @return 解码后的无符号 16 位数值
 */
static uint16_t DR16_ReadUint16LittleEndian(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

/**
 * @brief 按小端序读取有符号 16 位字段
 *
 * @param[in] data 两字节字段首地址，调用方保证非空且长度足够
 * @return 解码后的有符号 16 位数值
 */
static int16_t DR16_ReadInt16LittleEndian(const uint8_t *data)
{
    const uint16_t unsigned_value = DR16_ReadUint16LittleEndian(data);
    if (unsigned_value <= INT16_MAX)
    {
        return (int16_t)unsigned_value;
    }
    return (int16_t)((int32_t)unsigned_value - ((int32_t)UINT16_MAX + 1));
}

/**
 * @brief 检查 DR16 通道原始值是否位于合法范围
 *
 * @param[in] channel 通道原始值
 * @return 合法返回 true，否则返回 false
 */
static bool DR16_IsChannelValid(uint16_t channel)
{
    return channel >= DR16_CHANNEL_MIN && channel <= DR16_CHANNEL_MAX;
}

/**
 * @brief 检查 DR16 拨杆值是否属于三个有效位置
 *
 * @param[in] switch_position 拨杆位置
 * @return 合法返回 true，否则返回 false
 */
static bool DR16_IsSwitchValid(DR16_SwitchPosition_t switch_position)
{
    return switch_position == DR16_SWITCH_UP || switch_position == DR16_SWITCH_DOWN ||
           switch_position == DR16_SWITCH_MIDDLE;
}
