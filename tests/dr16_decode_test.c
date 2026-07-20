#include "device/dr16.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static void DR16Test_WriteUint16LittleEndian(uint8_t *data, uint16_t value);
static void DR16Test_PackFrame(uint8_t frame[DR16_FRAME_LENGTH], const uint16_t channel[5],
                              DR16_SwitchPosition_t switch_left, DR16_SwitchPosition_t switch_right);
static void DR16Test_DecodesCompleteFrame(void);
static void DR16Test_AcceptsChannelEndpoints(void);
static void DR16Test_RejectsNullPointers(void);
static void DR16Test_RejectsInvalidLengthWithoutChangingOutput(void);
static void DR16Test_RejectsEveryInvalidChannelWithoutChangingOutput(void);
static void DR16Test_RejectsInvalidSwitchWithoutChangingOutput(void);
static void DR16Test_ResetsState(void);

/**
 * @brief 运行 DR16 纯解码主机测试
 *
 * @return 全部断言通过时返回 0
 */
int main(void)
{
    DR16Test_DecodesCompleteFrame();
    DR16Test_AcceptsChannelEndpoints();
    DR16Test_RejectsNullPointers();
    DR16Test_RejectsInvalidLengthWithoutChangingOutput();
    DR16Test_RejectsEveryInvalidChannelWithoutChangingOutput();
    DR16Test_RejectsInvalidSwitchWithoutChangingOutput();
    DR16Test_ResetsState();

    puts("DR16 decode tests passed");
    return 0;
}

/**
 * @brief 按小端序写入无符号 16 位测试字段
 *
 * @param[out] data 两字节字段首地址
 * @param[in] value 待写入数值
 * @return 无返回值
 */
static void DR16Test_WriteUint16LittleEndian(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0x00FFU);
    data[1] = (uint8_t)(value >> 8U);
}

/**
 * @brief 将五个通道和两个拨杆编码为一帧测试数据
 *
 * @param[out] frame 十八字节测试帧
 * @param[in] channel 五个通道原始值
 * @param[in] switch_left 左拨杆位置
 * @param[in] switch_right 右拨杆位置
 * @return 无返回值
 */
static void DR16Test_PackFrame(uint8_t frame[DR16_FRAME_LENGTH], const uint16_t channel[5],
                              DR16_SwitchPosition_t switch_left, DR16_SwitchPosition_t switch_right)
{
    memset(frame, 0, DR16_FRAME_LENGTH);
    frame[0] = (uint8_t)channel[0];
    frame[1] = (uint8_t)((channel[0] >> 8U) | (channel[1] << 3U));
    frame[2] = (uint8_t)((channel[1] >> 5U) | (channel[2] << 6U));
    frame[3] = (uint8_t)(channel[2] >> 2U);
    frame[4] = (uint8_t)((channel[2] >> 10U) | (channel[3] << 1U));
    frame[5] = (uint8_t)((channel[3] >> 7U) | ((uint16_t)switch_right << 4U) |
                         ((uint16_t)switch_left << 6U));
    DR16Test_WriteUint16LittleEndian(&frame[16], channel[4]);
}

/**
 * @brief 验证完整合法帧的全部公开字段
 *
 * @return 无返回值
 */
static void DR16Test_DecodesCompleteFrame(void)
{
    uint8_t frame[DR16_FRAME_LENGTH];
    const uint16_t channel[5] = {1024U, 1124U, 924U, 1684U, 364U};
    DR16Test_PackFrame(frame, channel, DR16_SWITCH_MIDDLE, DR16_SWITCH_UP);
    DR16Test_WriteUint16LittleEndian(&frame[6], (uint16_t)(int16_t)-123);
    DR16Test_WriteUint16LittleEndian(&frame[8], (uint16_t)(int16_t)456);
    DR16Test_WriteUint16LittleEndian(&frame[10], (uint16_t)(int16_t)-789);
    frame[12] = 1U;
    frame[13] = 0U;
    DR16Test_WriteUint16LittleEndian(&frame[14], 0xA55AU);

    DR16_Data_t output = {0};
    assert(DR16_Decode(frame, sizeof(frame), &output) == DEVICE_OK);
    assert(output.ch_r_x == 0);
    assert(output.ch_r_y == 100);
    assert(output.ch_l_x == -100);
    assert(output.ch_l_y == 660);
    assert(output.wheel == -660);
    assert(output.sw_l == DR16_SWITCH_MIDDLE);
    assert(output.sw_r == DR16_SWITCH_UP);
    assert(output.mouse_x == -123);
    assert(output.mouse_y == 456);
    assert(output.mouse_z == -789);
    assert(output.mouse_left);
    assert(!output.mouse_right);
    assert(output.key_mask == 0xA55AU);
}

/**
 * @brief 验证五个通道的合法端点可以通过校验
 *
 * @return 无返回值
 */
static void DR16Test_AcceptsChannelEndpoints(void)
{
    uint8_t frame[DR16_FRAME_LENGTH];
    const uint16_t channel[5] = {364U, 1684U, 364U, 1684U, 1024U};
    DR16Test_PackFrame(frame, channel, DR16_SWITCH_DOWN, DR16_SWITCH_MIDDLE);

    DR16_Data_t output = {0};
    assert(DR16_Decode(frame, sizeof(frame), &output) == DEVICE_OK);
    assert(output.ch_r_x == -660);
    assert(output.ch_r_y == 660);
    assert(output.ch_l_x == -660);
    assert(output.ch_l_y == 660);
    assert(output.wheel == 0);
}

/**
 * @brief 验证空指针会返回对应错误
 *
 * @return 无返回值
 */
static void DR16Test_RejectsNullPointers(void)
{
    uint8_t frame[DR16_FRAME_LENGTH] = {0};
    DR16_Data_t output = {0};

    assert(DR16_Decode(NULL, sizeof(frame), &output) == DEVICE_ERR_NULL);
    assert(DR16_Decode(frame, sizeof(frame), NULL) == DEVICE_ERR_NULL);
}

/**
 * @brief 验证错误长度不会覆盖已有输出
 *
 * @return 无返回值
 */
static void DR16Test_RejectsInvalidLengthWithoutChangingOutput(void)
{
    uint8_t frame[DR16_FRAME_LENGTH] = {0};
    DR16_Data_t output;
    memset(&output, 0x5A, sizeof(output));
    const DR16_Data_t previous = output;

    assert(DR16_Decode(frame, DR16_FRAME_LENGTH - 1U, &output) == DEVICE_ERR);
    assert(memcmp(&output, &previous, sizeof(output)) == 0);
}

/**
 * @brief 验证五个通道的上下越界值都不会覆盖已有输出
 *
 * @return 无返回值
 */
static void DR16Test_RejectsEveryInvalidChannelWithoutChangingOutput(void)
{
    const uint16_t invalid_value[2] = {363U, 1685U};
    for (uint32_t invalid_value_index = 0U; invalid_value_index < 2U; invalid_value_index++)
    {
        for (uint32_t channel_index = 0U; channel_index < 5U; channel_index++)
        {
            uint8_t frame[DR16_FRAME_LENGTH];
            uint16_t channel[5] = {1024U, 1024U, 1024U, 1024U, 1024U};
            channel[channel_index] = invalid_value[invalid_value_index];
            DR16Test_PackFrame(frame, channel, DR16_SWITCH_UP, DR16_SWITCH_DOWN);
            DR16_Data_t output;
            memset(&output, 0x5A, sizeof(output));
            const DR16_Data_t previous = output;

            assert(DR16_Decode(frame, sizeof(frame), &output) == DEVICE_ERR);
            assert(memcmp(&output, &previous, sizeof(output)) == 0);
        }
    }
}

/**
 * @brief 验证非法拨杆不会覆盖已有输出
 *
 * @return 无返回值
 */
static void DR16Test_RejectsInvalidSwitchWithoutChangingOutput(void)
{
    uint8_t frame[DR16_FRAME_LENGTH];
    const uint16_t channel[5] = {1024U, 1024U, 1024U, 1024U, 1024U};
    DR16Test_PackFrame(frame, channel, DR16_SWITCH_ERROR, DR16_SWITCH_UP);
    DR16_Data_t output;
    memset(&output, 0x5A, sizeof(output));
    const DR16_Data_t previous = output;

    assert(DR16_Decode(frame, sizeof(frame), &output) == DEVICE_ERR);
    assert(memcmp(&output, &previous, sizeof(output)) == 0);
}

/**
 * @brief 验证状态复位会生成离线零状态
 *
 * @return 无返回值
 */
static void DR16Test_ResetsState(void)
{
    DR16_State_t state;
    memset(&state, 0x5A, sizeof(state));

    DR16_ResetState(&state);

    const uint8_t zero_state[sizeof(state)] = {0};
    assert(memcmp(&state, zero_state, sizeof(state)) == 0);
    DR16_ResetState(NULL);
}
