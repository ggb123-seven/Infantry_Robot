#include "device/dr16.h"

#include "ringbuffer/ringbuffer.h"

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

/*
 * DR16 接收器私有状态：
 * - ring_buffer、ring_buffer_storage：单一 ISR 生产者与任务消费者共享的协议字节流
 * - state：最近一次完整合法帧形成的稳定业务状态
 * - initialized：协议字节流缓冲区可用时为 true
 * - diagnostics：帧校验、重同步和缓冲区溢出的累计诊断数据
 */
typedef struct
{
    lwrb_t ring_buffer;
    uint8_t ring_buffer_storage[DR16_RECEIVER_BUFFER_SIZE];
    DR16_State_t state;
    volatile bool initialized;
    uint32_t valid_frame_count;
    uint32_t invalid_frame_count;
    uint32_t resync_discarded_byte_count;
    volatile uint32_t ring_buffer_overflow_count;
} DR16_Receiver_t;

static DR16_Receiver_t dr16_receiver;

static uint16_t DR16_ReadUint16LittleEndian(const uint8_t *data);
static int16_t DR16_ReadInt16LittleEndian(const uint8_t *data);
static bool DR16_IsChannelValid(uint16_t channel);
static bool DR16_IsSwitchValid(DR16_SwitchPosition_t switch_position);
static bool DR16_ReceiverProcessFrames(uint64_t now_us);
static bool DR16_ReceiverCheckOffline(uint64_t now_us);

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
 * @brief 初始化 DR16 协议字节流接收器
 *
 * @return 初始化成功返回 DR16_RECEIVER_OK，否则返回对应接收器状态
 */
int8_t DR16_ReceiverInit(void)
{
    // 先恢复全部私有状态，再建立单生产者单消费者协议字节流
    memset(&dr16_receiver, 0, sizeof(dr16_receiver));
    if (lwrb_init(&dr16_receiver.ring_buffer, dr16_receiver.ring_buffer_storage,
                  sizeof(dr16_receiver.ring_buffer_storage)) == 0U)
    {
        return DR16_RECEIVER_ERROR;
    }

    dr16_receiver.initialized = true;
    return DR16_RECEIVER_OK;
}

/**
 * @brief 将 UART 新增字节写入 DR16 协议字节流
 *
 * 本函数供单一 UART 中断生产者调用，只复制字节并记录溢出状态
 *
 * @param[in] data 本次新增字节段首地址
 * @param[in] length 本次新增字节数
 * @return 全部写入返回 DR16_RECEIVER_OK，否则返回对应接收器状态
 */
int8_t DR16_ReceiverFeed(const uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0U)
    {
        return DR16_RECEIVER_NULL_ERROR;
    }
    if (!dr16_receiver.initialized)
    {
        return DR16_RECEIVER_NOT_INITIALIZED;
    }

    // 单次写入不完整时保留溢出证据，由任务停止 UART 后统一复位未验证字节
    if (lwrb_write(&dr16_receiver.ring_buffer, data, length) != length)
    {
        dr16_receiver.ring_buffer_overflow_count++;
        return DR16_RECEIVER_OVERFLOW;
    }
    return DR16_RECEIVER_OK;
}

/**
 * @brief 解码全部候选帧并更新 DR16 在线状态
 *
 * @param[in] now_us 当前时间，单位微秒
 * @return 业务状态发生变化时返回 true，否则返回 false
 */
bool DR16_ReceiverUpdate(uint64_t now_us)
{
    if (!dr16_receiver.initialized)
    {
        return false;
    }

    // 先提交本轮全部完整合法帧，再判断最后合法帧是否已经超时
    const bool frame_updated = DR16_ReceiverProcessFrames(now_us);
    const bool offline_updated = DR16_ReceiverCheckOffline(now_us);
    return frame_updated || offline_updated;
}

/**
 * @brief 强制 DR16 切换为离线安全状态
 *
 * @return 无返回值
 */
void DR16_ReceiverSetOffline(void)
{
    dr16_receiver.state.header.online = false;
    memset(&dr16_receiver.state.data, 0, sizeof(dr16_receiver.state.data));
}

/**
 * @brief 清空 DR16 协议字节流中的未验证数据
 *
 * 调用方必须先停止 UART 字节交付，避免生产者与消费者并发复位
 *
 * @return 复位成功返回 DR16_RECEIVER_OK，否则返回对应接收器状态
 */
int8_t DR16_ReceiverResetStream(void)
{
    if (!dr16_receiver.initialized)
    {
        return DR16_RECEIVER_NOT_INITIALIZED;
    }

    lwrb_reset(&dr16_receiver.ring_buffer);
    return DR16_RECEIVER_OK;
}

/**
 * @brief 获取 DR16 一致状态快照
 *
 * @param[out] state 待写入的 DR16 状态快照
 * @return 成功返回 DR16_RECEIVER_OK，否则返回对应接收器状态
 */
int8_t DR16_ReceiverGetState(DR16_State_t *state)
{
    if (state == NULL)
    {
        return DR16_RECEIVER_NULL_ERROR;
    }
    if (!dr16_receiver.initialized)
    {
        DR16_ResetState(state);
        return DR16_RECEIVER_NOT_INITIALIZED;
    }

    *state = dr16_receiver.state;
    return DR16_RECEIVER_OK;
}

/**
 * @brief 获取 DR16 接收器诊断快照
 *
 * @param[out] diagnostics 待写入的接收器诊断快照
 * @return 成功返回 DR16_RECEIVER_OK，否则返回对应接收器状态
 */
int8_t DR16_ReceiverGetDiagnostics(DR16_ReceiverDiagnostics_t *diagnostics)
{
    if (diagnostics == NULL)
    {
        return DR16_RECEIVER_NULL_ERROR;
    }

    *diagnostics = (DR16_ReceiverDiagnostics_t)
    {
        .initialized = dr16_receiver.initialized,
        .valid_frame_count = dr16_receiver.valid_frame_count,
        .invalid_frame_count = dr16_receiver.invalid_frame_count,
        .resync_discarded_byte_count = dr16_receiver.resync_discarded_byte_count,
        .ring_buffer_overflow_count = dr16_receiver.ring_buffer_overflow_count,
    };
    return dr16_receiver.initialized ? DR16_RECEIVER_OK : DR16_RECEIVER_NOT_INITIALIZED;
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

/**
 * @brief 从协议字节流中提交全部完整合法帧
 *
 * @param[in] now_us 当前时间，单位微秒
 * @return 至少提交一帧完整合法数据时返回 true，否则返回 false
 */
static bool DR16_ReceiverProcessFrames(uint64_t now_us)
{
    bool state_changed = false;
    uint8_t frame[DR16_FRAME_LENGTH];
    while (lwrb_get_full(&dr16_receiver.ring_buffer) >= DR16_FRAME_LENGTH)
    {
        // 查看候选帧但暂不移除，保证非法帧只丢弃一个字节后继续搜索边界
        if (lwrb_peek(&dr16_receiver.ring_buffer, 0U, frame, sizeof(frame)) != sizeof(frame))
        {
            break;
        }

        DR16_Data_t decoded;
        if (DR16_Decode(frame, sizeof(frame), &decoded) == DEVICE_OK)
        {
            // 完整合法帧一次性提交，并按整帧长度推进读取位置
            lwrb_skip(&dr16_receiver.ring_buffer, DR16_FRAME_LENGTH);
            dr16_receiver.state.header.online = true;
            dr16_receiver.state.header.last_online_time = now_us;
            dr16_receiver.state.valid_frame_sequence++;
            dr16_receiver.state.data = decoded;
            dr16_receiver.valid_frame_count++;
            state_changed = true;
        }
        else
        {
            // 候选帧非法时只丢弃一个字节，以便从任意错位位置重新获得帧边界
            lwrb_skip(&dr16_receiver.ring_buffer, 1U);
            dr16_receiver.invalid_frame_count++;
            dr16_receiver.resync_discarded_byte_count++;
        }
    }
    return state_changed;
}

/**
 * @brief 检查连续无合法帧时间并切换为离线安全状态
 *
 * @param[in] now_us 当前时间，单位微秒
 * @return 本轮发生在线到离线切换时返回 true，否则返回 false
 */
static bool DR16_ReceiverCheckOffline(uint64_t now_us)
{
    if (!dr16_receiver.state.header.online || now_us < dr16_receiver.state.header.last_online_time)
    {
        return false;
    }
    if (now_us - dr16_receiver.state.header.last_online_time < DR16_RECEIVER_OFFLINE_TIMEOUT_US)
    {
        return false;
    }

    DR16_ReceiverSetOffline();
    return true;
}
