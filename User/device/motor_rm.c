#include "motor_rm.h"
#include <stdbool.h>
#include <string.h>
#include "bsp/can.h"
#include "bsp/mm.h"
#include "bsp/time.h"
#include "component/user_math.h"
/*
 * M3508 与 M2006 的反馈 ID 为 0x201~0x208
 * ID 1~4 使用控制帧 0x200，ID 5~8 使用控制帧 0x1FF，每帧包含四个有符号大端电流值
 * @datasheet RoboMaster C620 与 C610 电调用户手册的 CAN 通信协议
 */
#define M3508_M2006_FB_ID_BASE (0x201)
#define M3508_M2006_CTRL_ID_BASE (0x200)
#define M3508_M2006_CTRL_ID_EXTAND (0x1ff)
#define M3508_M2006_ID_SETTING_ID (0x700)
#define M3508_MAX_ABS_LSB (16384)
#define M2006_MAX_ABS_LSB (10000)
#define MOTOR_TX_BUF_SIZE (8)
#define MOTOR_RX_BUF_SIZE (8)
#define MOTOR_RM_FEEDBACK_TIMEOUT_US (100000u)
#define MOTOR_RM_TX_GROUP_M3508_M2006_LOW (0u)
#define MOTOR_RM_TX_GROUP_M3508_M2006_HIGH (1u)
#define MOTOR_RM_TX_GROUP_GM6020 (2u)
#define MOTOR_RM_TX_GROUP_MASK(group) ((uint8_t)(1u << (group)))
#define MOTOR_ENC_RES (8192)  /* 电机编码器分辨率 */
#define MOTOR_CUR_RES (16384) /* 电机转矩电流分辨率 */
#define MOTOR_RM_SUSPICIOUS_DELTA_COUNT ((MOTOR_ENC_RES * 9) / 20)

/*
 * GM6020 电流环兼容协议使用反馈帧 0x209、0x20A 和控制帧 0x2FE
 * ID 5、6 分别位于控制帧数据字节 0~1、2~3，±3 A 对应原始值 ±16384
 * @datasheet RoboMaster GM6020 使用说明 20231013，第 7 页电流控制协议
 */
#define GM6020_FB_ID_BASE (0x209U)
#define GM6020_CTRL_ID (0x2FEU)
#define GM6020_MAX_ABS_CURRENT_A (3.0F)
#define GM6020_OUTPUT_SLOT_BASE (4U)

static MOTOR_RM_CANManager_t *can_managers[BSP_CAN_NUM] =
{
    NULL,
};
/* 最近一次 RM 组帧发送的调试快照 */
static MOTOR_RM_TxDebug_t motor_rm_tx_debug =
{
    0,
};
MOTOR_RM_SlotTxDebug_t g_motor_rm_slot_tx_debug[MOTOR_RM_MAX_MOTORS] =
{
    0,
};

/**
 * @brief 将反馈标识映射为八路输出槽位
 *
 * @param[in] can_id 反馈标准帧标识，M2006/M3508 为 0x201~0x208，GM6020 为 0x209/0x20A
 * @param[in] module 电机型号，支持 M2006、M3508 和 GM6020
 * @return 成功返回 0~7，否则返回 DEVICE_ERR
 */
static int8_t MOTOR_RM_GetLogicalIndex(uint16_t can_id, MOTOR_RM_Module_t module)
{
    switch (module)
    {
    case MOTOR_M2006:
    case MOTOR_M3508:
        if (can_id >= M3508_M2006_FB_ID_BASE && can_id <= M3508_M2006_FB_ID_BASE + 7)
        {
            return can_id - M3508_M2006_FB_ID_BASE;
        }
        break;
    case MOTOR_GM6020:
        if (can_id >= GM6020_FB_ID_BASE && can_id <= GM6020_FB_ID_BASE + 1U)
        {
            return GM6020_OUTPUT_SLOT_BASE + can_id - GM6020_FB_ID_BASE;
        }
        break;
    default:
        break;
    }
    return DEVICE_ERR;
}

/**
 * @brief 选择反馈标识所属的电流控制帧组
 *
 * @param[in] can_id 反馈标准帧标识，支持 0x201~0x208
 * @return 成功返回控制帧组索引，否则返回 DEVICE_ERR
 */
static int8_t MOTOR_RM_GetTxGroupById(uint16_t can_id)
{
    if (can_id < M3508_M2006_FB_ID_BASE || can_id > M3508_M2006_FB_ID_BASE + 7U)
    {
        return DEVICE_ERR;
    }
    return can_id < M3508_M2006_FB_ID_BASE + 4U ? MOTOR_RM_TX_GROUP_M3508_M2006_LOW :
           MOTOR_RM_TX_GROUP_M3508_M2006_HIGH;
}
static int8_t MOTOR_RM_GetTxGroup(MOTOR_RM_Param_t *param)
{
    if (param == NULL)
    {
        return DEVICE_ERR_NULL;
    }
    if (param->module == MOTOR_GM6020)
    {
        return param->id >= GM6020_FB_ID_BASE && param->id <= GM6020_FB_ID_BASE + 1U ?
               (int8_t)MOTOR_RM_TX_GROUP_GM6020 : DEVICE_ERR;
    }
    return MOTOR_RM_GetTxGroupById(param->id);
}
static void MOTOR_RM_MarkPendingGroup(MOTOR_RM_CANManager_t *manager, int8_t group)
{
    if (manager == NULL || group < 0)
    {
        return;
    }
    manager->pending_tx_groups |= MOTOR_RM_TX_GROUP_MASK((uint8_t)group);
}

/**
 * @brief 将缓存电流封装为对应控制帧
 *
 * @param[in] manager 当前 CAN 总线的电机管理器
 * @param[in] group 控制帧组索引，0/1 对应 M3508 或 M2006，2 对应 GM6020
 * @param[out] tx_frame 待发送的八字节标准数据帧
 * @return 成功返回 DEVICE_OK，空指针或无效帧组返回设备错误码
 */
static int8_t MOTOR_RM_FillTxFrame(const MOTOR_RM_CANManager_t *manager, uint8_t group,
                                    BSP_CAN_StdDataFrame_t *tx_frame)
{
    if (manager == NULL || tx_frame == NULL)
    {
        return DEVICE_ERR_NULL;
    }
    // 清零数据区，确保 GM6020 控制帧未使用的字节保持零
    // 再按帧组打包同周期缓存电流
    const MOTOR_RM_MsgOutput_t *output_msg = &manager->output_msg;
    tx_frame->dlc = MOTOR_TX_BUF_SIZE;
    memset(tx_frame->data, 0, sizeof(tx_frame->data));
    switch (group)
    {
    case MOTOR_RM_TX_GROUP_M3508_M2006_LOW:
    case MOTOR_RM_TX_GROUP_M3508_M2006_HIGH:
    {
        const uint8_t output_index_base = group == MOTOR_RM_TX_GROUP_M3508_M2006_LOW ? 0U : 4U;
        tx_frame->id = group == MOTOR_RM_TX_GROUP_M3508_M2006_LOW ? M3508_M2006_CTRL_ID_BASE :
                       M3508_M2006_CTRL_ID_EXTAND;
        for (uint8_t slot = 0U; slot < 4U; slot++)
        {
            const int16_t current = output_msg->output[output_index_base + slot];
            tx_frame->data[slot * 2U] = (uint8_t)((current >> 8) & 0xFF);
            tx_frame->data[slot * 2U + 1U] = (uint8_t)(current & 0xFF);
        }
        return DEVICE_OK;
    }
    case MOTOR_RM_TX_GROUP_GM6020:
        tx_frame->id = GM6020_CTRL_ID;
        for (uint8_t motor_index = 0U; motor_index < 2U; motor_index++)
        {
            const int16_t current = output_msg->output[GM6020_OUTPUT_SLOT_BASE + motor_index];
            tx_frame->data[motor_index * 2U] = (uint8_t)((current >> 8) & 0xFF);
            tx_frame->data[motor_index * 2U + 1U] = (uint8_t)(current & 0xFF);
        }
        return DEVICE_OK;
    default:
        return DEVICE_ERR;
    }
}
static void MOTOR_RM_UpdateTxDebug(const MOTOR_RM_CANManager_t *manager, const BSP_CAN_StdDataFrame_t *tx_frame,
                                   uint16_t source_motor_id, int8_t logical_index)
{
    if (manager == NULL || tx_frame == NULL)
    {
        return;
    }
    motor_rm_tx_debug.valid = 1;
    motor_rm_tx_debug.can = manager->can;
    motor_rm_tx_debug.source_motor_id = source_motor_id;
    motor_rm_tx_debug.tx_frame_id = tx_frame->id;
    motor_rm_tx_debug.logical_index = logical_index;
    if (logical_index >= 0 && logical_index < MOTOR_RM_MAX_MOTORS)
    {
        motor_rm_tx_debug.requested_current = manager->output_msg.output[logical_index];
    }
    else
    {
        motor_rm_tx_debug.requested_current = 0;
    }
    memcpy(motor_rm_tx_debug.grouped_output, manager->output_msg.output, sizeof(motor_rm_tx_debug.grouped_output));
    memcpy(motor_rm_tx_debug.tx_data, tx_frame->data, sizeof(motor_rm_tx_debug.tx_data));
    motor_rm_tx_debug.pending_tx_groups = manager->pending_tx_groups;
    for (int i = 0; i < MOTOR_RM_MAX_MOTORS; i++)
    {
        g_motor_rm_slot_tx_debug[i].valid = 1;
        g_motor_rm_slot_tx_debug[i].can = manager->can;
        g_motor_rm_slot_tx_debug[i].tx_frame_id = tx_frame->id;
        g_motor_rm_slot_tx_debug[i].logical_index = (int8_t)i;
        g_motor_rm_slot_tx_debug[i].output_value = manager->output_msg.output[i];
        memcpy(g_motor_rm_slot_tx_debug[i].tx_data, tx_frame->data, sizeof(g_motor_rm_slot_tx_debug[i].tx_data));
    }
}
static int8_t MOTOR_RM_SendGroup(MOTOR_RM_CANManager_t *manager, uint8_t group, uint16_t source_motor_id,
                                 int8_t logical_index)
{
    if (manager == NULL)
    {
        return DEVICE_ERR_NULL;
    }
    BSP_CAN_StdDataFrame_t tx_frame;
    const int8_t frame_ret = MOTOR_RM_FillTxFrame(manager, group, &tx_frame);
    if (frame_ret != DEVICE_OK)
    {
        return frame_ret;
    }
    MOTOR_RM_UpdateTxDebug(manager, &tx_frame, source_motor_id, logical_index);
    const int8_t ret = BSP_CAN_TransmitStdDataFrame(manager->can, &tx_frame) == BSP_OK ? DEVICE_OK : DEVICE_ERR;
    if (ret == DEVICE_OK)
    {
        manager->pending_tx_groups &= (uint8_t)~MOTOR_RM_TX_GROUP_MASK(group);
        motor_rm_tx_debug.pending_tx_groups = manager->pending_tx_groups;
        motor_rm_tx_debug.flush_count++;
    }
    return ret;
}
static float MOTOR_RM_GetRatio(MOTOR_RM_Module_t module)
{
    switch (module)
    {
    case MOTOR_M2006:
        return 36.0f;
    case MOTOR_M3508:
        return 3591.0f / 187.0f;
    case MOTOR_GM6020:
        return 1.0f;
    default:
        return 1.0f;
    }
}
static int16_t MOTOR_RM_GetLSB(MOTOR_RM_Module_t module)
{
    switch (module)
    {
    case MOTOR_M2006:
        return M2006_MAX_ABS_LSB;
    case MOTOR_M3508:
        return M3508_MAX_ABS_LSB;
    case MOTOR_GM6020:
        return M3508_MAX_ABS_LSB;
    default:
        return DEVICE_ERR;
    }
}
static float MOTOR_RM_GetCurrentRangeAmp(MOTOR_RM_Module_t module)
{
    switch (module)
    {
    case MOTOR_M2006:
        return 10.0f;
    case MOTOR_M3508:
        return 20.0f;
    case MOTOR_GM6020:
        return GM6020_MAX_ABS_CURRENT_A;
    default:
        return 0.0f;
    }
}
static float MOTOR_RM_WrapAnglePositive(float angle)
{
    while (angle < 0.0f)
    {
        angle += M_2PI;
    }
    while (angle >= M_2PI)
    {
        angle -= M_2PI;
    }
    return angle;
}
static int32_t MOTOR_RM_UnwrapRawDelta(uint16_t current_raw_angle, uint16_t last_raw_angle)
{
    int32_t delta = (int32_t)current_raw_angle - (int32_t)last_raw_angle;
    if (delta > (MOTOR_ENC_RES / 2))
    {
        delta -= MOTOR_ENC_RES;
    }
    else if (delta < -(MOTOR_ENC_RES / 2))
    {
        delta += MOTOR_ENC_RES;
    }
    return delta;
}
static void MOTOR_RM_ResetAngleTracker(MOTOR_RM_t *motor, uint16_t raw_angle)
{
    if (motor == NULL)
    {
        return;
    }
    motor->last_raw_angle = raw_angle;
    motor->angle_inited = true;
    motor->gearbox_round_count = 0;
    motor->gearbox_total_raw_count = 0;
    motor->gearbox_total_angle = 0.0f;
}
static MOTOR_RM_CANManager_t *MOTOR_RM_GetCANManager(BSP_CAN_t can)
{
    if (can >= BSP_CAN_NUM)
    {
        return NULL;
    }
    return can_managers[can];
}
static int8_t MOTOR_RM_CreateCANManager(BSP_CAN_t can)
{
    if (can >= BSP_CAN_NUM)
    {
        return DEVICE_ERR;
    }
    if (can_managers[can] != NULL)
    {
        return DEVICE_OK;
    }
    can_managers[can] = (MOTOR_RM_CANManager_t *)BSP_Malloc(sizeof(MOTOR_RM_CANManager_t));
    if (can_managers[can] == NULL)
    {
        return DEVICE_ERR;
    }
    memset(can_managers[can], 0, sizeof(MOTOR_RM_CANManager_t));
    can_managers[can]->can = can;
    return DEVICE_OK;
}

/**
 * @brief 解析反馈并维持现有输出轴角度与速度约定
 *
 * @param[in,out] motor 待更新的已注册电机实例
 * @param[in] msg 本周期八字节反馈帧
 * @return 无返回值
 */
static void Motor_RM_Decode(MOTOR_RM_t *motor, BSP_CAN_Message_t *msg)
{
    // 提取协议原始量，保留底盘已采用的反馈标度
    uint16_t raw_angle = (uint16_t)((msg->data[0] << 8) | msg->data[1]);
    int16_t raw_speed = (int16_t)((msg->data[2] << 8) | msg->data[3]);
    int16_t raw_current = (int16_t)((msg->data[4] << 8) | msg->data[5]);
    int16_t lsb = MOTOR_RM_GetLSB(motor->param.module);
    float ratio = MOTOR_RM_GetRatio(motor->param.module);
    uint64_t now_time = BSP_TIME_Get();
    float rotor_angle = raw_angle / (float)MOTOR_ENC_RES * M_2PI;
    float rotor_speed = raw_speed;
    // GM6020 电流反馈按电流环量程换算为安培，其他型号维持现有反馈标度
    float torque_current;
    if (motor->param.module == MOTOR_GM6020)
    {
        torque_current = raw_current * MOTOR_RM_GetCurrentRangeAmp(motor->param.module) / (float)MOTOR_CUR_RES;
    }
    else
    {
        torque_current = raw_current * lsb / (float)MOTOR_CUR_RES;
    }
    motor->motor.raw_feedback.raw_angle = raw_angle;
    motor->motor.raw_feedback.raw_speed = raw_speed;
    motor->motor.raw_feedback.raw_current = raw_current;
    motor->motor.raw_feedback.raw_temp = msg->data[6];
    motor->motor.raw_feedback.raw_error_code = msg->data[7];
    // 按减速箱配置维护连续角度，再应用安装方向并发布反馈时间
    if (motor->param.gear)
    {
        if (!motor->angle_inited)
        {
            MOTOR_RM_ResetAngleTracker(motor, raw_angle);
        }
        int32_t delta = MOTOR_RM_UnwrapRawDelta(raw_angle, motor->last_raw_angle);
        if (delta >= MOTOR_RM_SUSPICIOUS_DELTA_COUNT || delta <= -MOTOR_RM_SUSPICIOUS_DELTA_COUNT)
        {
            motor->angle_lost_count++;
        }
        motor->gearbox_total_raw_count += delta;
        motor->gearbox_round_count = motor->gearbox_total_raw_count / MOTOR_ENC_RES;
        motor->last_raw_angle = raw_angle;
        motor->gearbox_total_angle = ((float)motor->gearbox_total_raw_count / (float)MOTOR_ENC_RES) * M_2PI / ratio;
        // 输出轴多圈绝对值
        motor->feedback.rotor_total_angle = motor->gearbox_total_angle;
        motor->feedback.rotor_single_angle = MOTOR_RM_WrapAnglePositive(motor->gearbox_total_angle);
        motor->feedback.rotor_abs_angle = motor->feedback.rotor_single_angle;
        motor->feedback.rotor_speed = rotor_speed / ratio;
        motor->feedback.torque_current = torque_current * ratio;
    }
    else
    {
        // 非gear模式，直接用转子单圈
        MOTOR_RM_ResetAngleTracker(motor, raw_angle);
        motor->feedback.rotor_abs_angle = rotor_angle;
        motor->feedback.rotor_total_angle = rotor_angle;
        motor->feedback.rotor_single_angle = rotor_angle;
        motor->feedback.rotor_speed = rotor_speed;
        motor->feedback.torque_current = torque_current;
    }
    motor->feedback.angle_valid = (motor->angle_lost_count == 0U);
    if (motor->motor.reverse)
    {
        motor->feedback.rotor_total_angle = -motor->feedback.rotor_total_angle;
        motor->feedback.rotor_single_angle = MOTOR_RM_WrapAnglePositive(-motor->feedback.rotor_single_angle);
        motor->feedback.rotor_abs_angle = motor->feedback.rotor_single_angle;
        motor->feedback.rotor_speed = -motor->feedback.rotor_speed;
        motor->feedback.torque_current = -motor->feedback.torque_current;
    }
    motor->feedback.temp = msg->data[6];
    motor->feedback.angle_lost_count = motor->angle_lost_count;
    motor->feedback.last_update_time = (uint32_t)now_time;
    motor->last_feedback_time = (uint32_t)now_time;
}
static MOTOR_RM_t *MOTOR_RM_FindMotorById(const MOTOR_RM_CANManager_t *manager, uint16_t id)
{
    if (manager == NULL)
    {
        return NULL;
    }
    const uint8_t motor_total = manager->motor_count + manager->external_motor_count;
    for (uint8_t i = 0U; i < motor_total; i++)
    {
        MOTOR_RM_t *motor = i < manager->motor_count ? manager->motors[i] :
                            manager->external_motors[i - manager->motor_count];
        if (motor != NULL && motor->param.id == id)
        {
            return motor;
        }
    }
    return NULL;
}
static MOTOR_RM_Param_t *MOTOR_RM_FindParamByTxGroup(MOTOR_RM_CANManager_t *manager, uint8_t group)
{
    if (manager == NULL)
    {
        return NULL;
    }
    const uint8_t motor_total = manager->motor_count + manager->external_motor_count;
    for (uint8_t i = 0U; i < motor_total; i++)
    {
        MOTOR_RM_t *motor = i < manager->motor_count ? manager->motors[i] :
                            manager->external_motors[i - manager->motor_count];
        if (motor != NULL && MOTOR_RM_GetTxGroup(&motor->param) == (int8_t)group)
        {
            return &motor->param;
        }
    }
    return NULL;
}
/**
 * @brief 将 RM 电机实例绑定到对应的管理器实例数组
 *
 * @param[in] manager 当前 CAN 总线的电机管理器
 * @param[in] param 待注册电机的 CAN、ID 和型号参数
 * @param[out] motor 待初始化并绑定的电机实例
 * @param[in] external 是否由外部对象持有该实例
 * @return 成功返回 DEVICE_OK，否则返回设备错误码
 */
static int8_t MOTOR_RM_BindMotor(MOTOR_RM_CANManager_t *manager, MOTOR_RM_Param_t *param,
                                  MOTOR_RM_t *motor, bool external)
{
    if (manager == NULL || param == NULL || motor == NULL)
    {
        return DEVICE_ERR_NULL;
    }
    if (MOTOR_RM_FindMotorById(manager, param->id) != NULL)
    {
        return DEVICE_ERR_INITED;
    }
    uint8_t *motor_count = external ? &manager->external_motor_count : &manager->motor_count;
    MOTOR_RM_t **motors = external ? manager->external_motors : manager->motors;
    if (*motor_count >= MOTOR_RM_MAX_MOTORS)
    {
        return DEVICE_ERR;
    }
    memset(motor, 0, sizeof(MOTOR_RM_t));
    memcpy(&motor->param, param, sizeof(MOTOR_RM_Param_t));
    motor->motor.reverse = param->reverse;
    if (BSP_CAN_RegisterLatestId(param->can, param->id) != BSP_OK)
    {
        return DEVICE_ERR;
    }
    motors[*motor_count] = motor;
    (*motor_count)++;
    return DEVICE_OK;
}
int8_t MOTOR_RM_Register(MOTOR_RM_Param_t *param)
{
    if (param == NULL)
    {
        return DEVICE_ERR_NULL;
    }
    if (MOTOR_RM_CreateCANManager(param->can) != DEVICE_OK)
    {
        return DEVICE_ERR;
    }
    MOTOR_RM_CANManager_t *manager = MOTOR_RM_GetCANManager(param->can);
    if (manager == NULL)
    {
        return DEVICE_ERR;
    }
    MOTOR_RM_t *new_motor = (MOTOR_RM_t *)BSP_Malloc(sizeof(MOTOR_RM_t));
    if (new_motor == NULL)
    {
        return DEVICE_ERR;
    }
    const int8_t ret = MOTOR_RM_BindMotor(manager, param, new_motor, false);
    if (ret != DEVICE_OK)
    {
        BSP_Free(new_motor);
        return ret;
    }
    return DEVICE_OK;
}
int8_t MOTOR_RM_Update(MOTOR_RM_Param_t *param)
{
    if (param == NULL)
    {
        return DEVICE_ERR_NULL;
    }
    MOTOR_RM_CANManager_t *manager = MOTOR_RM_GetCANManager(param->can);
    if (manager == NULL)
    {
        return DEVICE_ERR_NO_DEV;
    }
    MOTOR_RM_t *motor = MOTOR_RM_FindMotorById(manager, param->id);
    if (motor == NULL)
    {
        return DEVICE_ERR_NO_DEV;
    }
    BSP_CAN_Message_t rx_msg;
    if (BSP_CAN_GetLatestMessage(param->can, param->id, &rx_msg) != BSP_OK)
    {
        uint64_t now_time = BSP_TIME_Get();
        if (now_time - motor->motor.header.last_online_time > MOTOR_RM_FEEDBACK_TIMEOUT_US)
        {
            motor->motor.header.online = false;
            return DEVICE_ERR_NO_DEV;
        }
        return DEVICE_ERR;
    }
    // 只解析完整的标准数据帧，拒绝远程帧、扩展帧与截断反馈
    if (rx_msg.frame_type != BSP_CAN_FRAME_STD_DATA || rx_msg.dlc != MOTOR_RX_BUF_SIZE)
    {
        return DEVICE_ERR;
    }
    motor->motor.header.online = true;
    motor->motor.header.last_online_time = BSP_TIME_Get();
    Motor_RM_Decode(motor, &rx_msg);
    motor->motor.feedback = motor->feedback;
    return DEVICE_OK;
}
int8_t MOTOR_RM_UpdateAll(void)
{
    int8_t ret = DEVICE_OK;
    for (int can = 0; can < BSP_CAN_NUM; can++)
    {
        MOTOR_RM_CANManager_t *manager = MOTOR_RM_GetCANManager((BSP_CAN_t)can);
        if (manager == NULL)
        {
            continue;
        }
        const uint8_t motor_total = manager->motor_count + manager->external_motor_count;
        for (uint8_t i = 0U; i < motor_total; i++)
        {
            MOTOR_RM_t *motor = i < manager->motor_count ? manager->motors[i] :
                                manager->external_motors[i - manager->motor_count];
            if (motor != NULL && MOTOR_RM_Update(&motor->param) != DEVICE_OK)
            {
                ret = DEVICE_ERR;
            }
        }
    }
    return ret;
}
/* 原生接口保留归一化输出、组帧发送与状态控制 */
/*
 * 底层兼容接口：输入为归一化输出值 [-1, 1]。
 * 主要保留给旧代码或直接比例输出场景使用。
 */
int8_t MOTOR_RM_SetOutput(MOTOR_RM_Param_t *param, float value)
{
    if (param == NULL)
    {
        return DEVICE_ERR_NULL;
    }
    MOTOR_RM_CANManager_t *manager = MOTOR_RM_GetCANManager(param->can);
    if (manager == NULL)
    {
        return DEVICE_ERR_NO_DEV;
    }
    if (value > 1.0f)
    {
        value = 1.0f;
    }
    if (value < -1.0f)
    {
        value = -1.0f;
    }
    if (param->reverse)
    {
        value = -value;
    }
    MOTOR_RM_t *motor = MOTOR_RM_GetMotor(param);
    if (motor == NULL)
    {
        return DEVICE_ERR_NO_DEV;
    }
    int8_t logical_index = MOTOR_RM_GetLogicalIndex(param->id, param->module);
    if (logical_index < 0)
    {
        return DEVICE_ERR;
    }
    int16_t output_value = (int16_t)(value * (float)MOTOR_RM_GetLSB(param->module));
    manager->output_msg.output[logical_index] = output_value;
    MOTOR_RM_MarkPendingGroup(manager, MOTOR_RM_GetTxGroup(param));
    return DEVICE_OK;
}
int8_t MOTOR_RM_Ctrl(MOTOR_RM_Param_t *param)
{
    if (param == NULL)
    {
        return DEVICE_ERR_NULL;
    }
    MOTOR_RM_CANManager_t *manager = MOTOR_RM_GetCANManager(param->can);
    if (manager == NULL)
    {
        return DEVICE_ERR_NO_DEV;
    }
    int8_t logical_index = MOTOR_RM_GetLogicalIndex(param->id, param->module);
    const int8_t group = MOTOR_RM_GetTxGroup(param);
    if (group < 0)
    {
        return DEVICE_ERR;
    }
    return MOTOR_RM_SendGroup(manager, (uint8_t)group, param->id, logical_index);
}
int8_t MOTOR_RM_FlushGroup(MOTOR_RM_Param_t *param)
{
    if (param == NULL)
    {
        return DEVICE_ERR_NULL;
    }
    MOTOR_RM_CANManager_t *manager = MOTOR_RM_GetCANManager(param->can);
    if (manager == NULL)
    {
        return DEVICE_ERR_NO_DEV;
    }
    const int8_t group = MOTOR_RM_GetTxGroup(param);
    if (group < 0)
    {
        return DEVICE_ERR;
    }
    if ((manager->pending_tx_groups & MOTOR_RM_TX_GROUP_MASK((uint8_t)group)) == 0u)
    {
        return DEVICE_OK;
    }
    return MOTOR_RM_Ctrl(param);
}

/**
 * @brief 发送指定 CAN 上待提交的电流控制帧
 *
 * @param[in] can CAN 总线编号
 * @return 全部待发送帧提交成功返回 DEVICE_OK，否则返回设备错误码
 */
int8_t MOTOR_RM_FlushCAN(BSP_CAN_t can)
{
    MOTOR_RM_CANManager_t *manager = MOTOR_RM_GetCANManager(can);
    if (manager == NULL)
    {
        return DEVICE_ERR_NO_DEV;
    }
    int8_t ret = DEVICE_OK;
    // 只提交已写入缓存的电机帧组，失败状态留给调用方处理
    const uint8_t pending = manager->pending_tx_groups;
    for (uint8_t group = MOTOR_RM_TX_GROUP_M3508_M2006_LOW; group <= MOTOR_RM_TX_GROUP_GM6020; group++)
    {
        if ((pending & MOTOR_RM_TX_GROUP_MASK(group)) == 0u)
        {
            continue;
        }
        MOTOR_RM_Param_t *param = MOTOR_RM_FindParamByTxGroup(manager, group);
        if (param == NULL || MOTOR_RM_Ctrl(param) != DEVICE_OK)
        {
            ret = DEVICE_ERR;
        }
    }
    return ret;
}
int8_t MOTOR_RM_FlushAll(void)
{
    int8_t ret = DEVICE_OK;
    for (int can = 0; can < BSP_CAN_NUM; can++)
    {
        MOTOR_RM_CANManager_t *manager = MOTOR_RM_GetCANManager((BSP_CAN_t)can);
        if (manager == NULL || manager->pending_tx_groups == 0u)
        {
            continue;
        }
        if (MOTOR_RM_FlushCAN((BSP_CAN_t)can) != DEVICE_OK)
        {
            ret = DEVICE_ERR;
        }
    }
    return ret;
}

MOTOR_RM_t *MOTOR_RM_GetMotor(MOTOR_RM_Param_t *param)
{
    if (param == NULL)
    {
        return NULL;
    }
    MOTOR_RM_CANManager_t *manager = MOTOR_RM_GetCANManager(param->can);
    if (manager == NULL)
    {
        return NULL;
    }
    return MOTOR_RM_FindMotorById(manager, param->id);
}

int8_t MOTOR_RM_Relax(MOTOR_RM_Param_t *param)
{
    return MOTOR_RM_SetOutput(param, 0.0f);
}

int8_t MOTOR_RM_Offine(MOTOR_RM_Param_t *param)
{
    MOTOR_RM_t *motor = MOTOR_RM_GetMotor(param);
    if (motor)
    {
        motor->motor.header.online = false;
        return DEVICE_OK;
    }
    return DEVICE_ERR_NO_DEV;
}

/* 外部实例适配接口保留物理量命令、发送缓存与调试能力 */

/* C++ 驱动层将外部持有的 vendor instance 绑定到 C 管理器。 */
int8_t MOTOR_RM_AttachExternal(MOTOR_RM_Param_t *param, MOTOR_RM_t *external_motor)
{
    if (param == NULL || external_motor == NULL)
    {
        return DEVICE_ERR_NULL;
    }
    if (MOTOR_RM_CreateCANManager(param->can) != DEVICE_OK)
    {
        return DEVICE_ERR;
    }
    MOTOR_RM_CANManager_t *manager = MOTOR_RM_GetCANManager(param->can);
    if (manager == NULL)
    {
        return DEVICE_ERR;
    }
    return MOTOR_RM_BindMotor(manager, param, external_motor, true);
}

/* C++ RM 驱动的主下发接口：输入为转子侧目标电流，单位 A。 */
int8_t MOTOR_RM_SetTorqueCurrent(MOTOR_RM_Param_t *param, float current_a)
{
    if (param == NULL)
    {
        return DEVICE_ERR_NULL;
    }
    const float lsb = (float)MOTOR_RM_GetLSB(param->module);
    const float current_range_a = MOTOR_RM_GetCurrentRangeAmp(param->module);
    if (lsb <= 0.0f || current_range_a <= 0.0f)
    {
        return DEVICE_ERR;
    }
    const float raw_current = current_a * lsb / current_range_a;
    const float normalized = raw_current / lsb;
    return MOTOR_RM_SetOutput(param, normalized);
}

/* C++ 驱动层读取底层原始反馈缓存。 */
const MOTOR_RM_RawFeedback_t *MOTOR_RM_GetRawFeedback(MOTOR_RM_Param_t *param)
{
    MOTOR_RM_t *motor = MOTOR_RM_GetMotor(param);
    if (motor == NULL)
    {
        return NULL;
    }
    return &motor->motor.raw_feedback;
}

const MOTOR_RM_TxDebug_t *MOTOR_RM_GetTxDebug(void)
{
    return &motor_rm_tx_debug;
}

const MOTOR_RM_SlotTxDebug_t *MOTOR_RM_GetSlotTxDebug(uint8_t logical_index)
{
    if (logical_index >= MOTOR_RM_MAX_MOTORS)
    {
        return NULL;
    }
    return &g_motor_rm_slot_tx_debug[logical_index];
}
