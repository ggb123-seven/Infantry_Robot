#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>

#include "user_math.h"

// 沿用参考工程的默认初值 0xFFFF，取值范围为一个无符号十六位整数
#define CRC16_INIT 0xFFFFU

/**
 * @brief 计算字节流的 CRC16，支持分段累积
 *
 * @param[in] buf 待计算的字节流，调用方保证非空且至少包含 len 个可读字节
 * @param[in] len 待计算的字节数，不包含尾部校验值，零长度时返回输入初值
 * @param[in] crc 首段传入协议初值，后续段传入上一段结果
 * @return 计算得到的 CRC16
 */
uint16_t CRC16_Calc(const uint8_t *buf, size_t len, uint16_t crc);

/**
 * @brief 按默认初值校验尾部包含两个 CRC16 字节的完整帧
 *
 * @param[in] buf 完整帧，尾部低字节在前，调用方保证非空且至少包含 len 个可读字节
 * @param[in] len 完整帧字节数，至少两个字节，可校验仅含校验值的空负载帧
 * @return 校验一致返回 true，帧过短或校验不一致返回 false
 */
bool CRC16_Verify(const uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif
