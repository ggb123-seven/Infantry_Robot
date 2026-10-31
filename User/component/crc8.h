/*
 * 移植自参考工程的 CRC8 组件，原实现参考 Linux
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// 沿用参考工程的默认初值 0xFF，取值范围为一个无符号字节
#define CRC8_INIT 0xFFU

/**
 * @brief 计算字节流的 CRC8，支持分段累积
 *
 * @param[in] buf 待计算的字节流，调用方保证非空且至少包含 len 个可读字节
 * @param[in] len 待计算的字节数，不包含尾部校验值，零长度时返回输入初值
 * @param[in] crc 首段传入协议初值，后续段传入上一段结果
 * @return 计算得到的 CRC8
 */
uint8_t CRC8_Calc(const uint8_t *buf, size_t len, uint8_t crc);

/**
 * @brief 按默认初值校验尾部包含一个 CRC8 字节的完整帧
 *
 * @param[in] buf 完整帧，调用方保证非空且至少包含 len 个可读字节
 * @param[in] len 完整帧字节数，沿用参考实现至少两个字节的约束
 * @return 校验一致返回 true，帧过短或校验不一致返回 false
 */
bool CRC8_Verify(const uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif
