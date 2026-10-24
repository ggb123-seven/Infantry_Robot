#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "module/gimbal.h"

/**
 * @brief 获取整机云台默认配置
 *
 * @return 指向只读云台配置的指针，生命周期为整个程序运行期
 */
const Gimbal_Config_t *Config_GetGimbal(void);

#ifdef __cplusplus
}
#endif
