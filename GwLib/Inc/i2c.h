#ifndef __GW_I2C_H__
#define __GW_I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"

// 初始化I2C
void GW_I2C_Init(void);
// 向某地址写入数据
HAL_StatusTypeDef GW_I2C_Write_Byte(uint8_t daddr, uint8_t reg, uint8_t data);
HAL_StatusTypeDef GW_I2C_Write_Data(uint8_t daddr,
                                    uint8_t reg,
                                    uint8_t* buf,
                                    uint8_t len);
// 从某地址读取数据
uint8_t GW_I2C_Read_Byte(uint8_t daddr, uint8_t reg);
HAL_StatusTypeDef GW_I2C_Read_Data(uint8_t daddr,
                                   uint8_t reg,
                                   uint8_t* buf,
                                   uint8_t len);
#ifdef __cplusplus
}
#endif

#endif /*__GW_I2C_H__ */