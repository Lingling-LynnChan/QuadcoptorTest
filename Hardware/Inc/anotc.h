#ifndef __GW_ANOTC_H__
#define __GW_ANOTC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "pid.h"
#include "state.h"

// 帧类型
typedef enum {
  ANOTC_VER = 0x00,       // 版本
  ANOTC_STATUS = 0x01,    // 飞行器状态
  ANOTC_SENSER = 0X02,    // 姿态传感器数据
  ANOTC_RCDATA = 0x03,    // 遥控数据
  ANOTC_GPSDATA = 0x04,   // GPS数据
  ANOTC_POWER = 0x05,     // 电池数据
  ANOTC_MOTOR = 0x06,     // 电机数据
  ANOTC_SENSER2 = 0x07,   // 高度传感器数据
  ANOTC_RESERD1 = 0x08,   // 保留1
  ANOTC_RESERD2 = 0x09,   // 保留2
  ANOTC_FLY_MODE = 0x0A,  // 飞行模式
  ANOTC_PID1 = 0x10,      // 角速度
  ANOTC_PID2 = 0x11,      // 姿态
  ANOTC_PID3 = 0x12,      // 高度
  ANOTC_PID4 = 0x13,      // 未使用
  ANOTC_PID5 = 0x14,      // 未使用
  ANOTC_PID6 = 0x15,      // 未使用
  ANOTC_CHECK = 0xEF      // 校验
} ANOTC_Msg_Type;

extern int16_t ANOTC_Check_PID;

void ANOTC_Recive(int8_t* pt);
void ANOTC_Send(ANOTC_Msg_Type type);
void ANOTC_Polling(void);

#ifdef __cplusplus
}
#endif

#endif /*__GW_ANOTC_H__ */