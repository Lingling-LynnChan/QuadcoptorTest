#ifndef __GW_ANOTC_H__
#define __GW_ANOTC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "pid.h"

// 帧类型
typedef enum {
  ANOTC_VER = 0x00,
  ANOTC_STATUS = 0x01,
  ANOTC_MPU_MAGIC = 0X02,
  ANOTC_RCDATA = 0x03,
  ANOTC_GPSDATA = 0x04,
  ANOTC_POWER = 0x05,
  ANOTC_MOTOR = 0x06,
  ANOTC_SENSER2 = 0x07,
  ANOTC_RESERD1 = 0x08,
  ANOTC_RESERD2 = 0x09,
  ANOTC_FLY_MODE = 0x0A,
  ANOTC_RATE_PID = 0x10,
  ANOTC_ANGLE_PID = 0x11,
  ANOTC_HEIGHT_PID = 0x12,
  ANOTC_PID4 = 0x13,
  ANOTC_PID5 = 0x14,
  ANOTC_PID6 = 0x15,
  ANOTC_CHECK = 0xEF
} ANOTC_Type;

extern int16_t ANOTC_Check_PID;

void ANOTC_Recive(int8_t* pt);
void ANOTC_Send(ANOTC_Type type);
void ANOTC_Polling(void);

#ifdef __cplusplus
}
#endif

#endif /*__GW_ANOTC_H__ */