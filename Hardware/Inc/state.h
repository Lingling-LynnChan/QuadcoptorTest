#ifndef __GW_STATE_H__
#define __GW_STATE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// 解锁状态: 兼容ANOTC
typedef enum {
  GWS_FLY_LOCK,   // 锁定
  GWS_FLY_UNLOCK  // 解锁
} GWS_Lock_State_Type;

// 飞行模式: 兼容ANOTC
typedef enum {
  GWS_FLY_MODE_NONE = 0x00,         // 无
  GWS_FLY_MODE_POSE = 0x01,         // 姿态
  GWS_FLY_MODE_FIXED_HIGH = 0x02,   // 定高
  GWS_FLY_MODE_FIXED_POINT = 0x03,  // 定点
  GWS_FLY_MODE_ROUTE = 0x11,        // 航线
  GWS_FLY_MODE_LANDING = 0x20,      // 降落
  GWS_FLY_MODE_RETURN = 0x21        // 返航
} GWS_Fly_Mode_Type;

#ifdef __cplusplus
}
#endif

#endif /*__GW_STATE_H__ */