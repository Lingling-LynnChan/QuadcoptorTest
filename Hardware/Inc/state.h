#ifndef __GW_STATE_H__
#define __GW_STATE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "icm20608.h"
#include "imu.h"
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

// 六轴传感器
extern MPU6050_State_Type GWS_MPU6050;
// 姿态角
extern GW_IMU_Angle_Type GWS_IMU_Angle;
// 飞行器锁状态
extern GWS_Lock_State_Type GWS_Lock_State;
// 飞行模式
extern GWS_Fly_Mode_Type GWS_Fly_Mode;

#ifdef __cplusplus
}
#endif

#endif /*__GW_STATE_H__ */