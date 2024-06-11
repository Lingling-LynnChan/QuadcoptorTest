#ifndef __GW_IMU_H__
#define __GW_IMU_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct {
  float Roll;   // 横滚角
  float Pitch;  // 俯仰角
  float Yaw;    // 偏航角
} GW_IMU_Angle_Type;

#ifdef __cplusplus
}
#endif

#endif /*__GW_IMU_H__ */
