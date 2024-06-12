#ifndef __GW_IMU_H__
#define __GW_IMU_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "state.h"

typedef struct {
  float Roll;   // 横滚角
  float Pitch;  // 俯仰角
  float Yaw;    // 偏航角
} GW_IMU_Angle_Type;

// 姿态角
extern GW_IMU_Angle_Type GWS_IMU_Angle;

extern const float IMU_RTA;
extern const float IMU_ATR;
extern const float IMU_GYRO_G;
extern const float IMU_GYRO_GR;

void IMU_Get_Angle(float dt);
float IMU_Get_Norm_AccelZ(void);

#ifdef __cplusplus
}
#endif

#endif /*__GW_IMU_H__ */
