#ifndef __GW_PID_H__
#define __GW_PID_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "state.h"

typedef struct {
  float Desired;
  float Offset;
  float Previous_Error;
  float Integral;
  float P;  // P的参数
  float I;  // I的参数
  float D;  // D的参数
  float Integral_Limit_High;
  float Integral_Limit_Low;
  float Measured;
  float Out;
  float Out_Limit_High;
  float Out_Limit_Low;
} GW_PID_Type;

typedef struct {
  // 内环PID X轴角速度
  GW_PID_Type RateX;
  // 内环PID Y轴角速度
  GW_PID_Type RateY;
  // 内环PID Z轴角速度
  GW_PID_Type RateZ;
  // 外环PID 俯仰角
  GW_PID_Type Pitch;
  // 外环PID 横滚角
  GW_PID_Type Roll;
  // 外环PID 偏航角
  GW_PID_Type Yaw;
} GW_PID_State_Type;

// PID操作
extern GW_PID_State_Type GWS_PID;

void GW_PID_Param_Init(void);
void GW_PID_Reset(void);
void GW_PID_Update(GW_PID_Type* pid, float dt);

#ifdef __cplusplus
}
#endif

#endif /*__GW_PID_H__ */