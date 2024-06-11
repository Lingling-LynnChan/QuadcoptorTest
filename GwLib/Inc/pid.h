#ifndef __GW_PID_H__
#define __GW_PID_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct {
  float desired;
  float Offset;
  float Previous_Error;
  float Integral;
  float P;
  float I;
  float D;
  float Integral_Limit_High;
  float Integral_Limit_Low;
  float Measured;
  float Out;
  float Out_Limit_High;
  float Out_Limit_Low;
} GW_PID_State_Type;

#ifdef __cplusplus
}
#endif

#endif /*__GW_PID_H__ */