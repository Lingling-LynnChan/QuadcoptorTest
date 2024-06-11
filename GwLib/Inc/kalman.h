#ifndef __KALMAN_H__
#define __KALMAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct {
  float Last_P;
  float Now_P;
  float Out;
  float Kg;
  float Q;
  float R;
} GW_Kalman_State_Type;

void GW_Kalman_V1(GW_Kalman_State_Type* kalman, float in);

#ifdef __cplusplus
}
#endif

#endif /*__KALMAN_H__ */