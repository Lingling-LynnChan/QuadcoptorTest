#ifndef __KALMAN_H__
#define __KALMAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct {
  float Last_P;  // 上次的P值
  float Now_P;   // 当前的P值
  float Out;     // 卡尔曼滤波后的值
  float Kg;      // 卡尔曼增益
  float Q;       // 测量'过程'噪声偏差的方差
  float R;       // 测量'值'噪声偏差的方差
} GW_Kalman_State_Type;

void GW_Kalman_V1(GW_Kalman_State_Type* kalman, float in);

#ifdef __cplusplus
}
#endif

#endif /*__KALMAN_H__ */