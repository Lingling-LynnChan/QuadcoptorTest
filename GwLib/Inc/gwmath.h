#ifndef __GW_MATH_H__
#define __GW_MATH_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define GWM_PI 3.1415926535f
#define GWM_Square(N) (((float)N) * ((float)N))
#define GWM_Abs(N) (((N) > 0) ? (N) : (-(N)))
#define GWM_Min(A, B) (((A) < (B)) ? (A) : (B))
#define GWM_Max(A, B) (((A) > (B)) ? (A) : (B))
#define GWM_Limit(N, MIN, MAX) \
  (((N) < (MIN)) ? (MIN) : (((N) > (MAX)) ? (MAX) : (N)))

float GWM_Arcsin(float x);
float GWM_Arctan(float x);
float GWM_Arctan2(float y, float x);
float GWM_Sin(float x);
float GWM_Cos(float x);
float GWM_RSqrt(float x);

#ifdef __cplusplus
}
#endif

#endif /*__GW_MATH_H__ */