#ifndef __GW_TYPES_H__
#define __GW_TYPES_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
  int16_t X;
  int16_t Y;
  int16_t Z;
} GW_Vector3i;

typedef struct {
  float X;
  float Y;
  float Z;
} GW_Vector3f;

typedef struct {
  float Q0;
  float Q1;
  float Q2;
  float Q3;
} Quaternion;

#ifdef __cplusplus
}
#endif

#endif /*__GW_TYPES_H__ */
