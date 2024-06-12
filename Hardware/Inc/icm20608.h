#ifndef __ICM20608_H__
#define __ICM20608_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "gwtypes.h"
#include "kalman.h"
#include "main.h"
#include "state.h"

HAL_StatusTypeDef ICM20608_Init(void);
HAL_StatusTypeDef ICM20608_Read(float* pitch, float* roll, float* yaw);

#ifdef __cplusplus
}
#endif

#endif /*__ICM20608_H__ */