#include "state.h"

GW_IMU_Angle_Type GWS_IMU_Angle = {0};
MPU6050_State_Type GWS_MPU6050 = {0};
GWS_Lock_State_Type GWS_Lock_State = GWS_FLY_LOCK;
GWS_Fly_Mode_Type GWS_Fly_Mode = GWS_FLY_MODE_NONE;
