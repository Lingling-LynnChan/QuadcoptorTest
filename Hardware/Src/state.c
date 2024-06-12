#include "state.h"

GWS_Lock_State_Type GWS_Lock = GWS_FLY_LOCK;
GWS_Remote_Type GWS_Remote = {
    .Accelerator = 0,
    .GWS_Fly_Mode = GWS_FLY_MODE_NONE,
};
uint8_t GWS_Debug_Flag = 0;