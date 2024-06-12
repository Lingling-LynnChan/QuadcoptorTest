#include "pid.h"

#include "gwmath.h"

void GW_PID_Param_Init(void) {
  GW_PID_State_Type Def = {
      .RateX =
          {
              .P = 3.0f,
              .I = 0.0f,
              .D = 0.2f,
              .Integral_Limit_High = 0,
              .Integral_Limit_Low = 0,
              .Out_Limit_High = 0,
              .Out_Limit_Low = 0,
          },
      .RateY =
          {
              .P = 3.0f,
              .I = 0.0f,
              .D = 0.2f,
              .Integral_Limit_High = 0,
              .Integral_Limit_Low = 0,
              .Out_Limit_High = 0,
              .Out_Limit_Low = 0,
          },
      .RateZ =
          {
              .P = 8.0f,
              .I = 0.0f,
              .D = 0.4f,
              .Integral_Limit_High = 0,
              .Integral_Limit_Low = 0,
              .Out_Limit_High = 0,
              .Out_Limit_Low = 0,
          },
      .Pitch =
          {
              .P = 8.0f,
              .I = 0.0f,
              .D = 0.0f,
              .Integral_Limit_High = 0,
              .Integral_Limit_Low = 0,
              .Out_Limit_High = 0,
              .Out_Limit_Low = 0,
          },
      .Roll =
          {
              .P = 8.0f,
              .I = 0.0f,
              .D = 0.0f,
              .Integral_Limit_High = 0,
              .Integral_Limit_Low = 0,
              .Out_Limit_High = 0,
              .Out_Limit_Low = 0,
          },
      .Yaw =
          {
              .P = 6.0f,
              .I = 0.0f,
              .D = 0.0f,
              .Integral_Limit_High = 0,
              .Integral_Limit_Low = 0,
              .Out_Limit_High = 0,
              .Out_Limit_Low = 0,
          },
  };
  GWS_PID = Def;
}

void GW_PID_Update(GW_PID_Type* pid, float dt) {
  float Error = pid->Desired - pid->Measured + pid->Offset;  // 求误差
  pid->Integral += Error * dt;  // 误差积分累加
#if 0
  // 积分限幅
  pid->Integral = GWM_Limit(pid->Integral,            //
                            pid->Integral_Limit_Low,  //
                            pid->Integral_Limit_High);
#endif
  float Derivative = (Error - pid->Previous_Error) / dt;  // 前后两次误差做微分
  pid->Out = pid->P * Error + pid->I * pid->Integral + pid->D * Derivative;
#if 0
  // 输出限幅
  pid->Out = GWM_Limit(pid->Out, pid->Out_Limit_Low, pid->Out_Limit_High);
#endif
  pid->Previous_Error = Error;  // 更新误差
}

void GW_PID_Reset(void) {
  GW_PID_Type* pid = (GW_PID_Type*)&GWS_PID;
  for (uint8_t i = 0; i < 6; i++) {
    pid[i].Measured = 0;
    pid[i].Desired = 0;
    pid[i].Integral = 0;
    pid[i].Previous_Error = 0;
    pid[i].Out = 0;
  }
}