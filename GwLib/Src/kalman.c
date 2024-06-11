#include "kalman.h"

void GW_Kalman_V1(GW_Kalman_State_Type* kalman, float in) {
  kalman->Now_P = kalman->Last_P + kalman->Q;
  kalman->Kg = kalman->Now_P / (kalman->Now_P + kalman->R);
  kalman->Out = kalman->Out + kalman->Kg * (in - kalman->Out);
  kalman->Last_P = (1 - kalman->Kg) * kalman->Now_P;
}