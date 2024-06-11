#include "imu.h"
#include "gwtypes.h"

typedef struct {
  float Q0;
  float Q1;
  float Q2;
  float Q3;
} Quaternion;

void IMU_Get_Angle(GW_IMU_Angle_Type* angle, float dt) {
  GW_Vector3f Gravity, Acc, Gyro, AccGravity;
  static Quaternion NumQ;
}