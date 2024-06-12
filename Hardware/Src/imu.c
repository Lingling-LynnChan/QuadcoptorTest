#include "imu.h"
#include "gwmath.h"
#include "gwtypes.h"
#include "icm20608.h"

// GLOBAL
GW_IMU_Angle_Type GWS_IMU_Angle = {0};

const float IMU_RTA = 57.2957795f;
const float IMU_ATR = 0.0174532925f;
const float IMU_GYRO_G =
    1.f / (65536.f / (MPU6050_GYRO_RANGE_CONFIG_GET_VALUE * 2.f));
const float IMU_GYRO_GR = IMU_GYRO_G * IMU_ATR;

typedef struct {
  float Q0;
  float Q1;
  float Q2;
  float Q3;
} Quaternion;

static float NormAccelOnZ;  // Z轴垂直方向上的加速度

/**
 * @brief 获取姿态角
 * @param dt 采样时间
 */
void IMU_Get_Angle(float dt) {
  GW_IMU_Angle_Type* angle = &GWS_IMU_Angle;
  static GW_Vector3f GyroIntegralError = {.X = 0.f, .Y = 0.f, .Z = 0.f};
  static float KpDef = 0.8f;
  static float KiDef = 0.0003f;
  static Quaternion NumQ = {1.f, 0.f, 0.f, 0.f};
  GW_Vector3f Gravity, Accel, Gyro, AccelGravity;
  float NormQuat;
  float HalfDt = dt * 0.5f;
  Quaternion Temp;
  // 提取等效旋转矩阵中的重力分量
  Gravity.X = 2 * (NumQ.Q1 * NumQ.Q3 - NumQ.Q0 * NumQ.Q2);
  Gravity.Y = 2 * (NumQ.Q0 * NumQ.Q1 + NumQ.Q2 * NumQ.Q3);
  Gravity.Z = 1 - 2 * (NumQ.Q1 * NumQ.Q1 + NumQ.Q2 * NumQ.Q2);
  // 加速度归一化
  NormQuat = GWM_RSqrt(                  //
      GWM_Square(GWS_MPU6050.Accel.X) +  //
      GWM_Square(GWS_MPU6050.Accel.Y) +  //
      GWM_Square(GWS_MPU6050.Accel.Z));  //
  Accel.X = GWS_MPU6050.Accel.X * NormQuat;
  Accel.Y = GWS_MPU6050.Accel.Y * NormQuat;
  Accel.Z = GWS_MPU6050.Accel.Z * NormQuat;
  // 求向量积
  AccelGravity.X = Accel.Y * Gravity.Z - Accel.Z * Gravity.Y;
  AccelGravity.Y = Accel.Z * Gravity.X - Accel.X * Gravity.Z;
  AccelGravity.Z = Accel.X * Gravity.Y - Accel.Y * Gravity.X;
  // 加速度积分补偿角速度的补偿值
  GyroIntegralError.X += AccelGravity.X * KiDef;
  GyroIntegralError.Y += AccelGravity.Y * KiDef;
  GyroIntegralError.Z += AccelGravity.Z * KiDef;
  // 角速度融合加速度积分补偿值
  Gyro.X = GWS_MPU6050.Gyro.X * IMU_GYRO_GR + KpDef * AccelGravity.X +
           GyroIntegralError.X;
  Gyro.Y = GWS_MPU6050.Gyro.Y * IMU_GYRO_GR + KpDef * AccelGravity.Y +
           GyroIntegralError.Y;
  Gyro.Z = GWS_MPU6050.Gyro.Z * IMU_GYRO_GR + KpDef * AccelGravity.Z +
           GyroIntegralError.Z;
  // 一阶龙格库塔法更新四元数
  Temp.Q0 = (-NumQ.Q1 * Gyro.X - NumQ.Q2 * Gyro.Y - NumQ.Q3 * Gyro.Z) * HalfDt;
  Temp.Q1 = (NumQ.Q0 * Gyro.X - NumQ.Q3 * Gyro.Y + NumQ.Q2 * Gyro.Z) * HalfDt;
  Temp.Q2 = (NumQ.Q3 * Gyro.X + NumQ.Q0 * Gyro.Y - NumQ.Q1 * Gyro.Z) * HalfDt;
  Temp.Q3 = (-NumQ.Q2 * Gyro.X + NumQ.Q1 * Gyro.Y + NumQ.Q0 * Gyro.Z) * HalfDt;
  NumQ.Q0 += Temp.Q0;
  NumQ.Q1 += Temp.Q1;
  NumQ.Q2 += Temp.Q2;
  NumQ.Q3 += Temp.Q3;
  // 四元数归一化
  NormQuat = GWM_RSqrt(      //
      GWM_Square(NumQ.Q0) +  //
      GWM_Square(NumQ.Q1) +  //
      GWM_Square(NumQ.Q2) +  //
      GWM_Square(NumQ.Q3));  //
  NumQ.Q0 *= NormQuat;
  NumQ.Q1 *= NormQuat;
  NumQ.Q2 *= NormQuat;
  NumQ.Q3 *= NormQuat;
  // 四元数转欧拉角
  GW_Vector3f vecToZ = {
      .X = 2.f * NumQ.Q1 * NumQ.Q3 - 2.f * NumQ.Q0 * NumQ.Q2,
      .Y = 2.f * NumQ.Q2 * NumQ.Q3 + 2.f * NumQ.Q0 * NumQ.Q1,
      .Z = 1.f - 2.f * GWM_Square(NumQ.Q1) - 2.f * GWM_Square(NumQ.Q2),
  };
  float YAW_G = GWS_MPU6050.Gyro.Z * IMU_GYRO_G;
  if (YAW_G > 0.8f || YAW_G < -0.8f) {
    angle->Yaw += YAW_G * dt;  // 角速度积分偏航角
  }
  angle->Pitch = -GWM_Arcsin(vecToZ.X) * IMU_RTA;           // 俯仰角
  angle->Roll = GWM_Arctan2(vecToZ.Y, vecToZ.Z) * IMU_RTA;  // 横滚角
  // Z轴垂直方向上的加速度
  NormAccelOnZ = GWS_MPU6050.Accel.X * vecToZ.X +  //
                 GWS_MPU6050.Accel.Y * vecToZ.Y +  //
                 GWS_MPU6050.Accel.Z * vecToZ.Z;   //
}

float IMU_Get_Norm_AccelZ(void) {
  static GW_Kalman_State_Type kalman = {
      0.02f, 0.f, 0.f, 0.f, 0.001f, 0.0f,
  };
  GW_Kalman_V1(&kalman, NormAccelOnZ);
  NormAccelOnZ = kalman.Out;
  return NormAccelOnZ;
}
