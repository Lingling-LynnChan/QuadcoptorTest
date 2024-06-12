#include "icm20608.h"
#include "gwmath.h"
#include "i2c.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "string.h"

static int8_t GW_Inv_Matrix[9] = {
    -1, 0,  0,  //
    0,  -1, 0,  //
    0,  0,  1,  //
};
static uint16_t GW_Inv_Orientation_Matrix_To_Scale(int8_t mtx[]);
static int run_test(void);
/**
 * @brief 初始化ICM20608
 */
HAL_StatusTypeDef ICM20608_Init(void) {
  struct int_param_s int_param;
  int err;
  GW_I2C_Init();
  err = mpu_init(&int_param);
  if (err) {
    printf("mpu_init failed\n");
    return HAL_ERROR;
  }
  // 使能陀螺仪和加速度计
  err = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  if (err) {
    printf("mpu_set_sensors failed\n");
    return HAL_ERROR;
  }
  // 设置FIFO
  err = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  if (err) {
    printf("mpu_configure_fifo failed\n");
    return HAL_ERROR;
  }
  // 设置采样率
  err = mpu_set_sample_rate(MPU6050_RATE_HZ);
  if (err) {
    printf("mpu_set_sample_rate failed\n");
    return HAL_ERROR;
  }
  // 加载DMP
  err = dmp_load_motion_driver_firmware();
  if (err) {
    printf("dmp_load_motion_driver_firmware failed\n");
    return HAL_ERROR;
  }
  // 设置陀螺仪方向
  err = dmp_set_orientation(GW_Inv_Orientation_Matrix_To_Scale(GW_Inv_Matrix));
  if (err) {
    printf("dmp_set_orientation failed\n");
    return HAL_ERROR;
  }
  // 使能DMP功能
  err = dmp_enable_feature(         //
      DMP_FEATURE_6X_LP_QUAT |      // 6轴低功耗四元数
      DMP_FEATURE_TAP |             // 敲击检测
      DMP_FEATURE_ANDROID_ORIENT |  // 朝向数据使用安卓标准
      DMP_FEATURE_SEND_RAW_ACCEL |  // 发送原始加速度计数据
      DMP_FEATURE_SEND_CAL_GYRO |   // 发送校准后的陀螺仪数据
      DMP_FEATURE_GYRO_CAL          // 陀螺仪校准
  );
  if (err) {
    printf("dmp_enable_feature failed\n");
    return HAL_ERROR;
  }
  // 自检
  err = run_test();
  if (err) {
    printf("run_test failed\n");
    return HAL_ERROR;
  }
  // 使能DMP
  err = mpu_set_dmp_state(1);
  if (err) {
    printf("mpu_set_dmp_state failed\n");
    return HAL_ERROR;
  }
  printf("ICM20608 Init Success\n");
  return HAL_OK;
}

HAL_StatusTypeDef ICM20608_Read(float* pitch, float* roll, float* yaw) {
  int err;
  const float Q30 = 1 << 30;
  Quaternion NumQ = {
      .Q0 = 1.f,
      .Q1 = 0.f,
      .Q2 = 0.f,
      .Q3 = 0.f,
  };
  int16_t Gyro[3];
  int16_t Accel[3];
  int32_t Quat[4];
  uint32_t Timestamp;
  int16_t Sensors;
  uint8_t more;
  err = dmp_read_fifo(Gyro, Accel, (long*)&Quat, &Timestamp, &Sensors, &more);
  if (err) {
    printf("dmp_read_fifo failed\n");
    return HAL_ERROR;
  }
  if (!(Sensors & INV_WXYZ_QUAT)) {
    printf("INV_WXYZ_QUAT not found\n");
    return HAL_ERROR;
  }
  NumQ.Q0 = Quat[0] / Q30;
  NumQ.Q1 = Quat[1] / Q30;
  NumQ.Q2 = Quat[2] / Q30;
  NumQ.Q3 = Quat[3] / Q30;
  *pitch = GWM_Arcsin(-2.f * (NumQ.Q1 * NumQ.Q3 + NumQ.Q0 * NumQ.Q2)) * 57.3f;
  *roll = GWM_Arctan2(                                                   //
              2.f * (NumQ.Q2 * NumQ.Q3 + NumQ.Q0 * NumQ.Q1),             //
              -2.f * (GWM_Square(NumQ.Q1) + GWM_Square(NumQ.Q2)) + 1) *  //
          57.3f;                                                         //
  *yaw = GWM_Arctan2(                                                    //
             2.f * (NumQ.Q1 * NumQ.Q2 + NumQ.Q0 * NumQ.Q3),              //
             GWM_Square(NumQ.Q0) + GWM_Square(NumQ.Q1) -                 //
                 GWM_Square(NumQ.Q2) - GWM_Square(NumQ.Q3)) *            //
         57.3f;
  return HAL_OK;
}
static uint16_t GW_Inv_Row_To_Scale(int8_t row[]) {
  uint16_t b;
  if (row[0] > 0) {
    b = 0;
  } else if (row[0] < 0) {
    b = 4;
  } else if (row[1] > 0) {
    b = 1;
  } else if (row[1] < 0) {
    b = 5;
  } else if (row[2] > 0) {
    b = 2;
  } else if (row[2] < 0) {
    b = 6;
  } else {
    b = 7;
  }
  return b;
}

static uint16_t GW_Inv_Orientation_Matrix_To_Scale(int8_t mtx[]) {
  uint16_t b = 0;
  b |= (GW_Inv_Row_To_Scale(mtx + 0) << 0);
  b |= (GW_Inv_Row_To_Scale(mtx + 3) << 3);
  b |= (GW_Inv_Row_To_Scale(mtx + 6) << 6);
  return b;
}
static int run_test(void) {
  int32_t Gyro[3], Accel[3];
  int result = mpu_run_self_test(Gyro, Accel);
  if (result != 0x3) {
    return -1;
  }
  float Gyro_Sens;
  mpu_get_gyro_sens(&Gyro_Sens);
  Gyro[0] = (int32_t)(Gyro[0] * Gyro_Sens);
  Gyro[1] = (int32_t)(Gyro[1] * Gyro_Sens);
  Gyro[2] = (int32_t)(Gyro[2] * Gyro_Sens);
  dmp_set_gyro_bias(Gyro);
  uint16_t Accel_Sens;
  mpu_get_accel_sens(&Accel_Sens);
  Accel[0] *= Accel_Sens;
  Accel[1] *= Accel_Sens;
  Accel[2] *= Accel_Sens;
  dmp_set_accel_bias(Accel);
  return 0;
}
