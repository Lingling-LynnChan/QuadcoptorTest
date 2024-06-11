#include "icm20608.h"
#include "i2c.h"
#include "string.h"

/**
 * @brief 初始化ICM20608
 */
HAL_StatusTypeDef ICM20608_Init(void) {
  HAL_StatusTypeDef state;
  GW_I2C_Init();
  // 复位
  state = GW_I2C_Write_Byte(MPU_ADDR, MPU_PWR_MGMT1_REG, 0x80);
  if (state != HAL_OK) {
    return HAL_ERROR;
  }
  HAL_Delay(100);
  // 开机
  state = GW_I2C_Write_Byte(MPU_ADDR, MPU_PWR_MGMT1_REG, 0x00);
  if (state != HAL_OK) {
    return HAL_ERROR;
  }
  // 配置
  MPU_Set_Gyro_Fsr(MPU6050_GYRO_RANGE_CONFIG);           // 加速度量程
  MPU_Set_Accel_Fsr(MPU6050_ACCEL_RANGE_CONFIG);         // 陀螺仪量程
  MPU_Set_Rate(MPU6050_RATE_HZ);                         // 采样率
  GW_I2C_Write_Byte(MPU_ADDR, MPU_INT_EN_REG, 0x00);     // 关闭所有中断
  GW_I2C_Write_Byte(MPU_ADDR, MPU_USER_CTRL_REG, 0x00);  // I2C主模式关闭
  GW_I2C_Write_Byte(MPU_ADDR, MPU_FIFO_EN_REG, 0x00);    // 关闭FIFO
  GW_I2C_Write_Byte(MPU_ADDR, MPU_INTBP_CFG_REG, 0x80);  // INT引脚低电平有效
  GW_I2C_Write_Byte(MPU_ADDR, 0x1D, 1);  // INT引脚低电平有效
  uint8_t id = GW_I2C_Read_Byte(MPU_ADDR, MPU_DEVICE_ID_REG);
  if (id != 0xAF) {  // 器件ID不正确
    return HAL_ERROR;
  }
  GW_I2C_Write_Byte(MPU_ADDR, MPU_PWR_MGMT1_REG, 0X01);  // 设置CLKSEL|PLL X轴
  GW_I2C_Write_Byte(MPU_ADDR, MPU_PWR_MGMT2_REG, 0X00);  // 启用accel|gyro
  MPU_Set_Rate(50);
  return HAL_OK;
}

/**
 * @brief 设置ICM20608加速度传感器满量程范围
 * @param fsr 0:250dps,1:500dps,2:1000dps,3:2000dps
 */
HAL_StatusTypeDef MPU_Set_Gyro_Fsr(uint8_t config) {
  return GW_I2C_Write_Byte(MPU_ADDR, MPU_GYRO_CFG_REG, config << 3);
}
/**
 * @brief 设置ICM20608陀螺仪传感器满量程范围
 * @param fsr 0:2g,1:4g,2:8g,3:16g
 */
HAL_StatusTypeDef MPU_Set_Accel_Fsr(uint8_t config) {
  return GW_I2C_Write_Byte(MPU_ADDR, MPU_ACCEL_CFG_REG, config << 3);
}
/**
 * @brief 设置ICM20608的数字低通滤波器
 * @param lpf 数字低通滤波频率:Hz
 */
HAL_StatusTypeDef MPU_Set_Lpf(uint16_t lpf) {
  uint8_t data = 0;
  if (lpf >= 188) {
    data = 1;
  } else if (lpf >= 98) {
    data = 2;
  } else if (lpf >= 42) {
    data = 3;
  } else if (lpf >= 20) {
    data = 4;
  } else if (lpf >= 10) {
    data = 5;
  } else {
    data = 6;
  }
  return GW_I2C_Write_Byte(MPU_ADDR, MPU_CFG_REG, data);
}
/**
 * @brief 设置ICM20608的采样率
 * @param rate 采样率: 4hz ... 1khz
 */
HAL_StatusTypeDef MPU_Set_Rate(uint16_t rate) {
  if (rate > 1000)
    rate = 1000;
  if (rate < 4)
    rate = 4;
  uint8_t data = 1000 / rate - 1;
  data = GW_I2C_Write_Byte(MPU_ADDR, MPU_SAMPLE_RATE_REG, data);
  data |= MPU_Set_Lpf(rate / 2);
  return data;
}
/**
 * @brief 获取温度
 */
HAL_StatusTypeDef MPU_Get_Temp(float* temp) {
  uint8_t buf[2];
  if (GW_I2C_Read_Data(MPU_ADDR, MPU_TEMP_OUTH_REG, buf, 2) != HAL_OK) {
    return HAL_ERROR;
  }
  int16_t raw = (buf[0] << 8) | buf[1];
  *temp = 36.53 + raw / 340.0f;
  return HAL_OK;
}
/**
 * @brief 获取陀螺仪
 */
HAL_StatusTypeDef MPU_Get_Gyro(void) {
  uint8_t buf[6];
  HAL_StatusTypeDef state =
      GW_I2C_Read_Data(MPU_ADDR, MPU_GYRO_XOUTH_REG, buf, 6);
  if (state != HAL_OK) {
    return state;
  }
  GWS_MPU6050.Gyro.X = (((uint16_t)buf[0] << 8) | buf[1]) - GWS_MPU6050.Gyro_Offset.X;
  GWS_MPU6050.Gyro.Y = (((uint16_t)buf[2] << 8) | buf[3]) - GWS_MPU6050.Gyro_Offset.Y;
  GWS_MPU6050.Gyro.Z = (((uint16_t)buf[4] << 8) | buf[5]) - GWS_MPU6050.Gyro_Offset.Z;
  return HAL_OK;
}
/**
 * @brief 获取加速度
 */
HAL_StatusTypeDef MPU_Get_Accel(void) {
  uint8_t buf[6];
  HAL_StatusTypeDef state =
      GW_I2C_Read_Data(MPU_ADDR, MPU_ACCEL_XOUTH_REG, buf, 6);
  if (state != HAL_OK) {
    return state;
  }
  GWS_MPU6050.Accel.X = (((uint16_t)buf[0] << 8) | buf[1]) - GWS_MPU6050.Accel_Offset.X;
  GWS_MPU6050.Accel.Y = (((uint16_t)buf[2] << 8) | buf[3]) - GWS_MPU6050.Accel_Offset.Y;
  GWS_MPU6050.Accel.Z = (((uint16_t)buf[4] << 8) | buf[5]) - GWS_MPU6050.Accel_Offset.Z;
  return HAL_OK;
}
/**
 * @brief 获取数据并进行滤波
 */
HAL_StatusTypeDef MPU_Get_And_Filter(void) {
  HAL_StatusTypeDef state = HAL_OK;
  state |= MPU_Get_Accel();  // 加速度
  state |= MPU_Get_Gyro();   // 角速度
  if (state != HAL_OK) {
    return HAL_ERROR;
  }
  // 对加速度进行卡尔曼滤波
  static MPU6050_Kalman_Type kalman = {.X = {0.02, 0, 0, 0, 0.001, 0.543},
                                       .Y = {0.02, 0, 0, 0, 0.001, 0.543},
                                       .Z = {0.02, 0, 0, 0, 0.001, 0.543}};
  GW_Kalman_V1(&kalman.X, (float)GWS_MPU6050.Accel.X);
  GWS_MPU6050.Accel.X = (uint16_t)kalman.X.Out;
  GW_Kalman_V1(&kalman.Y, (float)GWS_MPU6050.Accel.Y);
  GWS_MPU6050.Accel.Y = (uint16_t)kalman.Y.Out;
  GW_Kalman_V1(&kalman.Z, (float)GWS_MPU6050.Accel.Z);
  GWS_MPU6050.Accel.Z = (uint16_t)kalman.Z.Out;
  // 对角速度进行一阶低通滤波
  const float factor = 0.15f;
  static GW_Vector3i localGyro;
  GWS_MPU6050.Gyro.X = localGyro.X =
      localGyro.X * (1 - factor) + GWS_MPU6050.Gyro.X * factor;
  GWS_MPU6050.Gyro.Y = localGyro.Y =
      localGyro.Y * (1 - factor) + GWS_MPU6050.Gyro.Y * factor;
  GWS_MPU6050.Gyro.Z = localGyro.Z =
      localGyro.Z * (1 - factor) + GWS_MPU6050.Gyro.Z * factor;
  return HAL_OK;
}
/**
 * @brief 重置ICM20608
 */
HAL_StatusTypeDef MPU_Reset(void) {
  const int16_t MAX_GYRO_VALUE = 5;
  const int16_t MIN_GYRO_VALUE = -5;
  GW_Vector3i lastGyro = {0};
  GW_Vector3i erroGyro;
  GWS_MPU6050.Accel_Offset.X = 0;
  GWS_MPU6050.Accel_Offset.Y = 8192;
  GWS_MPU6050.Accel_Offset.Z = 0;
  GWS_MPU6050.Gyro_Offset.X = 0;
  GWS_MPU6050.Gyro_Offset.Y = 0;
  GWS_MPU6050.Gyro_Offset.Z = 0;
  int8_t times = 30;  // 读30次，看是否静止不动
  while (times--) {
    if (MPU_Get_And_Filter() != HAL_OK) {
      return HAL_ERROR;
    }
    erroGyro.X = GWS_MPU6050.Gyro.X - lastGyro.X;
    erroGyro.Y = GWS_MPU6050.Gyro.Y - lastGyro.Y;
    erroGyro.Z = GWS_MPU6050.Gyro.Z - lastGyro.Z;
    lastGyro = GWS_MPU6050.Gyro;
    if (erroGyro.X > MAX_GYRO_VALUE || erroGyro.X < MIN_GYRO_VALUE ||
        erroGyro.Y > MAX_GYRO_VALUE || erroGyro.Y < MIN_GYRO_VALUE ||
        erroGyro.Z > MAX_GYRO_VALUE || erroGyro.Z < MIN_GYRO_VALUE) {
      times = 30;
      printf("erro=%3d|%3d|%3d, retiming\n", erroGyro.X, erroGyro.Y,
             erroGyro.Z);
    }
    HAL_Delay(10);
  }
  uint32_t buff[6] = {0};
  for (int16_t i = 0; i < 356; i++) {
    MPU_Get_And_Filter();
    if (i < 100) {
      continue;
    }
    buff[0] += GWS_MPU6050.Accel.X;
    buff[1] += GWS_MPU6050.Accel.Y;
    buff[2] += GWS_MPU6050.Accel.Z;
    buff[3] += GWS_MPU6050.Gyro.X;
    buff[4] += GWS_MPU6050.Gyro.Y;
    buff[5] += GWS_MPU6050.Gyro.Z;
  }
  GWS_MPU6050.Accel_Offset.X = buff[0] >> 8;
  GWS_MPU6050.Accel_Offset.Y = buff[1] >> 8;
  GWS_MPU6050.Accel_Offset.Z = buff[2] >> 8;
  GWS_MPU6050.Gyro_Offset.X = buff[3] >> 8;
  GWS_MPU6050.Gyro_Offset.Y = buff[4] >> 8;
  GWS_MPU6050.Gyro_Offset.Z = buff[5] >> 8;
  return HAL_OK;
}
