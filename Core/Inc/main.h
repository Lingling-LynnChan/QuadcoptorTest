/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_R_Pin GPIO_PIN_7
#define LED_R_GPIO_Port GPIOE
#define LED_G_Pin GPIO_PIN_8
#define LED_G_GPIO_Port GPIOE
#define LED_B_Pin GPIO_PIN_9
#define LED_B_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
#define I2C_SCL_Pin GPIO_PIN_0
#define I2C_SCL_GPIO_Port GPIOC
#define I2C_SCL_GPIO_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
#define I2C_SDA_Pin GPIO_PIN_1
#define I2C_SDA_GPIO_Port GPIOC
#define I2C_SDA_GPIO_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
// GWS_MPU6050 配置
#define MPU6050_GYRO_RANGE_CONFIG 3   // �?螺仪量程2000dps
#define MPU6050_ACCEL_RANGE_CONFIG 0  // 加�?�度计量�?2g
#define MPU6050_RATE_HZ 50            // 采样�?50hz

#define MPU6050_GYRO_RANGE_CONFIG_GET_VALUE \
  (MPU6050_GYRO_RANGE_CONFIG == 0)   ? 250  \
  : (MPU6050_GYRO_RANGE_CONFIG == 1) ? 500  \
  : (MPU6050_GYRO_RANGE_CONFIG == 2) ? 1000 \
                                     : 2000
#define MPU6050_ACCEL_RANGE_CONFIG_GET_VALUE \
  (MPU6050_ACCEL_RANGE_CONFIG == 0)   ? 2    \
  : (MPU6050_ACCEL_RANGE_CONFIG == 1) ? 4    \
  : (MPU6050_ACCEL_RANGE_CONFIG == 2) ? 8    \
                                      : 16
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
