#include "i2c.h"
#include "tim.h"

#define GW_I2C_DELAY_X2() GW_Delay_Us(4)
#define GW_I2C_DELAY() GW_Delay_Us(2)

#define GW_I2C_MODE_IN()                         \
  {                                              \
    I2C_SDA_GPIO_Port->MODER &= ~(3 << (1 * 2)); \
    I2C_SDA_GPIO_Port->MODER |= 0 << (1 * 2);    \
  }  // 输入模式
#define GW_I2C_MODE_OUT()                        \
  {                                              \
    I2C_SDA_GPIO_Port->MODER &= ~(3 << (1 * 2)); \
    I2C_SDA_GPIO_Port->MODER |= 1 << (1 * 2);    \
  }  // 输出模式

#define GW_I2C_SCL_SET(n)                           \
  HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, \
                    n ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define GW_I2C_SDA_SET(n)                           \
  HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, \
                    n ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define GW_I2C_READ() HAL_GPIO_ReadPin(I2C_SDA_GPIO_Port, I2C_SDA_Pin)

/**
 * @brief  I2C开始信号
 */
static void GW_I2C_Begin(void) {
  GW_I2C_MODE_OUT();
  GW_I2C_SDA_SET(1);
  GW_I2C_SCL_SET(1);
  GW_I2C_DELAY_X2();
  GW_I2C_SDA_SET(0);
  GW_I2C_DELAY_X2();
  GW_I2C_SCL_SET(0);
}
/**
 * @brief I2C结束信号
 */
static void GW_I2C_End(void) {
  GW_I2C_MODE_OUT();
  GW_I2C_SDA_SET(0);
  GW_I2C_SCL_SET(1);
  GW_I2C_DELAY_X2();
  GW_I2C_SDA_SET(1);
  GW_I2C_DELAY_X2();
  GW_I2C_SCL_SET(0);
}
static HAL_StatusTypeDef GW_I2C_Wait_Ack(void) {
  uint8_t err = 0;
  GW_I2C_MODE_IN();
  GW_I2C_SDA_SET(1);
  GW_I2C_DELAY();
  GW_I2C_SCL_SET(1);
  GW_I2C_DELAY();
  while (GW_I2C_READ()) {
    err++;
    if (err > 250) {
      GW_I2C_End();
      return HAL_TIMEOUT;
    }
  }
  GW_I2C_SCL_SET(0);
  return HAL_OK;
}
static void GW_I2C_Ack(void) {
  GW_I2C_SCL_SET(0);
  GW_I2C_MODE_OUT();
  GW_I2C_SDA_SET(0);
  GW_I2C_DELAY();
  GW_I2C_SCL_SET(1);
  GW_I2C_DELAY();
  GW_I2C_SCL_SET(0);
}
static void GW_I2C_NAck(void) {
  GW_I2C_SCL_SET(0);
  GW_I2C_MODE_OUT();
  GW_I2C_SDA_SET(1);
  GW_I2C_DELAY();
  GW_I2C_SCL_SET(1);
  GW_I2C_DELAY();
  GW_I2C_SCL_SET(0);
}
static void GW_I2C_Wb(uint8_t data) {
  GW_I2C_MODE_OUT();
  GW_I2C_SCL_SET(0);
  for (uint8_t i = 0; i < 8; i++) {
    GW_I2C_SDA_SET((data & 0x80) >> 7);
    data <<= 1;
    GW_I2C_SCL_SET(1);
    GW_I2C_DELAY();
    GW_I2C_SCL_SET(0);
    GW_I2C_DELAY();
  }
}
static uint8_t GW_I2C_Rb(uint8_t ack) {
  uint8_t data = 0;
  GW_I2C_MODE_IN();
  for (uint8_t i = 0; i < 8; i++) {
    GW_I2C_SCL_SET(0);
    GW_I2C_DELAY();
    GW_I2C_SCL_SET(1);
    data <<= 1;
    if (GW_I2C_READ()) {
      data++;
    }
    GW_I2C_DELAY();
  }
  if (ack) {
    GW_I2C_Ack();
  } else {
    GW_I2C_NAck();
  }
  return data;
}

void GW_I2C_Init(void) {
  static uint8_t is_init = 0;
  if (is_init) {
    return;
  }

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  I2C_SCL_GPIO_ENABLE();
  GPIO_InitStruct.Pin = I2C_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(I2C_SCL_GPIO_Port, &GPIO_InitStruct);

  I2C_SDA_GPIO_ENABLE();
  GPIO_InitStruct.Pin = I2C_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(I2C_SDA_GPIO_Port, &GPIO_InitStruct);

  GW_I2C_SDA_SET(1);
  GW_I2C_SCL_SET(1);
  is_init = 1;
}

HAL_StatusTypeDef GW_I2C_Write_Byte(uint8_t daddr, uint8_t reg, uint8_t data) {
  GW_I2C_Begin();
  GW_I2C_Wb((daddr << 1) | 0);
  if (GW_I2C_Wait_Ack() != HAL_OK) {
    return HAL_TIMEOUT;
  }
  GW_I2C_Wb(reg);
  if (GW_I2C_Wait_Ack() != HAL_OK) {
    return HAL_TIMEOUT;
  }
  GW_I2C_Wb(data);
  if (GW_I2C_Wait_Ack() != HAL_OK) {
    return HAL_TIMEOUT;
  }
  GW_I2C_End();
  return HAL_OK;
}
uint8_t GW_I2C_Read_Byte(uint8_t daddr, uint8_t reg) {
  GW_I2C_Begin();
  GW_I2C_Wb((daddr << 1) | 0);
  if (GW_I2C_Wait_Ack() != HAL_OK) {
    return 0;
  }
  GW_I2C_Wb(reg);
  if (GW_I2C_Wait_Ack() != HAL_OK) {
    return 0;
  }
  GW_I2C_Begin();
  GW_I2C_Wb((daddr << 1) | 1);
  if (GW_I2C_Wait_Ack() != HAL_OK) {
    return 0;
  }
  uint8_t data = GW_I2C_Rb(0);
  GW_I2C_End();
  return data;
}
HAL_StatusTypeDef GW_I2C_Write_Data(uint8_t daddr,
                                    uint8_t reg,
                                    uint8_t* buf,
                                    uint8_t len) {
  GW_I2C_Begin();
  GW_I2C_Wb((daddr << 1) | 0);
  if (GW_I2C_Wait_Ack() != HAL_OK) {
    return HAL_TIMEOUT;
  }
  GW_I2C_Wb(reg);
  if (GW_I2C_Wait_Ack() != HAL_OK) {
    return HAL_TIMEOUT;
  }
  for (uint8_t i = 0; i < len; i++) {
    GW_I2C_Wb(buf[i]);
    if (GW_I2C_Wait_Ack() != HAL_OK) {
      return HAL_ERROR;
    }
  }
  GW_I2C_End();
  return HAL_OK;
}
HAL_StatusTypeDef GW_I2C_Read_Data(uint8_t daddr,
                                   uint8_t reg,
                                   uint8_t* buf,
                                   uint8_t len) {
  GW_I2C_Begin();
  GW_I2C_Wb((daddr << 1) | 0);
  if (GW_I2C_Wait_Ack() != HAL_OK) {
    return HAL_TIMEOUT;
  }
  GW_I2C_Wb(reg);
  if (GW_I2C_Wait_Ack() != HAL_OK) {
    return HAL_TIMEOUT;
  }
  GW_I2C_Begin();
  GW_I2C_Wb((daddr << 1) | 1);
  if (GW_I2C_Wait_Ack() != HAL_OK) {
    return HAL_TIMEOUT;
  }
  while (len) {
    *buf = GW_I2C_Rb(len == 1 ? 0 : 1);
    len--;
    buf++;
  }
  GW_I2C_End();
  return HAL_OK;
}
