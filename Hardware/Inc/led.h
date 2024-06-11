#ifndef __LED_H__
#define __LED_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define LED_ON(COLOR) \
  HAL_GPIO_WritePin(LED_##COLOR##_GPIO_Port, LED_##COLOR##_Pin, GPIO_PIN_RESET)
#define LED_OFF(COLOR) \
  HAL_GPIO_WritePin(LED_##COLOR##_GPIO_Port, LED_##COLOR##_Pin, GPIO_PIN_SET)
#define LED_TOGGLE(COLOR) \
  HAL_GPIO_TogglePin(LED_##COLOR##_GPIO_Port, LED_##COLOR##_Pin)

#ifdef __cplusplus
}
#endif

#endif /*__LED_H__ */