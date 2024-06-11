#include "gwmath.h"

#define GWM_PI_DIV2 1.570796f

/**
 * @brief 泰勒展开
 * @note tapyor次展开
 */
static float GWM_Sin_Tapyor(float x, int tapyor) {
  float t = x;
  float result = x;
  float X2 = x * x;
  uint8_t cnt = 1;
  do {
    t = -t;
    t *= X2;
    result += t / ((cnt << 1) + 1);
    cnt++;
  } while (cnt <= tapyor);
  return result;
}

/**
 * @brief 反正弦麦克劳林展开式
 * @note 42度以内准确
 */
float GWM_Arcsin(float x) {
  float d = 1;
  float t = x;
  unsigned char cnt = 1;
  float result = 0;
  float X2 = x * x;
  if (x >= 1.0f) {
    return GWM_PI_DIV2;
  }
  if (x <= -1.0f) {
    return -GWM_PI_DIV2;
  }
  do {
    result += t / (d * ((cnt << 1) - 1));
    t *= X2 * ((cnt << 1) - 1);
    d *= (cnt << 1);
    cnt++;
  } while (cnt <= 6);
  return result;
}
/**
 * @brief 反正切麦克劳林展开式
 * @note 70度以内准确
 */
float GWM_Arctan(float x) {
  float t = x;
  float result = 0;
  float X2 = x * x;
  unsigned char cnt = 1;
  do {
    result += t / ((cnt << 1) - 1);
    t = -t;
    t *= X2;
    cnt++;
  } while (cnt <= 6);
  return result;
}
/**
 * @brief 反正切函数
 */
float GWM_Arctan2(float y, float x) {
  return GWM_Arctan(y / x);
}
/**
 * @brief 泰勒展开
 * @note 4次展开
 */
float GWM_Sin(float x) {
  return GWM_Sin_Tapyor(x, 4);
}
/**
 * @brief 余弦函数
 * @note 精确度依赖于GWM_Sin函数
 */
float GWM_Cos(float x) {
  return GWM_Sin(x + GWM_PI / 2.f);
}
/**
 * @brief WTF算法
 */
float GWM_RSqrt(float x) {
  uint64_t i;
  float x2, y;
  const float threehalfs = 1.5F;
  x2 = x * 0.5F;
  y = x;
  i = *(uint64_t*)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i;
  y = y * (threehalfs - (x2 * y * y));
  return y;
}