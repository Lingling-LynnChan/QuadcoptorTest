#include "gwmath.h"
#ifdef __GW_USE_CMATH__
#include "math.h"
#endif
#define GWM_PI_DIV2 1.570796f

#ifndef __GW_USE_CMATH__
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
#endif

/**
 * @brief 反正弦麦克劳林展开式
 * @note 42度以内准确
 */
float GWM_Arcsin(float x) {
#ifdef __GW_USE_CMATH__
  return asinf(x);
#else
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
#endif
}
/**
 * @brief 反正切麦克劳林展开式
 * @note 70度以内准确
 */
float GWM_Arctan(float x) {
#ifdef __GW_USE_CMATH__
  return atanf(x);
#else
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
#endif
}
/**
 * @brief 反正切函数
 */
float GWM_Arctan2(float y, float x) {
#ifdef __GW_USE_CMATH__
  return atan2f(y, x);
#else
  return GWM_Arctan(y / x);
#endif
}
/**
 * @brief 泰勒展开
 * @note 4次展开
 */
float GWM_Sin(float x) {
#ifdef __GW_USE_CMATH__
  return sinf(x);
#else
  return GWM_Sin_Tapyor(x, 4);
#endif
}
/**
 * @brief 余弦函数
 * @note 精确度依赖于GWM_Sin函数
 */
float GWM_Cos(float x) {
#ifdef __GW_USE_CMATH__
  return cosf(x);
#else
  return GWM_Sin(x + GWM_PI / 2.f);
#endif
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
/**
 * @brief 浮点数转整数表示
 */
void GWM_Ftoiu(float f, int32_t* integer, uint32_t* decimal) {
  int8_t sign = f >= 0 ? 1 : -1;
  f = f >= 0 ? f : -f;
  *integer = (int32_t)f;
  float fdec = f - *integer;
  *integer *= sign;
  *decimal = (uint32_t)(fdec * 1000000);
}