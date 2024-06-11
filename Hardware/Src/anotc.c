#include "anotc.h"
#include <string.h>
#include "icm20608.h"
#include "usart.h"
// PID校验
int16_t ANOTC_Check_PID;
// 来自上位机的PID数据缓存
static struct {
  uint8_t Rate[18];
  uint8_t Angle[18];
  uint8_t High[18];
} ANOTC_PID_Cache;
// 数据种类标志位
static struct {
  uint8_t PID1 : 1;  // 上位机PID组1
  uint8_t PID2 : 1;  // 上位机PID组2
  uint8_t PID3 : 1;  // 上位机PID组3
  uint8_t PID4 : 1;  // 上位机PID组4
  uint8_t PID5 : 1;  // 上位机PID组5
  uint8_t PID6 : 1;  // 上位机PID组6
  uint8_t REQ : 1;   // 上位机请求读取PID
} ANOTC_Recived_flag;
/**
 * @brief 处理接收到的数据帧
 */
void ANOTC_Recive(int8_t* pt) {
  switch (pt[2]) {
    case ANOTC_PID1:  // 速度环
      memcpy(ANOTC_PID_Cache.Rate, &pt[4], 18);
      ANOTC_Recived_flag.PID1 = 1;
      break;
    case ANOTC_PID2:  // 角度环
      memcpy(ANOTC_PID_Cache.Angle, &pt[4], 18);
      ANOTC_Recived_flag.PID2 = 1;
      break;
    case ANOTC_PID3:  // 高度环
      memcpy(ANOTC_PID_Cache.High, &pt[4], 18);
      ANOTC_Recived_flag.PID3 = 1;
      break;
    case ANOTC_PID4:
      break;
    case ANOTC_PID5:
      break;
    case ANOTC_PID6:
      break;
    case ANOTC_STATUS:
      break;
    case ANOTC_SENSER: {
      enum {
        LOCAL_READ_PID = 0X01,    // 读取飞控的PID请求
        LOCAL_READ_MODE = 0x02,   // 读取飞行模式
        LOCAL_READ_ROUTE = 0x21,  // 读取航点信息
        LOCAL_READ_VER = 0XA0,    // 读取飞控版本
        LOCAL_RESET_PID = 0xA1    // 恢复默认PID
      };
      switch (*(uint8_t*)&pt[4])  // 判断上位机发来CMD的内容
      {
        case LOCAL_READ_PID:
          ANOTC_Recived_flag.REQ = 1;
          break;
        case LOCAL_READ_MODE:
          // TODO
          break;
        case LOCAL_READ_ROUTE:
          // TODO
          break;
        case LOCAL_READ_VER:
          // TODO
          break;
        case LOCAL_RESET_PID:
          // TODO
          break;
        default:
          break;
      }
    } break;
    case ANOTC_RCDATA:
      // TODO
      break;
    default:
      break;
  }
  return;
}
/**
 * @brief 发送数据给上位机
 */
void ANOTC_Send(ANOTC_Msg_Type type) {
  uint8_t i;
  uint8_t len = 2;
  int16_t ANOTC_Buffer[12];
  int8_t* pt = (int8_t*)(ANOTC_Buffer);
  GW_PID_State_Type* PID_X = NULL;
  GW_PID_State_Type* PID_Y = NULL;
  GW_PID_State_Type* PID_Z = NULL;

  switch (type) {
    case ANOTC_PID1:
      // TODO 角速度PID
      goto LABEL_SEND_PID;
    case ANOTC_PID2:
      // TODO 姿态PID
      goto LABEL_SEND_PID;
    case ANOTC_PID3:
      // TODO 高度PID
      goto LABEL_SEND_PID;
    case ANOTC_PID4:
    case ANOTC_PID5:
    case ANOTC_PID6:
    LABEL_SEND_PID:
      if (PID_X != NULL) {
        ANOTC_Buffer[2] = (int16_t)(PID_X->P * 1000);
        ANOTC_Buffer[3] = (int16_t)(PID_X->I * 1000);
        ANOTC_Buffer[4] = (int16_t)(PID_X->D * 1000);
      }
      if (PID_Y != NULL) {
        ANOTC_Buffer[5] = (int16_t)(PID_Y->P * 1000);
        ANOTC_Buffer[6] = (int16_t)(PID_Y->I * 1000);
        ANOTC_Buffer[7] = (int16_t)(PID_Y->D * 1000);
      }
      if (PID_Z != NULL) {
        ANOTC_Buffer[8] = (int16_t)(PID_Z->P * 1000);
        ANOTC_Buffer[9] = (int16_t)(PID_Z->I * 1000);
        ANOTC_Buffer[10] = (int16_t)(PID_Z->D * 1000);
      }
      len = 18;
      break;
    case ANOTC_MOTOR:
      ANOTC_Buffer[2] = 0;
      ANOTC_Buffer[3] = 0;
      ANOTC_Buffer[4] = 0;
      ANOTC_Buffer[5] = 0;
      ANOTC_Buffer[6] = 0;  // 最大可支持8组PWM上传上位机
      ANOTC_Buffer[7] = 0;
      ANOTC_Buffer[8] = 0;
      ANOTC_Buffer[9] = 0;
      len = 16;
      break;
    case ANOTC_RCDATA:
      // ANOTC_Buffer[2] = Remote.thr;
      // ANOTC_Buffer[3] = Remote.yaw;
      // ANOTC_Buffer[4] = Remote.roll;
      // ANOTC_Buffer[5] = Remote.pitch;
      // ANOTC_Buffer[6] = Remote.AUX1;
      // ANOTC_Buffer[7] = Remote.AUX2;
      // ANOTC_Buffer[8] = Remote.AUX3;
      // ANOTC_Buffer[9] = Remote.AUX4;
      // ANOTC_Buffer[10] = Remote.AUX3;
      // ANOTC_Buffer[11] = Remote.AUX4;
      // len = 20;
      break;
    case ANOTC_SENSER:
      MPU_Get_And_Filter();
      ANOTC_Buffer[2] = MPU6050.Accel.X;
      ANOTC_Buffer[3] = MPU6050.Accel.Y;
      ANOTC_Buffer[4] = MPU6050.Accel.Z;
      ANOTC_Buffer[5] = MPU6050.Gyro.X;
      ANOTC_Buffer[6] = MPU6050.Gyro.Y;
      ANOTC_Buffer[7] = MPU6050.Gyro.Z;
      ANOTC_Buffer[8] = 0;   // 磁力计X
      ANOTC_Buffer[9] = 0;   // 磁力计Y
      ANOTC_Buffer[10] = 0;  // 磁力计Z
      len = 18;
      break;
    case ANOTC_SENSER2:
      break;
    case ANOTC_STATUS:
      ANOTC_Buffer[2] = (int16_t)(Angle.roll * 100);
      ANOTC_Buffer[3] = (int16_t)(Angle.pitch * 100);
      ANOTC_Buffer[4] = -(int16_t)(Angle.yaw * 100);
      ((int32_t*)&ANOTC_Buffer[5])[0] = 0;                  // 高度数据
      ((uint8_t*)&ANOTC_Buffer[7])[0] = GWS_FLY_MODE_NONE;  // 飞行模式
      ((uint8_t*)&ANOTC_Buffer[7])[1] = GWS_FLY_LOCK;       // 解锁信息
      len = 12;
      break;
    case ANOTC_POWER:

      break;
    case ANOTC_CHECK:
      ANOTC_Buffer[2] = ANOTC_Check_PID;
      len = 2;
      break;
    default:
      break;
  }
  ANOTC_Buffer[0] = 0XAAAA;
  ANOTC_Buffer[1] = len | type << 8;
  pt[len + 4] = (int8_t)(0xAA + 0xAA);
  for (i = 2; i < len + 4; i += 2) {
    pt[i] ^= pt[i + 1];
    pt[i + 1] ^= pt[i];
    pt[i] ^= pt[i + 1];
    pt[len + 4] += pt[i] + pt[i + 1];
  }
  HAL_UART_Transmit(&huart1, pt, len + 5, 0xffff);
}

/**
 * @brief 轮询发送到上位机
 */
void ANOTC_Polling(void) {}