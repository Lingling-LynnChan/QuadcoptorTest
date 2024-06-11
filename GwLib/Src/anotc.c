#include "anotc.h"
#include <string.h>
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
    case ANOTC_RATE_PID:  // 速度环
      memcpy(ANOTC_PID_Cache.Rate, &pt[4], 18);
      ANOTC_Recived_flag.PID1 = 1;
      break;
    case ANOTC_ANGLE_PID:  // 角度环
      memcpy(ANOTC_PID_Cache.Angle, &pt[4], 18);
      ANOTC_Recived_flag.PID2 = 1;
      break;
    case ANOTC_HEIGHT_PID:  // 高度环
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
    case ANOTC_MPU_MAGIC: {
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
void ANOTC_Send(ANOTC_Type type) {
  uint8_t i;
  uint8_t len = 2;
  int16_t Anotc_Buffer[12];
  int8_t* pt = (int8_t*)(Anotc_Buffer);
  GW_PID_State_Type* pidX = NULL;
  GW_PID_State_Type* pidY = NULL;
  GW_PID_State_Type* pidZ = NULL;

  switch (type) {
    case ANOTC_RATE_PID:  // 发送PID1到上位机
      pidX = &pidRateX;   // 指定发送数据为角速度内环
      pidY = &pidRateY;
      pidZ = &pidRateZ;
      goto LABEL_SEND_PID;
    case ANOTC_ANGLE_PID:  // 发送PID2到上位机
      pidX = &pidRoll;
      pidY = &pidPitch;
      pidZ = &pidYaw;
      goto LABEL_SEND_PID;
    case ANOTC_HEIGHT_PID:  // 发送PID3到上位机
      goto LABEL_SEND_PID;
    case ANOTC_PID4:  // PID4
    case ANOTC_PID5:  // PID5
    case ANOTC_PID6:
    LABEL_SEND_PID:
      if (pidX != NULL) {
        Anotc_Buffer[2] = (int16_t)(pidX->kp * 1000);
        Anotc_Buffer[3] = (int16_t)(pidX->ki * 1000);
        Anotc_Buffer[4] = (int16_t)(pidX->kd * 1000);
      }
      if (pidY != NULL) {
        Anotc_Buffer[5] = (int16_t)(pidY->kp * 1000);
        Anotc_Buffer[6] = (int16_t)(pidY->ki * 1000);
        Anotc_Buffer[7] = (int16_t)(pidY->kd * 1000);
      }
      if (pidZ != NULL) {
        Anotc_Buffer[8] = (int16_t)(pidZ->kp * 1000);
        Anotc_Buffer[9] = (int16_t)(pidZ->ki * 1000);
        Anotc_Buffer[10] = (int16_t)(pidZ->kd * 1000);
      }
      len = 18;
      break;
    case ANOTC_MOTOR:
      Anotc_Buffer[2] = 0;
      Anotc_Buffer[3] = 0;
      Anotc_Buffer[4] = 0;
      Anotc_Buffer[5] = 0;
      Anotc_Buffer[6] = 0;  // 最大可支持8组PWM上传上位机
      Anotc_Buffer[7] = 0;
      Anotc_Buffer[8] = 0;
      Anotc_Buffer[9] = 0;
      len = 16;
      break;
    case ANOTC_RCDATA:  // send RC data
      Anotc_Buffer[2] = Remote.thr;
      Anotc_Buffer[3] = Remote.yaw;
      Anotc_Buffer[4] = Remote.roll;
      Anotc_Buffer[5] = Remote.pitch;
      Anotc_Buffer[6] = Remote.AUX1;
      Anotc_Buffer[7] = Remote.AUX2;
      Anotc_Buffer[8] = Remote.AUX3;
      Anotc_Buffer[9] = Remote.AUX4;
      Anotc_Buffer[10] = Remote.AUX3;
      Anotc_Buffer[11] = Remote.AUX4;
      len = 20;
      break;
    case ANOTC_MPU_MAGIC:  // 发送MPU6050和磁力计的数据
      memcpy(&Anotc_Buffer[2], (int8_t*)&MPU6050, sizeof(_st_Mpu));
      Anotc_Buffer[8] = flow_data.vel_y;  // 没有磁力计传感器
      Anotc_Buffer[9] = flow_y_lpf_att_i;
      Anotc_Buffer[10] = 0;
      len = 18;
      break;
    case ANOTC_SENSER2:

      break;
    case ANOTC_STATUS:  // send angle
      Anotc_Buffer[2] = (int16_t)(Angle.roll * 100);
      Anotc_Buffer[3] = (int16_t)(Angle.pitch * 100);
      Anotc_Buffer[4] = -(int16_t)(Angle.yaw * 100);
      Anotc_Buffer[5] = 0;  // 没有高度数据
      Anotc_Buffer[6] = 0;
      Anotc_Buffer[7] = ALL_flag.unlock << 8;  // 解锁信息
      len = 12;
      break;
    case ANOTC_POWER:

      break;
    case ANOTC_CHECK:
      Anotc_Buffer[2] = ANOTC_Check_PID;
      len = 2;
      break;
    default:
      break;
  }

  Anotc_Buffer[0] = 0XAAAA;
  Anotc_Buffer[1] = len | type << 8;
  pt[len + 4] = (int8_t)(0xAA + 0xAA);
  for (i = 2; i < len + 4; i += 2)  // a swap with b;
  {
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