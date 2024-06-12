#include "task.h"
#include "gwmath.h"
#include "icm20608.h"
#include "imu.h"
#include "pid.h"
#include "state.h"

void GW_Task_1ms() {
  // TODO: 定时器
  static uint16_t t3ms = 0;
  static uint16_t t10ms = 0;
  if (++t3ms >= 3) {
    t3ms = 0;
    MPU_Get_And_Filter();
    IMU_Get_Angle(0.003f);
  }
  if (++t10ms >= 10) {
    t10ms = 0;
    printf("Roll: %f, Pitch: %f, Yaw: %f\n", GWS_IMU_Angle.Roll,
           GWS_IMU_Angle.Pitch, GWS_IMU_Angle.Yaw);
  }
}

// TODO: 丢到PWM里
typedef struct {
  uint16_t Motor1;
  uint16_t Motor2;
  uint16_t Motor3;
  uint16_t Motor4;
} GW_Motor_Pwm_Type;
// TODO: 丢到PWM里
GW_Motor_Pwm_Type GWS_Motors;

GW_PID_State_Type GWS_PID;

void GW_Ctrl_Flight(float dt) {
  // 状态列表
  static enum {
    LS_FLY_STATE_WAIT_UNLOCK,    // 等待解锁
    LS_FLY_STATE_BEFORE_UNLOCK,  // 准备解锁
    LS_FLY_STATE_IDLE,           // 正常运行
    LS_FLY_STATE_EXIT,           // 退出
  } status = LS_FLY_STATE_WAIT_UNLOCK;
  // 状态机
  switch (status) {
    case LS_FLY_STATE_WAIT_UNLOCK: {
      if (GWS_Lock == GWS_FLY_UNLOCK) {
        status = LS_FLY_STATE_BEFORE_UNLOCK;
      }
      break;
    }
    case LS_FLY_STATE_BEFORE_UNLOCK: {
      GW_PID_Reset();
      // 锁定偏航角
      GWS_IMU_Angle.Yaw = GWS_PID.Yaw.Desired = GWS_PID.Yaw.Measured = 0;
      status = LS_FLY_STATE_IDLE;
      break;
    }
    case LS_FLY_STATE_IDLE: {
      if (GWS_Remote.Accelerator >= 1200 ||      // 油门大于1200 且
          (GWM_Abs(GWS_IMU_Angle.Pitch) > 50 ||  // (俯仰角大于50 或
           GWM_Abs(GWS_IMU_Angle.Roll) > 50)     // 横滚角大于50)
      ) {                                        // 失控锁定
        GWS_Lock = GWS_FLY_LOCK;
        status = LS_FLY_STATE_EXIT;
        break;
      }
      // 内环测量值
      GWS_PID.RateX.Measured = GWS_MPU6050.Gyro.X * IMU_GYRO_G;  // X轴角速度r/s
      GWS_PID.RateY.Measured = GWS_MPU6050.Gyro.Y * IMU_GYRO_G;  // Y轴角速度r/s
      GWS_PID.RateZ.Measured = GWS_MPU6050.Gyro.Z * IMU_GYRO_G;  // Z轴角速度r/s
      // 外环测量值
      GWS_PID.Pitch.Measured = GWS_IMU_Angle.Pitch;  // 俯仰角r
      GWS_PID.Roll.Measured = GWS_IMU_Angle.Roll;    // 横滚角r
      GWS_PID.Yaw.Measured = GWS_IMU_Angle.Yaw;      // 偏航角r
      // 更新PID
      struct {
        GW_PID_Type* InnerRing;  // 内环
        GW_PID_Type* OuterRing;  // 外环
      } LOCAL_PID_ARRAY[3] = {
          {&GWS_PID.RateX, &GWS_PID.Roll},
          {&GWS_PID.RateY, &GWS_PID.Pitch},
          {&GWS_PID.RateZ, &GWS_PID.Yaw},
      };
      for (uint8_t i = 0; i < 3; i++) {
        GW_PID_Type* InnerRing = LOCAL_PID_ARRAY[i].InnerRing;
        GW_PID_Type* OuterRing = LOCAL_PID_ARRAY[i].OuterRing;
        // 更新外环
        GW_PID_Update(OuterRing, dt);
        InnerRing->Desired = OuterRing->Out;
        // 更新内环
        GW_PID_Update(InnerRing, dt);
      }
      break;
    }
    case LS_FLY_STATE_EXIT:
    default:
      GW_PID_Reset();
      status = LS_FLY_STATE_WAIT_UNLOCK;
      break;
  }
}

void GW_Ctrl_Motor(void) {
  // 状态列表
  static enum {
    LS_MOTOR_STATE_WAIT_UNLOCK,  // 等待解锁
    LS_MOTOR_STATE_BEFORE_FLY,   // 准备飞行
    LS_MOTOR_STATE_IDLE,         // 正常飞行
    LS_MOTOR_STATE_EXIT,         // 退出
  } status = LS_MOTOR_STATE_WAIT_UNLOCK;
  // 失控锁定
  if (GWS_Lock == GWS_FLY_LOCK) {
    status = LS_MOTOR_STATE_EXIT;
  }
  // 状态机
  switch (status) {
    case LS_MOTOR_STATE_WAIT_UNLOCK: {
      GWS_Motors.Motor1 = 0;
      GWS_Motors.Motor2 = 0;
      GWS_Motors.Motor3 = 0;
      GWS_Motors.Motor4 = 0;
      if (GWS_Lock == GWS_FLY_UNLOCK) {
        status = LS_MOTOR_STATE_BEFORE_FLY;
      }
      break;
    }
    case LS_MOTOR_STATE_BEFORE_FLY: {
      if (GWS_Remote.Accelerator > 1100) {
        status = LS_MOTOR_STATE_IDLE;
      }
      break;
    }
    case LS_MOTOR_STATE_IDLE: {
      // 油门调整
      // int16_t temp = GWS_Remote.Accelerator - 1000;
      // int16_t thr = 200 + 0.4f * temp;
      // TODO: 油门还没写
      break;
    }
    case LS_MOTOR_STATE_EXIT:
    default:
      status = LS_MOTOR_STATE_WAIT_UNLOCK;
      break;
  }
}