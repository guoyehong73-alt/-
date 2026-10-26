/*
 * PID.c
 *
 *  Created on: 2026年9月28日
 *      Author: Guoyehong6
 */

#include "PID.h"
#include "control.h"
#include "menu.h"

/*-------------------------------------------------------------------------------------------------------------------
  @brief     PD摄像头控制
  @param     err
  @return    舵机打角值
  Sample     Steer_Angle=PD_Camera(Err);//舵机PD调
  @note      null
-------------------------------------------------------------------------------------------------------------------*/
volatile int GYRO=0;
int PD_Camera(float err)//舵机PD调节
{
   int out;
   float  P=Menu_GetKp();
   float  D=Menu_GetKd();
   float  Kgyro=Menu_GetKgyro();
   volatile static float error_current,error_last;
   float er,er1;
   float gyro_z_dps=0;
   imu660ra_get_gyro();
   gyro_z_dps=imu660ra_gyro_transition(imu660ra_gyro_z);
   GYRO=(int)gyro_z_dps;
   error_current=err;
   er=error_current;
   er1=error_current-error_last;
//   if(Go==7)//常规控制
//   {
//       DuoJi_GetP(&P ,MT9V03X_H-Search_Stop_Line, err);//新型模糊P,P的地址，搜索截止行的位置，误差
//       D=Global_D;
//       if(Island_State!=0)//环岛单独的pd，其他阶段都是模糊控制
//       {
//           P=Island_P;
//           D=Island_D;
//       }
//   }
//    out=P*er+D*er1+SERVO_PWMMID;
    out=P*er+GYRO*0.33f+SERVO_PWMMID;
    error_last=error_current;

    if(out>=SERVO_PWM_MAX)//限幅处理
        out=SERVO_PWM_MAX;
    else if(out<=SERVO_PWM_MIN)//限幅处理
        out=SERVO_PWM_MIN;
    return (int)out;
}

