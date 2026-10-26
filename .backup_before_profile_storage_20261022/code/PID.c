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
volatile float Yaw_Total = 0;//总偏移量
volatile float Gyro_Z_Dps = 0;

static float gyro_bias = 0;//零通
static float gyro_lpf = 0;//低通滤波

#define GYRO_CAL_SAMPLES        (100)
#define GYRO_CAL_PERIOD_MS      (5)

static void Gyro_CalibrateBias(void)
{
    uint16 i;
    float sum = 0;
    uint32 interrupt_state;

    interrupt_state = interrupt_global_disable();
    for(i = 0; i < GYRO_CAL_SAMPLES; i++)
    {
        imu660ra_get_gyro();
        sum += imu660ra_gyro_transition(imu660ra_gyro_z);
        system_delay_ms(GYRO_CAL_PERIOD_MS);
    }

    gyro_bias = sum / (float)GYRO_CAL_SAMPLES;
    gyro_lpf = 0;
    Yaw_Total = 0;
    Gyro_Z_Dps = 0;
    interrupt_global_enable(interrupt_state);
}

void Gyro_Init(void)
{
    imu660ra_init();
    Gyro_CalibrateBias();
}

void Gyro_RecalibrateBias(void)
{
    Gyro_CalibrateBias();
}

void Gyro_Update_5ms(void)
{
    float gyro_z_dps=0;

    imu660ra_get_gyro();

    gyro_z_dps = imu660ra_gyro_transition(imu660ra_gyro_z);
    gyro_z_dps -= gyro_bias;

    if(fabs(gyro_z_dps) < 2.0f)
    {
        gyro_z_dps = 0.0f;
    }

    // 低通滤波
   // gyro_lpf = gyro_lpf * 0.70f + gyro_z_dps * 0.30f;//gyro_z_lpf = 上一次滤波值 * 0.9 + 当前值 * 0.1

   // Gyro_Z_Dps = gyro_lpf;

    // 5ms积分一次，单位变成“度”
    // 只在环岛3/4状态积分，避免平时静止时Yaw_Total一直漂
    if(Island_State >= 3)
    {
        Yaw_Total += gyro_z_dps * 0.005f;
    }
    //Yaw_Total += gyro_z_dps * 0.005f;
}
int PD_Camera(float err)//舵机PD调节
{
   int out;
   float  P=Menu_GetKp();
   float  D=Menu_GetKd();
   float  Island_P=Menu_GetIsland_P();
   float  Island_D=Menu_GetIsland_D();
   float  Kgyro=Menu_GetKgyro();
   volatile static float error_current,error_last;
   float er,er1;
   float gyro_z_dps=0;
   gyro_z_dps=imu660ra_gyro_transition(imu660ra_gyro_z);
   GYRO=(int)gyro_z_dps;
   error_current=err;
   er=error_current;
   er1=error_current-error_last;
  if(Island_State!=0)//环岛单独的pd，其他阶段都是模糊控制
   {
       P=Island_P;
       D=Island_D;
   }
      out=P*er+D*er1+SERVO_PWMMID;
    //out=P*er+D*er1-Kgyro*gyro_z_dps+SERVO_PWMMID;
    error_last=error_current;

    if(out>=SERVO_PWM_MAX)//限幅处理
        out=SERVO_PWM_MAX;
    else if(out<=SERVO_PWM_MIN)//限幅处理
        out=SERVO_PWM_MIN;
    return (int)out;
}

