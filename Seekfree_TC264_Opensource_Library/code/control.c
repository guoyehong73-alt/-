/*
 * control.c
 *
 *  Created on: 2026年9月23日
 *      Author: Guoyehong6
 */
#include "control.h"
#include "PID.h"
/*-------------------------------------------------------------------------------------------------------------------
  @brief     摄像头误差获取
  @param     null
  @return    获取到的误差
  Sample     err=Err_Sum();
  @note      加权取平均
-------------------------------------------------------------------------------------------------------------------*/
// 适配120行高度的权重数组（下标0=画面最顶端，119=画面最底端）
//const uint8 Weight[MT9V03X_H] = {
//    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // 00-09  远端平坦区
//    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // 10-19
//    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // 20-29
//    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,  // 30-39  权重上升段（整体下移1行）
//    11,12,13,14,16,17,19,20,20,20,  // 40-49  进入峰值区
//    21,21,20,20,20,19,18,17,16,14,  // 50-59  峰值后开始下降
//    13,12,11,10,9, 8, 7, 6, 5, 4,  // 60-69  权重下降段
//    3, 2, 1, 1, 1, 1, 1, 1, 1, 1,  // 70-79
//    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // 80-89  近端平坦区
//    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // 90-99
//    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // 100-109
//    1, 1, 1, 1, 1, 1, 1, 1, 1, 1   // 110-119 画面最底端
//};
const uint8 Weight[MT9V03X_H] = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // 00-09  远端平坦区
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // 10-19
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // 20-29
    1, 1, 1, 1, 1, 1, 2, 3, 4, 5,  // 30-39  权重上升段（整体下移1行）
    7, 9,10,12,14,16,17,19,20,20,  // 40-49  进入峰值区
   20,20,20,20,20,19,18,17,16,14,  // 50-59  峰值后开始下降
   12,10, 9, 7, 5, 4, 3, 2, 1, 1,  // 60-69  权重下降段
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // 70-79
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // 80-89  近端平坦区
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // 90-99
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // 100-109
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1   // 110-119 画面最底端
};


float Err_Sum(void)
{
    int i;
    float err=0;
    float weight_count=0;
    //常规误差
    for(i=MT9V03X_H-1;i>=MT9V03X_H-Search_Stop_Line-1;i--)//常规误差计算
    {
        err+=(MT9V03X_W/2-((Left_Line[i]+Right_Line[i])>>1))*Weight[i];   //右移1位，等效除2
        weight_count+=Weight[i];
    }
    err=err/weight_count;

    return err;
}

void Servo_Init(void)  //舵机初始化
{
    pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, SERVO_PWMMID);
}

static int Servo_LimitPwm(int pwm)
{
    if(pwm < SERVO_PWM_MIN)
        return SERVO_PWM_MIN;
    if(pwm > SERVO_PWM_MAX)
        return SERVO_PWM_MAX;
    return pwm;
}

int Servo_ControlPwm(float err)
{
    static int hold_pwm = SERVO_PWMMID;
    static int state4_buffer[5] = {0};
    static int state4_sum = 0;
    static uint8 state4_index = 0;
    static uint8 state4_count = 0;
    static uint8 last_island_state = 0;
    uint8 current_island_state = (uint8)Island_State;
    int output;

    if(current_island_state == 5)
    {
        /* 状态5保持状态4最近5帧的平均舵机角，只更新PD内部误差历史 */
        (void)PD_Camera(err);
        last_island_state = current_island_state;
        hold_pwm = Servo_LimitPwm(hold_pwm);
        return hold_pwm;
    }

    output = PD_Camera(err);
    if(current_island_state == 4)
    {
        if(last_island_state != 4)
        {
            state4_sum = 0;
            state4_index = 0;
            state4_count = 0;
            state4_buffer[0] = 0;
            state4_buffer[1] = 0;
            state4_buffer[2] = 0;
            state4_buffer[3] = 0;
            state4_buffer[4] = 0;
        }

        state4_sum -= state4_buffer[state4_index];
        state4_buffer[state4_index] = output;
        state4_sum += output;
        state4_index++;
        if(state4_index >= 5)
            state4_index = 0;
        if(state4_count < 5)
            state4_count++;

        hold_pwm = Servo_LimitPwm(state4_sum / state4_count);
    }
    else if(current_island_state != 5)
    {
        state4_sum = 0;
        state4_index = 0;
        state4_count = 0;
    }

    last_island_state = current_island_state;
    return Servo_LimitPwm(output);
}

extern int out_of_track_flag=0;
void Out_Of_Track_Detect(void)     //越界保护
{
    int count=0;
    static int n=0;
    for(int i=MT9V03X_H-1;i>=MT9V03X_H-6;i--)
    {
        for(int j=0;j<MT9V03X_W;j++)
        {
            if(image_two_value[i][j]==IMG_BLACK)
            {
                count++;
            }
        }
    }
    if(count>1100)
    {
        n++;
        if(n>3)
        {
            out_of_track_flag=1;
        }

    }
    else
    {
        n=0;
    }
}

