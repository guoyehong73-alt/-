/*
 * control.h
 *
 *  Created on: 2026年9月23日
 *      Author: Guoyehong6
 */

#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_

#include "zf_common_headfile.h"

#define SERVO_MOTOR_PWM             (ATOM0_CH1_P33_9)                           // 定义主板上舵机对应引脚
#define SERVO_MOTOR_FREQ            (50 )                                       // 定义主板上舵机频率  请务必注意范围 50-300

#define SERVO_PWMMID    504
#define SERVO_PWM_MAX   586
#define SERVO_PWM_MIN   422

extern int out_of_track_flag;

void Servo_Init(void);
int Servo_ControlPwm(float err);
float Err_Sum(void);
void Out_Of_Track_Detect(void);

#endif /* CODE_CONTROL_H_ */
