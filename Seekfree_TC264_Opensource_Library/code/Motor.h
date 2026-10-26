/*
 * Motor.h
 *
 *  Created on: 2026年9月26日
 *      Author: Guoyehong6
 */

#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_

#include "zf_common_headfile.h"

#define  MotorL_pwm    ATOM1_CH4_P21_6
#define  MotorL_turn   (P21_5)

#define  MotorR_pwm    ATOM1_CH2_P21_4
#define  MotorR_turn   (P21_3)

#define SPEED_MAX  4000
#define SPEED_MIN  -4000

void Motor_Init(void);
void MotorL_Set_Speed(int pwm);
void MotorR_Set_Speed(int pwm);
/* 差速转向：
 * base_pwm：左右轮基础PWM；
 * err：摄像头中线误差；
 * 返回值：左右轮速度差，正值表示左慢右快（向左转）；
 * 内部使用菜单里的 Diff Gain / Diff Limit / Diff Rate。 */
int Motor_Diff(int base_pwm, float err);

/* 复位差速控制器内部状态：停车、环岛、直道快速分支、进菜单时调用 */
void Motor_Diff_Reset(void);

#endif /* CODE_MOTOR_H_ */
