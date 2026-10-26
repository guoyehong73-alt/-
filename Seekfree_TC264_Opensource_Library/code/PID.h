/*
 * PID.h
 *
 *  Created on: 2026Äê9ÔÂ28ÈÕ
 *      Author: Guoyehong6
 */

#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "zf_common_headfile.h"

int PD_Camera(float err);
extern volatile int GYRO;
extern volatile float Yaw_Total;
extern volatile float Gyro_Z_Dps;

void Gyro_Init(void);
void Gyro_RecalibrateBias(void);
void Gyro_Update_5ms(void);

#endif /* CODE_PID_H_ */
