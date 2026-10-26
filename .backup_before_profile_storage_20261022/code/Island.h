/*
 * Insland.h
 *
 *  Created on: 2026Äê9ÔÂ23ÈÕ
 *      Author: Guoyehong6
 */

#ifndef CODE_ISLAND_H_
#define CODE_ISLAND_H_

#include "zf_common_headfile.h"

extern volatile int Left_Island_Flag;
extern volatile int Right_Island_Flag;

void Island_Reset(void);

int Monotonicity_Change_Left(int start,int end);
int Monotonicity_Change_Right(int start,int end);
int Continuity_Change_Left(int start,int end);
int Continuity_Change_Right(int start,int end);
//void K_Add_Boundry_Left(float k,int startX,int startY,int endY);
//void K_Add_Boundry_Right(float k,int startX,int startY,int endY);
void Island_Detect(void);

#endif /* CODE_ISLAND_H_ */
