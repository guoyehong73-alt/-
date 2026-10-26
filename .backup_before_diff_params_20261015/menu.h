/*********************************************************************************************************************
* 文件名称：menu.h
* 功能说明：单页参数菜单接口
* 参数内容：P、D、电机PWM
* 保存说明：参数只保存在RAM中，掉电后恢复默认值
********************************************************************************************************************/

#ifndef CODE_MENU_H_
#define CODE_MENU_H_

#include "zf_common_headfile.h"

// Err是CPU1计算出的摄像头中线误差
extern volatile float Err;
extern uint8 menu_page;
extern volatile int menu_motor_pwm;

// 菜单二级页面编号（cpu0_main 里也要用它判断是不是图像页/数据页）
// 二级页面编号
#define MENU_SUB_LIST            (0)                // 一级菜单（页面列表）
#define MENU_SUB_IMAGE           (1)                // 二值化图像页面
#define MENU_SUB_PARAM           (2)                // 参数调整页面
#define MENU_SUB_DATA            (3)                // 图像数据显示页面
#define MENU_SUB_START           (4)                // 发车倒计时页面
#define MENU_SUB_MOTOR           (5)                // 电机PWM设置页面

extern uint8 menu_sub;

// 发车标志：1=已发车，车才会跑；0=没发车，车停着
extern uint8 race_started;

// 初始化菜单、按键和屏幕
void Menu_Init(void);

// 扫描按键并更新菜单
void Menu_Update(void);


// 返回当前菜单中的P参数
float Menu_GetKp(void);

// 返回当前菜单中的D参数
float Menu_GetKd(void);

float Menu_GetIsland_P(void);

float Menu_GetIsland_D(void);

// 返回当前菜单中的Kgyro参数
float Menu_GetKgyro(void);

// 返回菜单中的电机基础PWM
int Menu_GetMotorPwm(void);

// 返回左环岛电机PWM
int Menu_GetLeftIslandPwm(void);

// 返回右环岛电机PWM
int Menu_GetRightIslandPwm(void);

// 返回高速直道电机PWM
int Menu_GetFastPwm(void);

#endif /* CODE_MENU_H_ */
