/*
 * Motor.c
 *
 *  Created on: 2026年9月26日
 *      Author: Guoyehong6
 */
#include "Motor.h"

void Motor_Init(void)
{
    gpio_init(MotorL_turn, GPO, 1, GPO_PUSH_PULL);
    pwm_init(MotorL_pwm, 5000, 0);
    gpio_init(MotorR_turn, GPO, 1, GPO_PUSH_PULL);
    pwm_init(MotorR_pwm, 5000, 0);
}

void MotorL_Set_Speed(int pwm)
{
    if(pwm>=SPEED_MAX)        //限幅处理
       pwm=SPEED_MAX;
    else if(pwm<=SPEED_MIN)
       pwm=SPEED_MIN;

    if(pwm>=0)
    {
        gpio_set_level(MotorL_turn, 0);
        pwm_set_duty(MotorL_pwm, pwm);
    }
    else
    {
        gpio_set_level(MotorL_turn, 1);
        pwm_set_duty(MotorL_pwm, -pwm);
    }
}

void MotorR_Set_Speed(int pwm)
{
    if(pwm>=SPEED_MAX)        //限幅处理
            pwm=SPEED_MAX;
    else if(pwm<=SPEED_MIN)
            pwm=SPEED_MIN;

    if(pwm>=0)
    {
        gpio_set_level(MotorR_turn, 1);
        pwm_set_duty(MotorR_pwm, pwm);
    }
    else
    {
        gpio_set_level(MotorR_turn, 0);
        pwm_set_duty(MotorR_pwm, -pwm);
    }
}
/* 差速控制器内部状态（只给 Motor_Diff 使用）：
 * diff_error_last：上一帧的摄像头误差 Err，用来计算 D 项 (Err - 上一帧Err)；
 * diff_last：上一帧真正输出的差速量，用来做每帧变化率限制；
 * 这两个变量在 Motor_Diff_Reset() 里清零。 */
static float diff_error_last = 0.0f;
static int diff_last = 0;

/* 复位差速控制器：停车、环岛、直道快速分支以及切换控制模式时调用，
 * 避免下一次进入差速分支时从旧值开始变化 */
void Motor_Diff_Reset(void)
{
    diff_error_last = 0.0f;
    diff_last = 0;
}

/* 差速计算：
 * base_pwm 是左右轮的基础PWM；
 * err 是摄像头中线误差；
 * 返回值是左右轮速度差，正值表示左慢右快（向左转）。
 *
 * DG = Diff Gain  差速增益系数：1.00 = 原来的×12强度，0.50 = 一半，0.00 = 关闭差速；
 * DM = Diff Limit 差速绝对限幅，单位PWM；
 * DS = Diff Rate  每帧最大变化量，单位PWM/帧，0表示不做变化率限制。
 */
int Motor_Diff(int base_pwm, float err)
{
    /* error_now：本帧摄像头误差 Err
     * error_delta：本帧误差 - 上一帧误差，用于计算D项
     * diff_cmd：经过 Diff Gain 缩放后的目标差速量
     * max_diff：最终允许的最大差速量
     * base_limit：基础PWM的45%限制，防止内轮掉速太多
     * speed_limit：受 SPEED_MAX 限制后允许的最大差速量
     * slew：菜单里的 Diff Rate，每帧最大变化量
     * diff：最终返回给左右轮的差速值 */
    float error_now = err;
    float error_delta = error_now - diff_error_last;
    float diff_cmd;
    float max_diff;
    float base_limit;
    float speed_limit;
    int slew;
    int diff;

    /* 记录本次误差，供下一帧计算D项 */
    diff_error_last = error_now;

    /* 基础PWM保护，防止菜单误设超范围 */
    if(base_pwm < 0)
        base_pwm = 0;
    else if(base_pwm > SPEED_MAX)
        base_pwm = SPEED_MAX;

    /* 原始PD，保留原来的12倍关系，再由DG整体缩放 */
    diff_cmd = (Menu_GetKp() * error_now
              + Menu_GetKd() * error_delta) * 12.0f;
    diff_cmd *= Menu_GetDiffGain();

    /* 限幅1：菜单设置的绝对上限 */
    max_diff = (float)Menu_GetDiffMax();

    /* 限幅2：最多只允许基础PWM的45%，防止内轮掉速过多 */
    base_limit = base_pwm * 0.45f;
    if(max_diff > base_limit)
        max_diff = base_limit;

    /* 限幅3：外轮不能超过SPEED_MAX */
    speed_limit = (float)(SPEED_MAX - base_pwm);
    if(max_diff > speed_limit)
        max_diff = speed_limit;

    if(diff_cmd > max_diff)
        diff_cmd = max_diff;
    else if(diff_cmd < -max_diff)
        diff_cmd = -max_diff;

    /* 变化率限制：每帧最多变化DS个PWM，DS=0时不限制 */
    slew = Menu_GetDiffSlew();
    if(slew <= 0)
    {
        diff = (int)diff_cmd;
    }
    else
    {
        if(diff_cmd - diff_last > slew)
            diff = diff_last + slew;
        else if(diff_cmd - diff_last < -slew)
            diff = diff_last - slew;
        else
            diff = (int)diff_cmd;
    }

    diff_last = diff;
    return diff;
}
