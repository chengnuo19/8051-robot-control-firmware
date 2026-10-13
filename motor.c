/**
 ******************************************************************************
 * @file    motor.c
 * @author  Antigravity
 * @brief   双路直流电机驱动控制 (支持正反转、差速调速)
 ******************************************************************************
 */

#include "motor.h"

// 电机全局占空比变量 (0~100)
u8 motor_l_duty = 0;
u8 motor_r_duty = 0;

/**
  * @brief  电机引脚初始化
  * @param  无
  * @retval 无
  */
void Motor_Init(void)
{
    // 初始化时所有引脚拉低，保证电机处于静止状态
    L_IN1 = 0; L_IN2 = 0; L_PWM = 0;
    R_IN1 = 0; R_IN2 = 0; R_PWM = 0;
    
    motor_l_duty = 0;
    motor_r_duty = 0;
}

/**
  * @brief  设置左、右电机速度与方向
  * @param  leftSpeed  左电机速度 (-100 ~ 100)
  * @param  rightSpeed 右电机速度 (-100 ~ 100)
  * @retval 无
  */
void Motor_SetSpeed(s16 leftSpeed, s16 rightSpeed)
{
    // 1. 限制速度幅値在 -100 到 100 之间
    if (leftSpeed  >  100) leftSpeed  =  100;
    if (leftSpeed  < -100) leftSpeed  = -100;
    if (rightSpeed >  100) rightSpeed =  100;
    if (rightSpeed < -100) rightSpeed = -100;
    
    // 2. 左轮方向与占空比控制
    if (leftSpeed > 0)
    {
        L_IN1 = 1; L_IN2 = 0;              // 前进方向
        EA = 0; motor_l_duty = (u8)leftSpeed; EA = 1;
    }
    else if (leftSpeed < 0)
    {
        L_IN1 = 0; L_IN2 = 1;              // 后退方向
        EA = 0; motor_l_duty = (u8)(-leftSpeed); EA = 1;
    }
    else
    {
        L_IN1 = 0; L_IN2 = 0;              // 滑行停止
        EA = 0; motor_l_duty = 0; EA = 1;
    }
    
    // 3. 右轮方向与占空比控制
    if (rightSpeed > 0)
    {
        R_IN1 = 1; R_IN2 = 0;              // 前进方向
        EA = 0; motor_r_duty = (u8)rightSpeed; EA = 1;
    }
    else if (rightSpeed < 0)
    {
        R_IN1 = 0; R_IN2 = 1;              // 后退方向
        EA = 0; motor_r_duty = (u8)(-rightSpeed); EA = 1;
    }
    else
    {
        R_IN1 = 0; R_IN2 = 0;              // 滑行停止
        EA = 0; motor_r_duty = 0; EA = 1;
    }
}

/**
  * @brief  小车紧急刹车 (先将PWM占空比设为0停转，再断开方向的3随资源引脚)
  * @note   对L298N：IN1=0,IN2=0是自由滑行剁车。若需短路强効厄车，将IN1=IN2=1（但让电机宕机制动允许时才用）。
  *         对TB6612：IN1=IN2=1 是剁车模式，比L298N更安全，需输入制动信号同时该模式小车会展示全功能剁车。
  *         【简单公式】：先寂PWM，再将IN1=IN2=0。对于手动推车正常来说，这就是最安全的溢出停止方式。
  */
void Motor_Brake(void)
{
    // 第一步: 先将PWM占空比清零 (防止其他轮达到全速给IN制造短路)
    EA = 0; motor_l_duty = 0; motor_r_duty = 0; EA = 1;
    // 第二步: 关闭方向控制，滑行停止 (小车明显应力，驱动把手数据安全)
    L_IN1 = 0; L_IN2 = 0;
    R_IN1 = 0; R_IN2 = 0;
}
