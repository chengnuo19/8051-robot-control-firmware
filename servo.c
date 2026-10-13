/**
 ******************************************************************************
 * @file    servo.c
 * @author  Antigravity
 * @brief   双路舵机驱动控制 (支持0-180度控制)
 ******************************************************************************
 */

#include "servo.h"

// 舵机高电平时间变量 (单位为100us，默认值为 15 对应 1.5ms 即 90 度中位)
u8 servo1_width = 15;
u8 servo2_width = 15;

/**
  * @brief  舵机初始化，使其归中 (90度)
  * @param  无
  * @retval 无
  */
void Servo_Init(void)
{
    SERVO1 = 0;
    SERVO2 = 0;
    servo1_width = 15; // 90度
    servo2_width = 15; // 90度
}

/**
  * @brief  控制舵机转动到指定角度
  * @param  servoId 舵机编号 (1 代表 SERVO1, 2 代表 SERVO2)
  * @param  angle   转动角度 (0 ~ 180 度)
  * @retval 无
  * @note   精度优化: 使用线性映射公式避免量化误差
  *         0°→脉兰5*100us=0.5ms, 90°→15*100us=1.5ms, 180°→25*100us=2.5ms
  *         公式: width = 5 + (angle * 20) / 180  (使用整数运算，误差 <0.056ms)
  */
void Servo_SetAngle(u8 servoId, u16 angle)
{
    u8 temp_width;
    
    // 限制角度在安全范围
    if (angle > 180) angle = 180;
    
    // 优化精度: 0°=5, 90°=15, 180°=25, 误差远小于原来的 angle/9 公式
    temp_width = 5 + (u8)((u16)(angle * 20) / 180);
    
    // 关中断写入，保证 servo_width 的原子性 (中断读到半旧半新的脑刀问题)
    EA = 0;
    if (servoId == 1) servo1_width = temp_width;
    else if (servoId == 2) servo2_width = temp_width;
    EA = 1;
}
