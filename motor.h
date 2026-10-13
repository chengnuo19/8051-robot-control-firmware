/**
 ******************************************************************************
 * @file    motor.h
 * @author  Antigravity
 * @brief   双路直流电机驱动控制头文件 (前进、后退、转向与差速控制)
 ******************************************************************************
 */

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "config.h"

// 初始化电机GPIO管脚 (初始设为静止)
void Motor_Init(void);

// 设置左、右电机的转速与方向
// speed: 范围 -100 到 100 (负数代表后退，正数代表前进，0代表停止)
void Motor_SetSpeed(s16 leftSpeed, s16 rightSpeed);

// 刹车控制
void Motor_Brake(void);

#endif /* __MOTOR_H__ */
