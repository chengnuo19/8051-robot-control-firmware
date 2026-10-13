/**
 ******************************************************************************
 * @file    servo.h
 * @author  Antigravity
 * @brief   双路舵机驱动控制头文件 (机械臂/云台)
 ******************************************************************************
 */

#ifndef __SERVO_H__
#define __SERVO_H__

#include "config.h"

// 舵机初始化 (初始化为中位 90 度)
void Servo_Init(void);

// 设置舵机角度
// servoId: 1 或 2
// angle: 范围 0 到 180 度 (0.5ms~2.5ms高电平时间自适应转换)
void Servo_SetAngle(u8 servoId, u16 angle);

#endif /* __SERVO_H__ */
