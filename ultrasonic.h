/**
 ******************************************************************************
 * @file    ultrasonic.h
 * @author  Antigravity
 * @brief   HC-SR04 超声波测距驱动头文件
 ******************************************************************************
 */

#ifndef __ULTRASONIC_H__
#define __ULTRASONIC_H__

#include "config.h"

// 超声波引脚初始化
void Ultrasonic_Init(void);

// 获取当前障碍物距离，单位：厘米 (cm)
// 带有超时保护，不会因为没有探头或连接断开而导致程序卡死。
u16 Ultrasonic_GetDistance(void);

#endif /* __ULTRASONIC_H__ */
