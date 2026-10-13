/**
 ******************************************************************************
 * @file    timer.h
 * @author  Antigravity
 * @brief   定时器 0 基准时间管理及软件 PWM 驱动
 ******************************************************************************
 */

#ifndef __TIMER_H__
#define __TIMER_H__

#include "config.h"

// 定时器0初始化，配置为100us中断一次
void Timer0_Init(void);

// 获取系统自开机以来的毫秒数 (用于精确延时或防死锁)
u32 Timer_GetMs(void);

#endif /* __TIMER_H__ */
