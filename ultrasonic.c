/**
 ******************************************************************************
 * @file    ultrasonic.c
 * @author  Antigravity
 * @brief   HC-SR04 超声波测距驱动 (软件精细延时测距，带防锁死超时机制)
 ******************************************************************************
 */

#include "ultrasonic.h"

/**
  * @brief  超声波模块引脚初始化
  * @param  无
  * @retval 无
  */
void Ultrasonic_Init(void)
{
    TRIG = 0;
    ECHO = 1; // 51单片机准双向口输入前需置1
}

/**
  * @brief  10微秒精细延时 (用于触发信号和测距校准)
  * @param  无
  * @retval 无
  */
static void Ultrasonic_Delay10us(void)
{
    #ifdef OSC_FREQ_110592
    // 11.0592MHz 延时 10us
    unsigned char i = 2;
    while (--i);
    #else
    // 12.0000MHz 延时 10us
    unsigned char i = 3;
    while (--i);
    #endif
}

/**
  * @brief  获取当前超声波测距值 (单位: 厘米)
  * @param  无
  * @retval 测距结果 (单位: cm)。若返回 999 代表测量超时或未连接。
  */
u16 Ultrasonic_GetDistance(void)
{
    u16 count = 0;
    u16 timeout = 0;
    u16 distance = 0;
    
    // 1. 发送触发信号：Trig 给至少 10us 的高电平
    TRIG = 1;
    Ultrasonic_Delay10us();
    Ultrasonic_Delay10us(); // 保证足够的时间
    TRIG = 0;
    
    // 2. 等待 Echo 引脚变高电平 (带超时退出，防止死锁卡死系统)
    timeout = 0;
    while (ECHO == 0)
    {
        timeout++;
        if (timeout > 5000) 
        {
            return 999; // 超时退出，说明探头异常或前方无障碍物
        }
    }
    
    // 3. 测量 Echo 引脚高电平持续时间
    // 在这个循环中，每次循环耗时约为 10us (包含循环判断、计数自增和延时函数)
    // 10us 声波往返距离 = 10us * 0.034cm/us / 2 = 0.17 厘米
    count = 0;
    while (ECHO == 1 && count < 2500) // 2500次循环对应最大量程约 4.2 米
    {
        count++;
        Ultrasonic_Delay10us(); // 每次循环延时10us
    }
    
    // 4. 计算距离 (厘米)
    // 距离 = 计数次数 * 0.17 厘米
    // 为避免浮点运算（51算浮点极慢），改写为整数运算：距离 = (count * 17) / 100
    distance = (count * 17) / 100;
    
    // 限制最小有效量程，HC-SR04盲区大概是2cm
    if (distance < 2) distance = 2;
    
    return distance;
}
