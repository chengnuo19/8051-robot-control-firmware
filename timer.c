/**
 ******************************************************************************
 * @file    timer.c
 * @author  Antigravity
 * @brief   定时器 0 驱动及软 PWM 中断服务程序
 ******************************************************************************
 */

#include "timer.h"

// 声明外部电机占空比变量 (在 motor.c 中定义)
extern u8 motor_l_duty;
extern u8 motor_r_duty;

// 声明外部舵机高电平时间变量 (在 servo.c 中定义，单位为100us，范围 5~25 代表 0.5ms~2.5ms)
extern u8 servo1_width;
extern u8 servo2_width;

// 全局毫秒计数器
static volatile u32 sys_ms = 0;

// 定时器0重载值定义 (100us中断基准)
// 计算公式: 65536 - (FOSC / 12 / 10000)
// 11.0592MHz: 65536 - (11059200/12/10000) = 65536 - 92 = 65444 = 0xFF A4 ← 原代码数值正确但注释错误
// 12.000MHz:  65536 - (12000000/12/10000) = 65536 - 100 = 65436 = 0xFF 9C ← 正确
#ifdef OSC_FREQ_110592
    #define TIMER0_RELOAD   (65536 - 92)   // 11.0592MHz 下 100us 精确重载值
#else
    #define TIMER0_RELOAD   (65536 - 100)  // 12MHz 下 100us 精确重载值
#endif
#define TIMER0_RELOAD_H  ((TIMER0_RELOAD) >> 8)
#define TIMER0_RELOAD_L  ((TIMER0_RELOAD) & 0xFF)

/**
  * @brief  定时器0初始化配置 (100us中断一次)
  * @param  无
  * @retval 无
  */
void Timer0_Init(void)
{
    // 不影响定时器1的配置 (串口要用)
    TMOD &= 0xF0; 
    TMOD |= 0x01; // 定时器0配置为模式1 (16位定时/计数器)
    
    // 装载初值
    TH0 = TIMER0_RELOAD_H;
    TL0 = TIMER0_RELOAD_L;
    
    TF0 = 0;      // 清除溢出标志位
    ET0 = 1;      // 开启定时器0中断
    TR0 = 1;      // 启动定时器0
    EA  = 1;      // 开启全局中断
}

/**
  * @brief  获取系统运行毫秒数
  * @param  无
  * @retval 毫秒值
  */
u32 Timer_GetMs(void)
{
    u32 temp;
    // 51单片机读取32位变量非原子操作，需关中断保护，防止读取过程中被中断修改
    ET0 = 0; 
    temp = sys_ms;
    ET0 = 1;
    return temp;
}

/**
  * @brief  定时器0中断服务程序 (100微秒触发一次)
  * @param  无
  * @retval 无
  */
void Timer0_Isr(void) interrupt 1
{
    static u8 motor_pwm_cnt = 0;  // 电机PWM周期计数器 (0~99, 对应10ms周期, 100Hz)
    static u8 servo_pwm_cnt = 0;  // 舵机PWM周期计数器 (0~199, 对应20ms周期, 50Hz)
    static u8 ms_cnt = 0;         // 毫秒基准计数器

    // 1. 重新装载定时器初值
    TH0 = TIMER0_RELOAD_H;
    TL0 = TIMER0_RELOAD_L;

    // 2. 毫秒时钟维护 (每10次100us中断 = 1毫秒)
    ms_cnt++;
    if (ms_cnt >= 10)
    {
        ms_cnt = 0;
        sys_ms++;
    }

    // 3. 软件模拟电机 PWM (周期 10ms = 100Hz)
    // 优化：用赋值代替分支，减少中断执行时间（ISR越短越好）
    L_PWM = (motor_pwm_cnt < motor_l_duty) ? 1 : 0;
    R_PWM = (motor_pwm_cnt < motor_r_duty) ? 1 : 0;

    motor_pwm_cnt++;
    if (motor_pwm_cnt >= 100) motor_pwm_cnt = 0;

    // 4. 软件模拟舵机 PWM (周期 20ms = 50Hz，计数器 0~199)
    SERVO1 = (servo_pwm_cnt < servo1_width) ? 1 : 0;
    SERVO2 = (servo_pwm_cnt < servo2_width) ? 1 : 0;

    servo_pwm_cnt++;
    if (servo_pwm_cnt >= 200) servo_pwm_cnt = 0;
}
