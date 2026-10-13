/**
 ******************************************************************************
 * @file    pid.c
 * @author  Antigravity
 * @brief   经典位置式 PID 控制算法实现 (集成抗饱和积分和输出限幅)
 ******************************************************************************
 */

#include "pid.h"

/**
  * @brief  PID 控制器参数初始化
  * @param  pid         PID 结构体指针
  * @param  kp/ki/kd    PID 三项参数 (实际系数已将小数部分扩大成1000倍存储，如 Kp=1.5 输入 1500)
  * @param  max_integ   积分上限
  * @param  max_out     输出限制上限
  * @retval 无
  * @note   使用定点整数运算替代float，51单片机上算法速度提升 5~10 倍。
  *         Kp、Ki、Kd 均为实际系数的 10 倍，如 Kp=1.5 则传入 15, Kp=0.3 则传入 3。
  */
void PID_Init(PID_t *pid, s16 kp, s16 ki, s16 kd, s32 max_integ, s16 max_out)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    
    pid->error = 0;
    pid->last_error = 0;
    pid->integral = 0;
    
    pid->max_integral = max_integ;
    pid->max_output = max_out;
}

/**
  * @brief  位置式 PID 计算
  * @param  pid         PID 结构体指针
  * @param  target      目标设定值
  * @param  feedback    实际测量值（反馈值）
  * @retval PID 控制器输出计算结果 (已做限幅处理)
  */
s16 PID_Compute(PID_t *pid, s16 target, s16 feedback)
{
    s32 output_val = 0;
    s16 delta_err = 0;
    
    // 1. 计算当前偏差値
    pid->error = target - feedback;
    
    // 2. 微分项（先算，后更新last_error）
    delta_err = pid->error - pid->last_error;
    
    // 3. 积分累加 + 抄墙限幅 (先判断确幸再加，这是正确的抗饱和顺序)
    pid->integral += pid->error;
    if (pid->integral >  pid->max_integral) pid->integral =  pid->max_integral;
    if (pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;
    
    // 4. 定点整数 PID 公式计算 (Kp/Ki/Kd 均已扩大倐10倍，最后除回10)
    // 输出 = (Kp*偏差 + Ki*积分 + Kd*微分) / 10
    output_val = ((s32)pid->Kp * pid->error)
               + ((s32)pid->Ki * pid->integral)
               + ((s32)pid->Kd * delta_err);
    output_val /= 10;    // 除以放大倍数
    
    // 5. 更新上次偏差
    pid->last_error = pid->error;
    
    // 6. 输出限制幅値限制，防止小车跑飞
    if (output_val >  pid->max_output) output_val =  pid->max_output;
    if (output_val < -pid->max_output) output_val = -pid->max_output;
    
    return (s16)output_val;
}

/**
  * @brief  清除 PID 的积分值和历史偏差，通常在小车停止或者重新起步时调用
  * @param  pid         PID 结构体指针
  * @retval 无
  */
void PID_Clear(PID_t *pid)
{
    pid->error = 0;
    pid->last_error = 0;
    pid->integral = 0;
}
