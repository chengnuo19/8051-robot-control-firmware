/**
 ******************************************************************************
 * @file    pid.h
 * @author  Antigravity
 * @brief   经典位置式与增量式 PID 控制算法头文件
 ******************************************************************************
 */

#ifndef __PID_H__
#define __PID_H__

#include "config.h"

// PID 控制器结构体定义
// 【重要】Kp、Ki、Kd 均为实际系数的 10 倍 (5倍实居4.5扩大输入 45)。
// 优点: 相比float，在STC89C52上算法速度提升至少6倍。
typedef struct {
    s16   Kp;              // 比例系数 (实际値×10)
    s16   Ki;              // 积分系数 (实际値×10)
    s16   Kd;              // 微分系数 (实际値×10)
    
    s16   error;           // 当前偏差
    s16   last_error;      // 上一次偏差
    s32   integral;        // 积分累加値
    
    s32   max_integral;    // 积分上限 (抗饱和)
    s16   max_output;      // 输出上限保护
} PID_t;

/**
  * @brief  PID 控制器参数初始化
  * @param  pid         PID 结构体指针
  * @param  kp/ki/kd    PID 三项参数 (匹配定点精度: 实际値×10传入，如Kp=1.5则传入 15)
  * @param  max_integ   积分上限
  * @param  max_out     输出限制上限
  */
void PID_Init(PID_t *pid, s16 kp, s16 ki, s16 kd, s32 max_integ, s16 max_out);

/**
  * @brief  位置式 PID 计算
  * @param  pid         PID 结构体指针
  * @param  target      目标设定值
  * @param  feedback    实际测量值（反馈值）
  * @retval PID 控制器输出计算结果
  */
s16 PID_Compute(PID_t *pid, s16 target, s16 feedback);

/**
  * @brief  清除 PID 的积分和历史记录 (在转向、刹车或重新启动时调用)
  * @param  pid         PID 结构体指针
  */
void PID_Clear(PID_t *pid);

#endif /* __PID_H__ */
