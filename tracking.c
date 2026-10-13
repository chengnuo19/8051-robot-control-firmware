/**
 ******************************************************************************
 * @file    tracking.c
 * @author  Antigravity
 * @brief   红外循迹传感器状态采集与加权偏差估计算法实现
 ******************************************************************************
 */

#include "tracking.h"

// 记录上一次的非零偏差值，用作脱轨（全白）时的方向记忆
static s16 last_non_zero_error = 0;

/**
  * @brief  循迹模块初始化
  * @param  无
  * @retval 无
  */
void Tracking_Init(void)
{
    // STC89C52的IO口置1即配置为准双向口输入模式
    TRACK_L2 = 1;
    TRACK_L1 = 1;
    TRACK_R1 = 1;
    TRACK_R2 = 1;
    
    last_non_zero_error = 0;
}

/**
  * @brief  读取当前红外传感器的原始状态 (统一归正: 1代表黑线，0代表白纸)
  * @param  无
  * @retval 低四位对应的传感器状态： Bit3-L2, Bit2-L1, Bit1-R1, Bit0-R2
  */
u8 Tracking_ReadRaw(void)
{
    u8 raw_val = 0;
    
    // 读取 L2
    if (TRACK_L2 == TRACK_BLACK) raw_val |= 0x08;
    // 读取 L1
    if (TRACK_L1 == TRACK_BLACK) raw_val |= 0x04;
    // 读取 R1
    if (TRACK_R1 == TRACK_BLACK) raw_val |= 0x02;
    // 读取 R2
    if (TRACK_R2 == TRACK_BLACK) raw_val |= 0x01;
    
    return raw_val;
}

/**
  * @brief  根据传感器状态进行加权算法，计算小车偏离轨迹的误差
  * @param  无
  * @retval 误差值 (-4 到 4)，负数偏左，正数偏右
  */
s16 Tracking_GetError(void)
{
    u8 raw = Tracking_ReadRaw();
    s16 current_error = 0;
    
    switch (raw)
    {
        // ------------------ 1. 正常居中 ------------------
        case 0x06: // 0110: 左右内侧都在黑线上
            current_error = 0;
            break;
            
        // ------------------ 2. 轻微偏离 ------------------
        case 0x04: // 0100: 仅左内侧在黑线上 (小车偏右，需向左修正)
            current_error = -1;
            last_non_zero_error = -1;
            break;
            
        case 0x02: // 0010: 仅右内侧在黑线上 (小车偏左，需向右修正)
            current_error = 1;
            last_non_zero_error = 1;
            break;
            
        // ------------------ 3. 中度偏离 ------------------
        case 0x0C: // 1100: 左外与左内都在黑线上
            current_error = -2;
            last_non_zero_error = -2;
            break;
            
        case 0x03: // 0011: 右内与右外都在黑线上
            current_error = 2;
            last_non_zero_error = 2;
            break;
            
        // ------------------ 4. 重度偏离 ------------------
        case 0x08: // 1000: 仅左外侧在黑线上
            current_error = -3;
            last_non_zero_error = -3;
            break;
            
        case 0x01: // 0001: 仅右外侧在黑线上
            current_error = 3;
            last_non_zero_error = 3;
            break;

        // ------------------ 5. 特殊状态 ------------------
        case 0x0F: // 1111: 十字路口或全黑
            current_error = 0; // 十字路口视为居中，外加标志位供主程序判定
            break;
            
        case 0x0E: // 1110: 偏左十字或转弯
            current_error = -2;
            break;
            
        case 0x07: // 0111: 偏右十字或转弯
            current_error = 2;
            break;

        case 0x00: // 0000: 全白/脱轨 (失轨记忆算法)
            // 如果上次偏左，则认为小车冲到了轨道右侧，需要用最大角度向左旋回
            if (last_non_zero_error < 0)
            {
                current_error = -4; // 给定一个更大的负误差
            }
            // 如果上次偏右，则认为小车冲到了轨道左侧，需要用最大角度向右旋回
            else if (last_non_zero_error > 0)
            {
                current_error = 4;  // 给定一个更大的正误差
            }
            else
            {
                current_error = 0;
            }
            break;
            
        default: // 其他杂散信号
            current_error = 0;
            break;
    }
    
    return current_error;
}
