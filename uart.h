/**
 ******************************************************************************
 * @file    uart.h
 * @author  Antigravity
 * @brief   串口通信与 OpenMV 协议解析头文件
 ******************************************************************************
 */

#ifndef __UART_H__
#define __UART_H__

#include "config.h"

// 串口初始化
void Uart_Init(void);

// 串口发送单个字节
void Uart_SendByte(u8 dat);

// 串口发送字符串
void Uart_SendString(char *str);

// --- OpenMV 协议解析出的全局控制变量 ---
extern volatile bit uart_rx_flag;  // 接收到新一帧数据的标志位 (1代表有新数据)
extern volatile u8  uart_cmd;      // 命令字 (例如: 0x01 代表循迹偏差数据, 0x02 代表停车标志)
extern volatile s16 uart_data;     // 携带的数据值 (例如: 摄像头计算出的偏离中心点脉冲数，有正负)

#endif /* __UART_H__ */
