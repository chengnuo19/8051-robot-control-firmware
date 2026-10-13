/**
 ******************************************************************************
 * @file    uart.c
 * @author  Antigravity
 * @brief   串口驱动与 OpenMV 视觉数据状态机协议解析
 ******************************************************************************
 */

#include "uart.h"

// --- 全局解析输出变量 ---
volatile bit uart_rx_flag = 0;  // 接收成功一帧标志位
volatile u8  uart_cmd     = 0;  // 当前接收到的命令字
volatile s16 uart_data    = 0;  // 当前接收到的数据

/**
  * @brief  串口初始化函数 (使用 Timer 1 产生波特率)
  * @param  无
  * @retval 无
  */
void Uart_Init(void)
{
    // 配置串口工作模式 1 (8位异步收发，波特率由定时器控制)
    SCON = 0x50; 
    
    // 配置定时器1为模式2 (8位自动重装载)
    TMOD &= 0x0F; 
    TMOD |= 0x20; 
    
#ifdef OSC_FREQ_110592
    // 11.0592MHz 晶振，配成稳定的 9600 波特率 (SMOD=0)
    PCON &= 0x7F; // SMOD = 0
    TH1 = 0xFA;   // 重载值
    TL1 = 0xFA;
#else
    // 12.0000MHz 晶振，配成相对更稳的 4800 波特率 (SMOD=1)
    PCON |= 0x80; // SMOD = 1 (波特率加倍)
    TH1 = 0xF3;   // 在12M加倍情况下，重载为0xF3对应4800波特率，误差小于0.16%
    TL1 = 0xF3;
#endif

    TR1 = 1;      // 启动定时器1
    ES = 1;       // 开启串口中断
    EA = 1;       // 开启总中断
}

/**
  * @brief  串口发送一个字节
  * @param  dat 要发送的字节
  * @retval 无
  */
void Uart_SendByte(u8 dat)
{
    SBUF = dat;
    while (!TI);  // 等待发送结束
    TI = 0;       // 清除发送中断标志位
}

/**
  * @brief  串口发送字符串
  * @param  str 字符串指针
  * @retval 无
  */
void Uart_SendString(char *str)
{
    while (*str != '\0')
    {
        Uart_SendByte(*str);
        str++;
    }
}

/**
  * @brief  串口中断服务程序 (状态机解析 OpenMV 协议数据)
  * @note   协议格式： 0x55 (头1) + 0xAA (头2) + CMD (命令) + Data_H (高字节) + Data_L (低字节) + Checksum (校验和)
  *         Checksum = (CMD + Data_H + Data_L) 的低8位值。
  */
void Uart_Isr(void) interrupt 4
{
    static u8 rx_state = 0;   // 状态机当前状态
    static u8 temp_cmd = 0;   // 临时暂存命令
    static u8 temp_data_h = 0;// 临时暂存数据高8位
    static u8 temp_data_l = 0;// 临时暂存数据低8位
    u8 rx_byte;

    if (RI)
    {
        RI = 0;               // 清除接收标志位
        rx_byte = SBUF;       // 读取接收到的字节

        switch (rx_state)
        {
            case 0: // 寻找帧头 1: 0x55
                if (rx_byte == 0x55)
                {
                    rx_state = 1;
                }
                break;

            case 1: // 寻找帧头 2: 0xAA
                if (rx_byte == 0xAA)
                {
                    rx_state = 2;
                }
                else
                {
                    rx_state = 0; // 头2匹配失败，重新寻找头1
                }
                break;

            case 2: // 接收命令字节 CMD
                temp_cmd = rx_byte;
                rx_state = 3;
                break;

            case 3: // 接收数据高8位 Data_H
                temp_data_h = rx_byte;
                rx_state = 4;
                break;

            case 4: // 接收数据低8位 Data_L
                temp_data_l = rx_byte;
                rx_state = 5;
                break;

            case 5: // 接收校验字节 Checksum 并校验
                // 校验算法：CMD + Data_H + Data_L
                if (rx_byte == (u8)(temp_cmd + temp_data_h + temp_data_l))
                {
                    // 校验成功，更新全局变量
                    uart_cmd = temp_cmd;
                    uart_data = (s16)((temp_data_h << 8) | temp_data_l);
                    uart_rx_flag = 1; // 置位接收标志位，通知主程序处理
                }
                
                rx_state = 0; // 无论校验成功与否，状态机归零准备下一帧
                break;

            default:
                rx_state = 0;
                break;
        }
    }
}
