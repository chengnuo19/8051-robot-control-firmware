/**
 ******************************************************************************
 * @file    oled.h
 * @author  Antigravity
 * @brief   0.96寸 I2C OLED (SSD1306) 屏幕驱动头文件
 ******************************************************************************
 */

#ifndef __OLED_H__
#define __OLED_H__

#include "config.h"

// OLED 初始化
void OLED_Init(void);

// 清屏
void OLED_Clear(void);

// 在指定位置显示一个字符
// row: 行 (1~4)
// col: 列 (1~16)
// chr: 要显示的字符
void OLED_ShowChar(u8 row, u8 col, char chr);

// 在指定位置显示字符串
// row: 行 (1~4)
// col: 列 (1~16)
// str: 字符串指针
void OLED_ShowString(u8 row, u8 col, char *str);

// 在指定位置显示无符号十进制数字
// row: 行 (1~4)
// col: 列 (1~16)
// num: 数字 (0~65535)
// len: 数字的位数 (例如 123 长度就是 3)
void OLED_ShowNum(u8 row, u8 col, u16 num, u8 len);

// 在指定位置显示有符号十进制数字 (带正负号)
// row: 行 (1~4)
// col: 列 (1~16)
// num: 数字 (-32768 ~ 32767)
// len: 数字的位数 (不计负号，如 -123 的长度是 3)
void OLED_ShowSignedNum(u8 row, u8 col, s16 num, u8 len);

#endif /* __OLED_H__ */
