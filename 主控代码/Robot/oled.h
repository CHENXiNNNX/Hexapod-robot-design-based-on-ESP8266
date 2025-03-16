#ifndef OLED_H
#define OLED_H
// 包含 Arduino 核心头文件，提供了 Arduino 编程常用的基础数据类型和函数等
#include <Arduino.h>
// 包含 Wire 库头文件，用于实现 I2C 通信功能，因为 OLED 可能通过 I2C 与主控板通信
#include <Wire.h>
// 包含自定义的 oledfont.h 头文件，可能包含了 OLED 显示所需的字体数据等相关定义
#include "oledfont.h"

#define OLED_CMD  0  // 定义一个宏 OLED_CMD，值为 0，表示向 OLED 写命令的标识
#define OLED_DATA 1  // 定义一个宏 OLED_DATA，值为 1，表示向 OLED 写数据的标识

// 声明函数 OLED_Init，用于初始化 OLED 显示屏，无参数，无返回值
void OLED_Init(void);

// 声明函数 OLED_ColorTurn，用于控制 OLED 显示屏的反显功能，参数 i 为 uint8_t 类型，无返回值
void OLED_ColorTurn(uint8_t i);

// 声明函数 OLED_DisplayTurn，用于将 OLED 显示屏的显示内容旋转 180 度，参数 i 为 uint8_t 类型，无返回值
void OLED_DisplayTurn(uint8_t i);

// 声明函数 OLED_WR_Byte，用于向 OLED 发送一个字节的数据，参数 dat 是要发送的字节数据，mode 用于标识是写命令还是写数据，无返回值
void OLED_WR_Byte(uint8_t dat, uint8_t mode);

// 声明函数 OLED_Set_Pos，用于设置 OLED 显示屏上的显示坐标，参数 x 和 y 分别表示横坐标和纵坐标，无返回值
void OLED_Set_Pos(uint8_t x, uint8_t y);

// 声明函数 OLED_Display_On，用于开启 OLED 显示屏的显示功能，无参数，无返回值
void OLED_Display_On(void);

// 声明函数 OLED_Display_Off，用于关闭 OLED 显示屏的显示功能，无参数，无返回值
void OLED_Display_Off(void);

// 声明函数 OLED_ClearRow，用于清除 OLED 显示屏上的指定行显示内容，参数 row 表示要清除的行号，无返回值
void OLED_ClearRow(uint8_t row);

// 声明函数 OLED_Clear，用于清除 OLED 显示屏上的显示内容，无参数，无返回值
void OLED_Clear(void);

// 声明函数 OLED_ShowChar，用于在 OLED 显示屏的指定位置显示一个字符，参数 x 和 y 是显示位置坐标，chr 是要显示的字符，sizey 表示字符的字体大小，无返回值
void OLED_ShowChar(uint8_t x, uint8_t y, const uint8_t chr, uint8_t sizey);

// 声明函数 oled_pow，用于计算 m 的 n 次方，参数 m 和 n 为 uint8_t 类型，返回值为计算结果，类型是 uint32_t
uint32_t oled_pow(uint8_t m, uint8_t n);

// 声明函数 OLED_ShowNum，用于在 OLED 显示屏的指定位置显示一个数字，参数 x 和 y 是显示位置坐标，num 是要显示的数字，len 表示数字的位数，sizey 表示数字的字体大小，无返回值
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t sizey);

// 声明函数 OLED_ShowString，用于在 OLED 显示屏的指定位置显示一个字符串，参数 x 和 y 是显示位置坐标，chr 是指向字符串的指针，sizey 表示字符串的字体大小，无返回值
void OLED_ShowString(uint8_t x, uint8_t y, const char *chr, uint8_t sizey);

// 声明函数 OLED_ShowChinese，用于在 OLED 显示屏的指定位置显示一个汉字，参数 x 和 y 是显示位置坐标，no 表示汉字的编号，sizey 表示汉字的字体大小，无返回值
void OLED_ShowChinese(uint8_t x, uint8_t y, const uint8_t no, uint8_t sizey);

// 声明函数 OLED_DrawBMP，用于在 OLED 显示屏的指定位置显示一张图片，参数 x 和 y 是显示位置坐标，sizex 和 sizey 表示图片的宽度和高度，BMP 是指向图片数据数组的指针，无返回值
void OLED_DrawBMP(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, const uint8_t BMP[]);

#endif
