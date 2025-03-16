#include "oled.h"
#include "oledfont.h"
// 定义常量，用于指定 I2C 通信的 SDA 引脚为 GPIO5
#define SDA 5
// 定义常量，用于指定 I2C 通信的 SCL 引脚为 GPIO4
#define SCL 4
// 定义一个无符号 8 位整数变量 t，并初始化为空格字符的 ASCII 码值
uint8_t t = ' ';

// 初始化函数，在程序开始时执行一次，用于设置硬件和初始状态
void setup() {   
    // 初始化 I2C 总线，指定 SDA 和 SCL 引脚
    Wire.begin(SDA, SCL); 
    // 初始化 OLED 显示屏，设置其初始状态和参数
    OLED_Init();
    // 设置 OLED 显示屏为正常显示模式（非反色显示）
    OLED_ColorTurn(0); 
    // 设置 OLED 显示屏为正常显示方向（不翻转 180 度）
    OLED_DisplayTurn(0); 
}

// 主循环函数，程序会不断重复执行其中的代码
void loop() {
    // 清屏
    OLED_Clear();
    // 在 (0, 0) 位置显示“陈”字，字号为 16x16
    OLED_ShowChinese(0, 0, 0, 16); 
    // 在 (16, 0) 位置显示“鑫”字，字号为 16x16
    OLED_ShowChinese(16, 0, 1, 16); 
    delay(5000); // 显示 5 秒
}