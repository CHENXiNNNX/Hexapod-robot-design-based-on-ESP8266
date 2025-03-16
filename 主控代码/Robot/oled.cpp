#include "oled.h"
// 包含自定义的 oled.h 头文件，该头文件声明了本文件中要实现的函数原型

// 反显函数
// 功能：控制 OLED 屏幕的显示颜色模式，实现正常显示或反色显示
// 参数：i 为 0 时正常显示，为 1 时反色显示
void OLED_ColorTurn(uint8_t i)
{
    // 判断参数 i 的值
    if (!i) 
        // 如果 i 为 0，向 OLED 发送 0xA6 命令，实现正常显示
        OLED_WR_Byte(0xA6, OLED_CMD); 
    else 
        // 如果 i 不为 0，向 OLED 发送 0xA7 命令，实现反色显示
        OLED_WR_Byte(0xA7, OLED_CMD);    
}

// 屏幕旋转180度
// 功能：控制 OLED 屏幕的显示方向，实现正常显示或旋转 180 度显示
// 参数：i 为 0 时正常显示，为非 0 时旋转 180 度显示
void OLED_DisplayTurn(uint8_t i)
{
    // 判断参数 i 的值是否为 0
    if (i == 0)
    {
        // 如果 i 为 0，向 OLED 发送 0xC8 命令，设置扫描方向为正常
        OLED_WR_Byte(0xC8, OLED_CMD); 
        // 向 OLED 发送 0xA1 命令，设置段重映射为正常
        OLED_WR_Byte(0xA1, OLED_CMD);
    }
    else
    {
        // 如果 i 不为 0，向 OLED 发送 0xC0 命令，设置扫描方向为反转
        OLED_WR_Byte(0xC0, OLED_CMD); 
        // 向 OLED 发送 0xA0 命令，设置段重映射为反转
        OLED_WR_Byte(0xA0, OLED_CMD);
    }
}

// 发送一个字节
// 功能：向 SSD1306 OLED 驱动芯片写入一个字节的数据
// 参数：dat 为要写入的字节数据，mode 为数据/命令标志，0 表示命令，1 表示数据
void OLED_WR_Byte(uint8_t dat, uint8_t mode)
{
    // 开始 I2C 传输，设备地址为 0x3c
    Wire.beginTransmission(0x3c);
    // 判断 mode 的值
    if (mode)
    {
        // 如果 mode 为 1，向 I2C 总线写入 0x40，表示接下来要写入的数据
        Wire.write(0x40);
    }
    else
    {
        // 如果 mode 为 0，向 I2C 总线写入 0x00，表示接下来要写入的是命令
        Wire.write(0x00);
    }
    // 向 I2C 总线写入要发送的字节数据
    Wire.write(dat); 
    // 结束 I2C 传输
    Wire.endTransmission();    
}

// 坐标设置
// 功能：设置 OLED 屏幕上的显示坐标
// 参数：x 为列坐标（范围 0 - 127），y 为页坐标（范围 0 - 7）
void OLED_Set_Pos(uint8_t x, uint8_t y)
{
    // 向 OLED 发送设置页地址的命令，页地址由 0xb0 + y 确定
    OLED_WR_Byte(0xb0 + y, OLED_CMD);
    // 向 OLED 发送设置列高地址的命令，列高地址由 (x & 0xf0) >> 4 | 0x10 确定
    OLED_WR_Byte(((x & 0xf0) >> 4) | 0x10, OLED_CMD);
    // 向 OLED 发送设置列低地址的命令，列低地址由 x & 0x0f 确定
    OLED_WR_Byte((x & 0x0f), OLED_CMD);
}

// 开启OLED显示
// 功能：开启 OLED 屏幕的显示功能
void OLED_Display_On(void)
{
    // 向 OLED 发送 SET DCDC 命令，用于设置电源管理
    OLED_WR_Byte(0X8D, OLED_CMD);  
    // 向 OLED 发送 DCDC ON 命令，开启 DCDC 电源
    OLED_WR_Byte(0X14, OLED_CMD);  
    // 向 OLED 发送 DISPLAY ON 命令，开启显示功能
    OLED_WR_Byte(0XAF, OLED_CMD);  
}

// 关闭OLED显示
// 功能：关闭 OLED 屏幕的显示功能
void OLED_Display_Off(void)
{
    // 向 OLED 发送 SET DCDC 命令，用于设置电源管理
    OLED_WR_Byte(0X8D, OLED_CMD);  
    // 向 OLED 发送 DCDC OFF 命令，关闭 DCDC 电源
    OLED_WR_Byte(0X10, OLED_CMD);  
    // 向 OLED 发送 DISPLAY OFF 命令，关闭显示功能
    OLED_WR_Byte(0XAE, OLED_CMD);  
}

// 清行函数
// 功能：清除 OLED 屏幕上的指定行显示内容
void OLED_ClearRow(uint8_t row) {
    OLED_Set_Pos(0, row);
    for(int i=0; i<128; i++) OLED_WR_Byte(0, OLED_DATA);
}

// 清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!
// 功能：清除 OLED 屏幕上的所有显示内容，使屏幕变为黑色
void OLED_Clear(void)
{
    // 定义两个无符号 8 位整数变量 i 和 n，用于循环计数
    uint8_t i, n;
    // 循环 8 次，遍历 8 个页地址（0 - 7）
    for (i = 0; i < 8; i++)
    {
        // 向 OLED 发送设置当前页地址的命令
        OLED_WR_Byte(0xb0 + i, OLED_CMD);    
        // 向 OLED 发送设置列低地址为 0 的命令
        OLED_WR_Byte(0x00, OLED_CMD);      
        // 向 OLED 发送设置列高地址为 0x10 的命令
        OLED_WR_Byte(0x10, OLED_CMD);      
        // 循环 128 次，遍历每一列
        for (n = 0; n < 128; n++)
            // 向当前页的每一列写入 0，即清除该位置的显示内容
            OLED_WR_Byte(0, OLED_DATA);
    } 
}

// 在指定位置显示一个字符
// 功能：在 OLED 屏幕的指定位置显示一个字符
// 参数：x 为列坐标（范围 0 - 127），y 为页坐标（范围 0 - 63），chr 为要显示的字符，sizey 为字体大小（可选 6x8 或 8x16）
void OLED_ShowChar(uint8_t x, uint8_t y, const uint8_t chr, uint8_t sizey)
{
    // 定义无符号 8 位整数变量 c 用于存储字符偏移量，sizex 用于存储字体的宽度，temp 用于临时存储数据
    uint8_t c = 0, sizex = sizey / 2, temp;
    // 定义无符号 16 位整数变量 i 用于循环计数，size1 用于存储字符数据的长度
    uint16_t i = 0, size1;
    // 判断字体大小是否为 8
    if (sizey == 8)
        // 如果字体大小为 8，字符数据长度为 6
        size1 = 6;
    else
        // 如果字体大小不为 8，计算字符数据长度
        size1 = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * (sizey / 2);
    // 计算字符相对于空格字符的偏移量
    c = chr - ' '; 
    // 设置显示字符的起始坐标
    OLED_Set_Pos(x, y);
    // 循环遍历字符数据的每一个字节
    for (i = 0; i < size1; i++)
    {
        // 判断是否需要换行
        if (i % sizex == 0 && sizey != 8)
            // 如果需要换行，设置新的显示坐标并将页坐标加 1
            OLED_Set_Pos(x, y++);
        // 判断字体大小是否为 8
        if (sizey == 8)
        {
            // 如果字体大小为 8，从字库中读取 6x8 字号的字符数据
            temp = pgm_read_byte(&asc2_0806[c][i]);
            // 将读取的数据写入 OLED 屏幕
            OLED_WR_Byte(temp, OLED_DATA); 
        }
        // 判断字体大小是否为 16
        else if (sizey == 16) 
        {
            // 如果字体大小为 16，从字库中读取 8x16 字号的字符数据
            temp = pgm_read_byte(&asc2_1608[c][i]);
            // 将读取的数据写入 OLED 屏幕
            OLED_WR_Byte(temp, OLED_DATA); 
        }
        else
            // 如果字体大小不是 8 或 16，直接返回
            return;
    }
}

// m^n函数
// 功能：计算 m 的 n 次方
// 参数：m 为底数，n 为指数
// 返回值：m 的 n 次方的结果
uint32_t oled_pow(uint8_t m, uint8_t n)
{
    // 定义无符号 32 位整数变量 result 用于存储计算结果，初始值为 1
    uint32_t result = 1;
    // 循环 n 次
    while (n--)
        // 每次循环将 result 乘以 m
        result *= m;
    // 返回计算结果
    return result;
}

// 显示数字
// 功能：在 OLED 屏幕的指定位置显示一个数字
// 参数：x, y 为显示起点坐标，num 为要显示的数字，len 为数字的位数，sizey 为字体大小
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t sizey)
{
    // 定义无符号 8 位整数变量 t 用于循环计数，temp 用于临时存储数字的每一位，m 用于调整字符间距
    uint8_t t, temp, m = 0;
    // 定义无符号 8 位整数变量 enshow 用于控制是否显示前导零
    uint8_t enshow = 0;
    // 判断字体大小是否为 8
    if (sizey == 8)
        // 如果字体大小为 8，设置字符间距调整值为 2
        m = 2;
    // 循环 len 次，遍历数字的每一位
    for (t = 0; t < len; t++)
    {
        // 计算当前位的数字
        temp = (num / oled_pow(10, len - t - 1)) % 10;
        // 判断是否还未开始显示有效数字且不是最后一位
        if (enshow == 0 && t < (len - 1))
        {
            // 如果当前位为 0
            if (temp == 0)
            {
                // 在相应位置显示空格字符
                OLED_ShowChar(x + (sizey / 2 + m) * t, y, ' ', sizey);
                // 跳过本次循环，继续下一次循环
                continue;
            }
            else
                // 如果当前位不为 0，设置 enshow 为 1，表示开始显示有效数字
                enshow = 1;
        }
        // 在相应位置显示当前位的数字
        OLED_ShowChar(x + (sizey / 2 + m) * t, y, temp + '0', sizey);
    }
}

// 显示一个字符号串
// 功能：在 OLED 屏幕的指定位置显示一个字符串
// 参数：x, y 为显示起点坐标，chr 为指向字符串的指针，sizey 为字体大小
void OLED_ShowString(uint8_t x, uint8_t y, const char *chr, uint8_t sizey)
{
    // 定义无符号 8 位整数变量 j 用于循环计数
    uint8_t j = 0;
    // 循环遍历字符串，直到遇到字符串结束符 '\0'
    while (chr[j] != '\0')
    {
        // 在当前位置显示字符串中的一个字符
        OLED_ShowChar(x, y, chr[j++], sizey);
        // 判断字体大小是否为 8
        if (sizey == 8)
            // 如果字体大小为 8，列坐标增加 6
            x += 6;
        else
            // 如果字体大小不为 8，列坐标增加字体宽度的一半
            x += sizey / 2;
    }
}

// 显示汉字
// 功能：在 OLED 屏幕的指定位置显示一个汉字
// 参数：x, y 为显示起点坐标，no 为汉字在字库中的编号，sizey 为字体大小
void OLED_ShowChinese(uint8_t x, uint8_t y, const uint8_t no, uint8_t sizey)
{
    // 定义无符号 16 位整数变量 i 用于循环计数，size1 用于存储汉字数据的长度
    uint16_t i, size1 = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
    // 定义无符号 8 位整数变量 temp 用于临时存储汉字数据
    uint8_t temp;
    // 循环遍历汉字数据的每一个字节
    for (i = 0; i < size1; i++)
    {
        // 判断是否需要换行
        if (i % sizey == 0)
            // 如果需要换行，设置新的显示坐标并将页坐标加 1
            OLED_Set_Pos(x, y++);
        // 判断字体大小是否为 16
        if (sizey == 16)
        {
            // 如果字体大小为 16，从字库中读取 16x16 字号的汉字数据
            temp = pgm_read_byte(&Hzk[no][i]);
            // 将读取的数据写入 OLED 屏幕
            OLED_WR_Byte(temp, OLED_DATA); 
        }
        // 这里可以添加其他字体大小的处理逻辑
        //    else if(sizey==xx) OLED_WR_Byte(xxx[c][i],OLED_DATA);//用户添加字号
        else
            // 如果字体大小不是 16，直接返回
            return;
    }
}

// 显示图片
// 功能：在 OLED 屏幕的指定位置显示一张图片
// 参数：x, y 为显示起点坐标，sizex, sizey 为图片的长宽，BMP 为指向图片数据数组的指针
void OLED_DrawBMP(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, const uint8_t BMP[])
{
    // 定义无符号 16 位整数变量 j 用于循环计数
    uint16_t j = 0;
    // 定义无符号 8 位整数变量 i, m 用于循环计数，temp 用于临时存储图片数据
    uint8_t i, m, temp;
    // 计算图片的页数
    sizey = sizey / 8 + ((sizey % 8) ? 1 : 0);
    // 循环遍历图片的每一页
    for (i = 0; i < sizey; i++)
    {
        // 设置显示图片的起始坐标
        OLED_Set_Pos(x, i + y);
        // 循环遍历图片的每一列
        for (m = 0; m < sizex; m++)
        {
            // 从图片数据数组中读取一个字节的数据
            temp = pgm_read_byte(&BMP[j++]);
            // 将读取的数据写入 OLED 屏幕
            OLED_WR_Byte(temp, OLED_DATA);
        }
    }
}

// 功能：初始化 OLED 屏幕
void OLED_Init (void)
{
    // 初始化 I2C 总线
    Wire.begin ();
    // 发送关闭显示命令
    OLED_WR_Byte (0xAE, OLED_CMD);
    // 设置列地址低 4 位
    OLED_WR_Byte (0x00, OLED_CMD);
    // 设置列地址高 4 位
    OLED_WR_Byte (0x10, OLED_CMD);
    // 设置显示起始行
    OLED_WR_Byte (0x40, OLED_CMD);
    // 设置对比度控制寄存器
    OLED_WR_Byte (0x81, OLED_CMD);
    // 设置对比度值
    OLED_WR_Byte (0xCF, OLED_CMD);
    // 设置段重映射
    OLED_WR_Byte (0xA1, OLED_CMD);
    // 设置扫描方向
    OLED_WR_Byte (0xC8, OLED_CMD);
    // 设置正常显示模式
    OLED_WR_Byte (0xA6, OLED_CMD);
    // 设置多路复用率
    OLED_WR_Byte (0xA8, OLED_CMD);
    // 设置多路复用率值
    OLED_WR_Byte (0x3f, OLED_CMD);
    // 设置显示偏移
    OLED_WR_Byte (0xD3, OLED_CMD);
    // 设置显示偏移值
    OLED_WR_Byte (0x00, OLED_CMD);
    // 设置时钟分频因子
    OLED_WR_Byte (0xD5, OLED_CMD);
    // 设置时钟分频因子值
    OLED_WR_Byte (0x80, OLED_CMD);
    // 设置预充电周期
    OLED_WR_Byte (0xD9, OLED_CMD);
    // 设置预充电周期值
    OLED_WR_Byte (0xF1, OLED_CMD);
    // 设置 COM 引脚硬件配置
    OLED_WR_Byte (0xDA, OLED_CMD);
    // 设置 COM 引脚硬件配置值
    OLED_WR_Byte (0x12, OLED_CMD);
    // 设置 VCOMH 电压等级
    OLED_WR_Byte (0xDB, OLED_CMD);
    // 设置 VCOMH 电压等级值
    OLED_WR_Byte (0x40, OLED_CMD);
    // 设置内存地址模式
    OLED_WR_Byte (0x20, OLED_CMD);
    // 设置内存地址模式为页寻址模式（这里 0x02 表示页寻址模式，具体根据芯片手册确定）
    OLED_WR_Byte (0x02, OLED_CMD);
    // 设置电荷泵（开启或关闭相关电源功能）
    OLED_WR_Byte (0x8D, OLED_CMD);
    // 使能电荷泵（开启内部升压电路，为 OLED 提供合适电压）
    OLED_WR_Byte (0x14, OLED_CMD);
    // 设置显示方式，选择整个显示区域显示内存内容
    OLED_WR_Byte (0xA4, OLED_CMD);
    // 设置正常显示（非反显）
    OLED_WR_Byte (0xA6, OLED_CMD);
    // 清屏操作，将 OLED 屏幕上的显示内容全部清除
    OLED_Clear ();
    // 发送开启显示命令，使 OLED 屏幕开始显示内容
    OLED_WR_Byte (0xAF, OLED_CMD);
}