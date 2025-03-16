/*
  功能：
  1. 接收红外信号并解码
  2. 根据键值识别按键类型
  3. 在串口输出按键类型
  接线说明：
  红外接收模块      ESP8266
  VCC          -> 3.3V
  GND          -> GND
  DATA         -> D2（GPIO4）
*/

#include <Arduino.h>          // 包含Arduino核心库
#include <IRremoteESP8266.h>  // 包含ESP8266红外接收库
#include <IRrecv.h>           // 包含红外接收类
#include <IRutils.h>          // 包含红外工具函数

const uint16_t recv_pin = 14;  // 定义红外接收引脚为GPIO4（D2）
IRrecv irrecv(recv_pin);      // 创建红外接收对象，使用指定引脚
decode_results results;       // 创建解码结果存储对象

// 按键识别函数
String getKeyType(uint64_t code) {
  switch (code) {
    case 0xFFA25D: return "数字1";   // 数字1
    case 0xFF629D: return "数字2";   // 数字2
    case 0xFFE21D: return "数字3";   // 数字3
    case 0xFF22DD: return "数字4";   // 数字4
    case 0xFF02FD: return "数字5";   // 数字5
    case 0xFFC23D: return "数字6";   // 数字6
    case 0xFFE01F: return "数字7";   // 数字7
    case 0xFFA857: return "数字8";   // 数字8
    case 0xFF906F: return "数字9";   // 数字9
    case 0xFF9867: return "数字0";   // 数字0
    case 0xFF18E7: return "方向上";  // 方向上
    case 0xFF4AB5: return "方向下";  // 方向下
    case 0xFF10EF: return "方向左";  // 方向左
    case 0xFF5AA5: return "方向右";  // 方向右
    case 0xFF38C7: return "确定（OK）"; // 确定键
    case 0xFF6897: return "星号键";   // 星号键
    case 0xFFB04F: return "＃号键";   // ＃号键
    default: return "未知按键";       // 未知按键
  }
}

void setup() {
  Serial.begin(115200);       // 初始化串口通信，波特率为115200
  irrecv.enableIRIn();        // 启动红外接收器
  Serial.println("\nESP8266红外接收器已启动...");
  Serial.println("等待接收红外信号...");
}

void loop() {
  if (irrecv.decode(&results)) {  // 检查是否接收到红外信号
    uint64_t decCode = results.value; // 获取接收到的键值
    String keyType = getKeyType(decCode); // 调用函数识别按键类型

    // 输出按键信息
    Serial.println("==========================");
    Serial.print("按键类型: ");
    Serial.println(keyType);  // 输出按键类型
    Serial.print("键值（HEX）: ");
    Serial.println(uint64ToString(decCode, HEX).c_str()); // 输出十六进制键值
    Serial.print("键值（DEC）: ");
    Serial.println(uint64ToString(decCode, DEC).c_str()); // 输出十进制键值
    Serial.println("==========================\n");

    irrecv.resume();  // 恢复接收，准备接收下一个信号
  }
  delay(100);  // 延时100ms，避免频繁检测
}