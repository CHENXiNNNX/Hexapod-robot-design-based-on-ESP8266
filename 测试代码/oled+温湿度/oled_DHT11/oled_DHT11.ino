#include <DHT.h>  // 包含DHT传感器库
#include "oled.h" // 包含OLED驱动头文件
#include "oledfont.h"

// DHT11引脚说明：
// VCC -> 3.3V
// DATA -> NodeMCU D2 (GPIO4)
// GND -> GND
#define DHTPIN 2      // 定义传感器连接的数字引脚D4
#define DHTTYPE DHT11 // 指定传感器型号

// 初始化DHT传感器对象
DHT dht(DHTPIN, DHTTYPE);

// 定义OLED I2C引脚
#define SDA 5
#define SCL 4

void setup() {
  Serial.begin(115200); // 初始化串口通信，波特率设置为115200
  Serial.println("\nDHT11温湿度监测实验"); // 打印欢迎信息

  Wire.begin(SDA, SCL); // 初始化I2C总线
  OLED_Init();          // 初始化OLED屏幕
  OLED_ColorTurn(0);    // 0正常显示 1反色显示
  OLED_DisplayTurn(0);  // 0正常显示 1翻转180度显示

  dht.begin();         // 初始化DHT传感器
  delay(500);          // 给传感器稳定时间
}

void loop() {
  // 每次测量之间至少间隔2秒（DHT11最大采样率为1Hz）
  delay(2000); 

  // 读取湿度（百分比）
  float humidity = dht.readHumidity();
  // 读取温度（摄氏度）
  float temperature = dht.readTemperature();

  // 检查传感器数据是否有效
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("无法读取DHT传感器数据！");
    OLED_Clear();
    OLED_ShowString(0, 0, "Error Reading", 16);
    return;
  }

  // 清屏
  OLED_Clear();

  // 显示湿度信息
  OLED_ShowChinese(0, 0, 2, 16); 
  OLED_ShowChinese(16, 0, 3, 16); 
  OLED_ShowString(32, 0, ": ", 16);
  char humidityStr[10];
  dtostrf(humidity, 4, 2, humidityStr); // 将浮点数转换为字符串
  OLED_ShowString(70, 0, humidityStr, 16);
  OLED_ShowString(120, 0, "%", 16);

  // 显示温度信息
  OLED_ShowChinese(0, 2, 0, 16); 
  OLED_ShowChinese(16, 2, 1, 16); 
  OLED_ShowString(32, 2, ": ", 16);
  char temperatureStr[10];
  dtostrf(temperature, 4, 2, temperatureStr); // 将浮点数转换为字符串
  OLED_ShowString(70, 2, temperatureStr, 16);
  OLED_ShowString(120, 2, "C", 16);
}