#include <DHT.h>  // 包含DHT传感器库
// DHT11引脚说明：
// VCC -> 3.3V
// DATA -> NodeMCU D2 (GPIO4)
// GND -> GND
#define DHTPIN 4      // 定义传感器连接的数字引脚D2
#define DHTTYPE DHT11 // 指定传感器型号

// 初始化DHT传感器对象
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(115200); // 初始化串口通信，波特率设置为115200
  Serial.println("\nDHT11温湿度监测实验"); // 打印欢迎信息
  
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
    return;
  }

  // 打印温湿度数据到串口
  Serial.print("湿度: ");
  Serial.print(humidity);
  Serial.print("%\t");
  Serial.print("温度: ");
  Serial.print(temperature);
  Serial.println("°C");
}