#include <ESP8266WiFi.h>        //引入ESP8266WiFi库，用于ESP8266模块连接Wi-Fi网络，实现网络通信功能
#include <PubSubClient.h>       //引入PubSubClient库，该库用于实现MQTT客户端功能，方便与MQTT服务器进行消息发布和订阅
#include <DHT.h>                //引入DHT库用于读取DHT温湿度传感器的数据
#include <ArduinoJson.h>        //引入ArduinoJson库，用于处理JSON数据格式，在与服务器通信时可能会用到JSON数据的解析和生成
#include <AliyunIoTSDK.h>       //引入AliyunIoTSDK库，这是阿里云物联网平台的SDK，用于设备与阿里云物联网平台进行交互
#include <NTPClient.h>          //引入NTPClient库，用于与网络时间协议（NTP）服务器进行通信，获取准确的时间信息
#include <WiFiUdp.h>            //引入WiFiUdp库，提供UDP（用户数据报协议）通信功能，NTPClient库会依赖它来进行网络数据传输
#include "oled.h"               //引入自定义的oled.h头文件，用于OLED显示屏的驱动和控制，包含了OLED相关函数的声明
#include "oledfont.h"           //引入自定义的oledfont.h头文件，包含了OLED显示所需的字体数据


#define DHTPIN 2                //定义DHT温湿度传感器的数据引脚连接到ESP8266的GPIO2引脚
#define DHTTYPE DHT11           //定义使用的DHT传感器类型为DHT11


#define SDA 5                   //定义I2C通信的SDA（数据线）引脚连接到ESP8266的GPIO5引脚
#define SCL 4                   //定义I2C通信的SCL（时钟线）引脚连接到ESP8266的GPIO4引脚


DHT dht(DHTPIN, DHTTYPE);       //创建一个DHT对象，用于后续读取DHT11传感器的温湿度数据，传入引脚号和传感器类型


const char* WIFI_SSID = "逼迫老丈人在外做1";        //定义要连接的Wi-Fi网络的名称
const char* WIFI_PASSWD = "abc142359";             //定义要连接的Wi-Fi网络的密码


const char* PRODUCT_KEY = "a1CBZS6ySaX";                          //定义阿里云物联网平台的产品密钥，在阿里云平台创建产品时生成，用于标识产品
const char* DEVICE_NAME = "Tem_Hum";                              //定义设备名称，在阿里云平台创建设备时设置，用于标识具体的设备
const char* DEVICE_SECRET = "151b48cc6754aeb6a042dd141713b61a";   //定义设备密钥，在阿里云平台创建设备时生成，用于设备身份验证
const char* REGION_ID = "cn-shanghai";                            //定义设备所在的区域ID，这里表示设备位于阿里云的上海区域


//指定 MQTT 连接参数
const char* MQTT_SERVER = "a1CBZS6ySaX.iot-as-mqtt.cn-shanghai.aliyuncs.com";                //定义MQTT服务器的地址，这里是阿里云物联网平台的MQTT服务器地址
const int MQTT_PORT = 1883;                       //定义MQTT服务器的端口号，1883是常见的MQTT协议默认端口
const char* MQTT_CLIENT_ID = "a1CBZS6ySaX.Tem_Hum|securemode=2,signmethod=hmacsha256,timestamp=1740067377306|";   //定义MQTT客户端的ID，用于在MQTT服务器上唯一标识该客户端，包含了一些安全模式、签名方法和时间戳等信息
const char* MQTT_USERNAME = "Tem_Hum&a1CBZS6ySaX";        //定义连接MQTT服务器的用户名，通常与设备名称和产品密钥相关
const char* MQTT_PASSWORD = "9a68136a57d886e082d240de60b21a8754f2cbfcc27d69706e2dfebdbda24e3d";     //定义连接MQTT服务器的密码，用于身份验证


WiFiClient espClient;             //创建一个WiFiClient对象，用于建立与Wi-Fi网络的连接，进而与服务器进行通信
PubSubClient client(espClient);   //创建一个PubSubClient对象，传入WiFiClient对象，用于实现MQTT客户端功能，与MQTT服务器进行消息交互
WiFiUDP ntpUDP;                   //创建一个WiFiUDP对象，用于通过UDP协议进行网络数据传输，为NTPClient提供底层支持
NTPClient timeClient(ntpUDP, "pool.ntp.org");       //创建一个NTPClient对象，传入WiFiUDP对象和NTP服务器地址（这里是pool.ntp.org），用于获取网络时间


void syncTime() {
    timeClient.begin();   //初始化NTPClient，开始与NTP服务器建立连接
    timeClient.update();  //更新时间信息，从NTP服务器获取最新的时间数据
  
    unsigned long epochTime = timeClient.getEpochTime();  //获取从1970年1月1日00:00:00 UTC到当前时间的秒数（时间戳）
    
    Serial.print("同步时间: ");   //在串口打印提示信息“同步时间: ”
    Serial.println(epochTime);   //在串口打印获取到的时间戳
    
}

void wifiConnect() {
    WiFi.begin(WIFI_SSID, WIFI_PASSWD);       //启动ESP8266连接Wi-Fi网络，传入Wi-Fi名称和密码
    while (WiFi.status() != WL_CONNECTED) {   //当Wi-Fi连接状态不是已连接（WL_CONNECTED）时，进入循环
        delay(1000);//延迟1秒
        Serial.println("Connecting to WiFi...");  //在串口打印提示信息“Connecting to WiFi...”，表示正在连接Wi-Fi
    }

    Serial.println("Connected to WiFi");    //当成功连接到Wi-Fi网络后，在串口打印提示信息“Connected to WiFi”
    Serial.print("IP Address: ");           //在串口打印提示信息“IP Address: ”
    Serial.println(WiFi.localIP());         //在串口打印ESP8266获取到的本地IP地址
    
}

void setupAliyun() {
    AliyunIoTSDK::begin(espClient, PRODUCT_KEY, DEVICE_NAME, DEVICE_SECRET, REGION_ID); //初始化阿里云物联网平台SDK，传入WiFiClient对象、产品密钥、设备名称、设备密钥和区域ID
    client.setServer(MQTT_SERVER, MQTT_PORT); //设置MQTT客户端要连接的MQTT服务器地址和端口号
    while (!client.connected()) {   //当MQTT客户端未连接到服务器时，进入循环
        Serial.println("Connecting to MQTT Server ...");  //在串口打印提示信息“Connecting to MQTT Server ...”，表示正在连接MQTT服务器
        if (client.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD)) {   //尝试连接MQTT服务器，传入客户端ID、用户名和密码，如果连接成功
            Serial.println("MQTT Connected!");    //在串口打印提示信息“MQTT Connected!”，表示已成功连接到MQTT服务器
            
        } else {    //如果连接失败
            Serial.print("MQTT Connect failed, rc="); //在串口打印提示信息“MQTT Connect failed, rc=”，表示连接失败
            Serial.print(client.state());   //打印连接失败的错误代码
            Serial.println(" Retrying in 3 seconds...");    //在串口打印提示信息“ Retrying in 5 seconds...”，表示将在3秒后重试连接
            delay(3000);  //延迟3秒    
        }
    }
}

// 定义消息回调函数，当MQTT客户端接收到消息时会被调用
// topic：消息的主题
// payload：消息的内容（字节数组形式）
// length：消息内容的长度
void messageCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived [");  //在串口打印提示信息“Message arrived [”
    Serial.print(topic);                //打印接收到的消息主题
    Serial.print("] ");                  //在串口打印提示信息“] ”
    for (int i = 0; i < length; i++) {   //循环遍历消息内容的每一个字节      
        
        Serial.print((char)payload[i]);  //将字节转换为字符并在串口打印出来
        
    }
    Serial.println(); //在串口打印换行符 
}

void setup() {
    Serial.begin(115200);   //初始化串口通信，设置波特率为115200，用于在串口监视器上查看调试信息
    Serial.println("\nDHT11温湿度监测实验");    //在串口打印实验名称“DHT11温湿度监测实验”
    Wire.begin(SDA, SCL);   //初始化I2C总线，传入SDA和SCL引脚号，用于与I2C设备（如OLED显示屏）通信
    OLED_Init();            //调用OLED_Init函数，初始化OLED显示屏
    

    OLED_ColorTurn(0);    //0正常显示 1反色显示
    OLED_DisplayTurn(0);  //0正常显示 1翻转180度显示
    OLED_Clear();         //清屏

    dht.begin();          //初始化DHT温湿度传感器
    
    wifiConnect();        //调用wifiConnect函数，连接到指定的Wi-Fi网络
    
    syncTime();           //调用syncTime函数，同步时间
    
    setupAliyun();        //调用setupAliyun函数，初始化阿里云物联网平台连接并连接到MQTT服务器
    
    client.setCallback(messageCallback);    //设置MQTT客户端的消息回调函数为messageCallback，当接收到消息时会调用该函数
    
    delay(500);          //给温湿度传感器稳定时间

}

void loop() {
    if (!client.connected()) {    //如果MQTT客户端未连接到服务器
        
        Serial.println("MQTT Disconnected. Reconnecting...");   //在串口打印提示信息“MQTT Disconnected. Reconnecting...”，表示MQTT连接已断开，正在重新连接
        setupAliyun();    // 调用setupAliyun函数，尝试重新连接到MQTT服务器
        
    }
    client.loop();    //调用MQTT客户端的loop函数，处理网络事件，保持连接并接收消息
    
    delay(2000);  //延迟2秒，控制数据采集的频率
    
    float humidity = dht.readHumidity();          //调用DHT对象的readHumidity函数，读取当前环境的湿度数据，返回值为浮点数
    float temperature = dht.readTemperature();    //调用DHT对象的readTemperature函数，读取当前环境的温度数据，返回值为浮点数
    
    if (isnan(humidity) || isnan(temperature)) {    //如果读取的湿度或温度数据为无效值（NaN）
        
        Serial.println("无法读取DHT传感器数据！");   //在串口打印提示信息“无法读取DHT传感器数据！”
  
        OLED_Clear();   //调用OLED_Clear函数，清除OLED显示屏上的内容
        
        OLED_ShowString(0, 0, "Error Reading", 16);   //在OLED显示屏的(0, 0)位置显示字符串“Error Reading”，字体大小为16
        
        return;
    }

    // 清屏
    OLED_Clear();

    // 显示湿度信息
    OLED_ShowChinese(0, 0, 2, 16);              //在OLED显示屏的(0, 0)位置显示第2个汉字（根据字库索引），字体大小为16---在oledfont.h中的HZK数组，下面同理
    OLED_ShowChinese(16, 0, 3, 16);             //在OLED显示屏的(16, 0)位置显示第3个汉字（根据字库索引），字体大小为16
    OLED_ShowString(32, 0, ": ", 16);           //在OLED显示屏的(32, 0)位置显示字符串“: ”，字体大小为16
    char humidityStr[10];                       //定义一个字符数组，用于存储转换后的湿度字符串
    dtostrf(humidity, 4, 2, humidityStr);       //调用dtostrf函数，将湿度浮点数转换为字符串，参数4表示总宽度（包括小数点和数字），参数2表示小数点后的位数
    OLED_ShowString(70, 0, humidityStr, 16);    //在OLED显示屏的(70, 0)位置显示转换后的湿度字符串，字体大小为16
    OLED_ShowString(120, 0, "%", 16);           //在OLED显示屏的(120, 0)位置显示字符串“%”，字体大小为16
    

    // 显示温度信息
    OLED_ShowChinese(0, 2, 0, 16);              //在OLED显示屏的(0, 2)位置显示第0个汉字（根据字库索引），字体大小为16
    OLED_ShowChinese(16, 2, 1, 16);             //在OLED显示屏的(16, 2)位置显示第1个汉字（根据字库索引），字体大小为16
    OLED_ShowString(32, 2, ": ", 16);           //在OLED显示屏的(32, 2)位置显示字符串“: ”，字体大小为16
    char temperatureStr[10];                    //定义一个字符数组，用于存储转换后的温度字符串   
    dtostrf(temperature, 4, 2, temperatureStr); //调用dtostrf函数，将温度浮点数转换为字符串，参数4表示总宽度（包括小数点和数字），参数2表示小数点后的位数
    OLED_ShowString(70, 2, temperatureStr, 16); //在OLED显示屏的(70, 2)位置显示转换后的温度字符串，字体大小为16
    OLED_ShowString(120, 2, "C", 16);           //在OLED显示屏的(120, 2)位置显示字符串“C”，字体大小为16
      
    AliyunIoTSDK::send((char*)"humidity", humidity);          //调用AliyunIoTSDK的send函数，将湿度数据上传到阿里云物联网平台，第一个参数是数据标识符“humidity”，第二个参数是湿度值
    AliyunIoTSDK::send((char*)"temperature", temperature);    //调用AliyunIoTSDK的send函数，将温度数据上传到阿里云物联网平台，第一个参数是数据标识符“temperature”，第二个参数是温度值
    
    AliyunIoTSDK::loop();   //调用AliyunIoTSDK的loop函数，处理与阿里云物联网平台的通信事件，确保数据正常上传和接收

}