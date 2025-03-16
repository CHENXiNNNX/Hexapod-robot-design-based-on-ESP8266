/********************库函数定义*******************/
#include <Robot.h>                    //机器人控制库
#include <ESP8266WiFi.h>              //ESP8266WiFi库自带
#include <EEPROM.h>                   //ESP8266EEPROM库自带
#include <Wire.h>                     //ESP8266Wire库自带
#include <Adafruit_PWMServoDriver.h>  //购买自带-PCA9685
#include "StepData.h"                 //预计算步态数据
#include <Arduino.h>                  //包含Arduino核心库
#include <IRremoteESP8266.h>          //包含ESP8266红外接收库
#include <IRrecv.h>                   //包含红外接收类
#include <DHTesp.h>                   //包含温湿度传感器库
#include "oled.h"                     //包含OLED驱动头文件
#include "oledfont.h"                 //包含OLED字模和图片
#include <PubSubClient.h>             //包含MQTT库
#include <AliyunIoTSDK.h>             //包含阿里云IoT库
#include <NTPClient.h>                //包含NTP时间同步库
#include <WiFiUdp.h>                  //用于处理UDP通信
/********************库函数定义*******************/

/***********************全局变量定义***********************/
//全局变量的定义部分，包括了一些对六足机器人控制和状态的变量声明和初始化
RobotClass robot; //实例化一个名为 robot 的 RobotClass 对象，用于控制六足机器人的各项功能

//红外遥控器配置
#define IR_DEBOUNCE 200         //200毫秒
const uint16_t recv_pin = 14;   //红外接收引脚GPIO14
IRrecv irrecv(recv_pin);        //红外接收对象
decode_results results;         //红外解码结果
unsigned long lastIRTime = 0;   //上次红外接收时间

//温湿度传感器配置
DHTesp dht;          //定义一个名为dht的DHTesp对象，用于读取温湿度传感器的数据
#define DHT_PIN 12      //定义温湿度传感器的引脚号为IO12
#define DHT_TYPE DHT11  //定义温湿度传感器的型号为 HT11
#define TEMP_MAX 25.0   //温度阈值
#define HUMI_MAX 50.0   //湿度阈值
bool dhtEnabled = false;    //定义一个名为dhtEnabled的布尔变量，用于控制温湿度传感器的开关
unsigned long lastDHTUpdate = 0;    //OLED刷新计时器
float currentTemp = 0.0;            //当前温度值
float currentHumi = 0.0;            //当前湿度值

//阿里云连接参数
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

//NTP和时间同步配置
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");
WiFiClient espClient;
PubSubClient mqttClient(espClient); //避免与原有client变量冲突

//关节标定数据存储起始位置
#define Servo_Addr 0  //定义关节标定数据存储的起始位置为 EEPROM 内存地址 0
#define del 100       //设置延时时间为 100 毫秒
#define deltr 3       //设置另一个延时时间为 3 毫秒
#define led 2         //用于控制 LED 的引脚号，当引脚为低电平时，LED 会点亮；高电平时，LED 熄灭
/***********************全局变量定义***********************/

/***********************WIFI模块的调用预处理***********************/
//采用硬编码的形式记录wifi's ssid和password，在实际使用时，可以将其替换为从外部读取的变量
//定义 WiFi 服务器的最大连接数
#define MAX_SRV_CLIENTS 3  //设置最大同时连接数为 3，表示你想要接入的设备数量，8266tcpserver最多只能接入五个设备

//定义 WiFi 连接状态标志
bool wifiEnabled = false;         //WiFi使能标志位
bool wifiConnected = false;       //WiFi连接状态标志，初始化为 false，表示未连接
unsigned long lastConnectTry = 0; //最后连接尝试时间

//定义 WiFi 网络信息
const char *ssid = "逼迫老丈人在外做1";  //定义 WiFi 的网络名称
const char *password = "abc142359";     //定义 WiFi 的密码

//定义 WiFi 服务器对象
WiFiServer server(8266);  //定义一个名为 server 的WiFi服务器对象，端口号为 8266，可以根据需要随意修改，范围在 0 到 65535 之间
WiFiClient serverClients[MAX_SRV_CLIENTS];  //定义一个名为 serverClients 的数组，用于存储最大连接数为 3 的 WiFiClient 对象
/***********************WIFI模块的调用预处理***********************/


/***********************定义用于调用舵机的函数变量***********************/
//创建一个名为 pwm1 的 Adafruit_PWMServoDriver 对象，用于驱动1~16或者0~15号舵机，参数 0x40 表示舵机驱动器的 I2C 地址---这里之间默认不写进行了隐式调用I2c地址
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(); 

//创建一个名为 pwm2 的 Adafruit_PWMServoDriver 对象，用于驱动17~32或者16~31号舵机。参数 0x41 表示舵机驱动器的 I2C 地址
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41); 
/***********************定义用于调用舵机的函数变量***********************/
  

/***********************舵机模块的调用预处理***********************/
//这里的设置为了方便以后更换舵机驱动器，可以根据实际情况修改。
#define servo180  
#ifdef servo180

  /**********************舵机控制信号的计算公式**********************
  这个公式是用来计算舵机控制信号的脉冲宽度的。具体来说，这个公式是在设置舵机的脉冲宽度时使用的，通常用于将角度转换为相应的脉冲信号值。

  脉冲宽度 = (舵机最大角度 / PWM周期) * PWM脉冲周期的刻度数

  在这个公式中：
  2.5 表示舵机的最大角度（例如180度舵机的最大角度为180度）
  20 表示 PWM 信号的周期，通常为20ms
  4096 是 PWM 周期的刻度数，表示 PWM 脉冲宽度的分辨率
  通过将舵机的最大角度（2.5）除以 PWM 周期（20），然后乘以 PWM 周期的刻度数（4096），得到了舵机在最大角度时对应的脉冲宽度。

  例如，对于 180 度舵机，最大角度为 180 度，PWM 信号的周期为 20ms，则：
  舵机的最大角度 / PWM 信号的周期 = 180 / 20 = 9.0
  舵机在最大角度时对应的脉冲宽度 = 9.0 / 20 * 4096 = 307
  因此，舵机在 180 度时对应的脉冲宽度为 307，舵机在 0 度时对应的脉冲宽度为 102，舵机在 90 度时对应的脉冲宽度为 204，舵机在 45 度时对应的脉冲宽度为 256。

  因此，在程序中，可以通过以下公式计算舵机控制信号的脉冲宽度(下面为测试数据)：
  #define SERVOMIN  102               //0.5/20 * 4096 = 102
  #define SERVOMID  307               //1.5/20 * 4096 = 307
  #define SERVOMAX  512               //2.5/20 * 4096 = 512

  这样的计算方法可以用于确定舵机在特定角度时所需的 PWM 脉冲宽度值，以便精确控制舵机的角度。
  **********************舵机控制信号的计算公式**********************/

  #define SERVOMIN  102  //定义舵机的最小脉冲宽度为 102
  #define SERVOMID  327  //定义舵机的中间脉冲宽度为 327
  #define SERVOMAX  552  //定义舵机的最大脉冲宽度为 552

#endif
/***********************舵机模块的调用预处理***********************/

/***********************机器人的相关信息配置***********************/
/*a:前进; b:后退; c:左转; d:右转; e:停止; f:向左横行 g:向右横行 h:步态切换 i:身高切换 j:原地蹲起 k:跳跳；
last_cmd表示上一个指令，可以表示机器人状态*/

//存储机器人当前的动作指令和上一个动作指令，并设置初始为停止动作
char cmd = 'e', last_cmd = 'e'; 

//gait表示步态，1为波动步态，0为三角步态；body表示身高，0为最低，1为中间，2为最高;robotstatus为机器人状态，
int gait = 0, body = 0, robotstatus = 0; 

//转弯半径控制比例
float zhuanwan_k = 1; 

//前进后退速度比例
float qianhou_k = 1; 

//存储机器人当前关节变量临时值，用于计算舵机的角度值
signed char rec_lin; 

//联控舵机的动作
signed char rec[18] = {
  0, 0, 0,   
  0, 0, 0,
  0, 0, 0,
  0, 0, 0,
  0, 0, 0,
  0, 0, 0
};

//存储机器人当前关节变量，舵机当前角度的值，即在任何给定时间点提供一个信息，显示出所有关节当前的状态。
float Servo_p[18] = {
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0
};

//存储机器人零点位置的关节角度值，即定义机器人的初始姿态
float zero_p[18] = {
  0.0 , 0.0 , 0.0 ,  
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0
};

//存储机器人目标关节变量，舵机目标角度的值
float Servo_r[18] = {
  0.0 , 0.0 , 0.0 ,  
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0
};

//存储舵机的方向反相参数，用于控制舵机的方向
int direct[18] = {-1, 1, 1, -1, 1, 1, -1, 1, 1, -1, 1, 1, -1, 1, 1, -1, 1, 1}; 

//用于存储或管理步态动作的序列帧数据，调节机器人行走的动态过程
int gait_zhen = 0; 
/***********************机器人的相关信息配置***********************/

/***********************I2C通信模块的调用预处理函数定义***********************/
/*首先尝试与地址为0x40的设备进行通信。
如果未检测到设备，程序将进入一个无限循环，LED会闪烁一次后停顿较长时间，以此循环表示未找到该设备。随后尝试与地址为0x41的设备进行通信。
如果同样未检测到这个设备，程序会进入另一个无限循环，但这次LED会连续闪烁两次后停顿较长时间，再次循环，以此表示未找到该设备。*/

//定义测试I2C设备的函数
void iic_device_test()
{
  //定义一个布尔数组，用来记录两个I2C设备的扫描结果
  bool iic_flag[2];

  //开始与地址为0x40的I2C设备通信
  Wire.beginTransmission(0x40);

  //结束通信并检测设备是否响应
  if (Wire.endTransmission() != 0) //如果返回非0值，表示设备未响应
  {
    while (1) //进入无限循环，用LED闪烁表示未找到设备
    {
      digitalWrite(led, 0); //LED关
      delay(100);
      digitalWrite(led, 1); //LED开
      delay(1000);
    }
  }
  
  //开始与地址为0x41的I2C设备通信
  Wire.beginTransmission(0x41);

  //结束通信并检测设备是否响应
  if(Wire.endTransmission()!=0) //如果返回非0值，表示设备未响应
  {
    while (1)  //进入无限循环，用LED闪烁表示未找到设备
    {
      digitalWrite(led, 0); //LED关
      delay(100);
      digitalWrite(led, 1); //LED开
      delay(100); //短暂延迟
      digitalWrite(led, 0); //LED关
      delay(100);
      digitalWrite(led, 1); //LED开
      delay(1000); //长时间延迟
    }
  }
};
/***********************I2C通信模块的调用预处理函数定义***********************/

/***********************读取EEPROM数据并打印***********************/
/*从EEPROM中读取数据并将其打印出来，主要是为了显示机器人舵机或关节的校准参数，方便后面进行舵机关节的标定*/
//定义一个函数，用于打印关节的校准数据
void print_jointjz(){

  //在串行监视器中打印一个空行
  Serial.println();

  //打印标定的数据提示信息
  Serial.println("标定信息:");
  
  //打印表头，显示每个关节的编号
  Serial.print(" ");

  //遍历机器人的18个舵机关节
  for(int i = 0; i < 18; i++) 
  {
    //打印关节编号
    Serial.print(i); 

    //打印一些空格以分隔数据
    Serial.print(" "); 
  }
  //打印一个换行符，结束当前行
  Serial.println(); 

  //打印每个关节的校准数据
  for(int i = 0; i < 18; i++) //遍历18个关节
  {
    //从EEPROM的第i位置读取一个字节数据
    rec_lin = EEPROM.read(i); 

    //打印读取的数据，代表该关节的校凑值
    Serial.print(rec_lin); 

    //打印空格用于分隔数据
    Serial.print("  "); 
  }
  //打印一个换行符，结束当前行
  Serial.println(); 
}
/***********************读取EEPROM数据并打印***********************/

/***********************控制LED的闪烁**********************/
void blink() {
  static unsigned long lastToggle = 0;
  static bool ledBlinking = false;  //用于决定 LED 是否应该闪烁,初始化为 false,当为 true 时，LED 会闪烁,当为 false 时，LED 会常亮
  static bool lastConnectionState = !wifiConnected;

  //状态变化时立即重置LED逻辑
  if (wifiConnected != lastConnectionState) {
    lastConnectionState = wifiConnected;
    ledBlinking = !wifiConnected; //未连接时启用闪烁
    digitalWrite(led, HIGH); //初始熄灭
    lastToggle = millis(); //重置计时器
  }else if(wifiConnected){
    //连接成功时停止闪烁
    digitalWrite(led, LOW);  //常亮
    ledBlinking = false;
  }


  //未连接时执行闪烁
  if (ledBlinking && (millis() - lastToggle >= 500)) {
    lastToggle = millis();
    digitalWrite(led, !digitalRead(led));
  }
}
/***********************控制LED的闪烁**********************/

/***********************时间同步函数**********************/
void syncTime() {
  timeClient.begin();
  timeClient.update();
  Serial.print("同步时间: ");
  Serial.println(timeClient.getEpochTime());
}
/***********************时间同步函数**********************/

/***********************红外遥控配置***********************/
String getKeyType(uint64_t code) {
  switch (code) {
    //方向控制
    case 0xFF18E7: return "UP";
    case 0xFF4AB5: return "DOWN";
    case 0xFF10EF: return "LEFT";
    case 0xFF5AA5: return "RIGHT";
    case 0xFF38C7: return "OK";
    
    //数字键控制
    case 0xFF9867: return "0";  
    case 0xFFA25D: return "1";
    case 0xFF629D: return "2";
    case 0xFFE21D: return "3";
    case 0xFF22DD: return "4";
    case 0xFF02FD: return "5";
    case 0xFFC23D: return "6";
    
    //功能键控制
    case 0xFF6897: return "STAR";
    case 0xFFB04F: return "HASH";
    
    default: return "UNKNOWN";
  }
}
/***********************红外遥控配置***********************/

/***********************红外遥控处理***********************/
void handleIR() {
  if (irrecv.decode(&results)) { //如果红外接收到信号
    if (millis() - lastIRTime > IR_DEBOUNCE) {  //如果两次红外信号接收时间间隔大于100ms
      lastIRTime = millis();  //记录当前时间
      uint64_t decCode = results.value; //将红外接收到的信号值存储到decCode变量中
      String keyType = getKeyType(decCode); //调用getKeyType函数，将红外接收到的信号值转换为对应的按键类型
      Serial.print("[IR] 接收命令: ");  //打印接收到的红外信号
      Serial.println(keyType);  //打印红外信号对应的按键类型
      if (keyType == "UP") cmd = 'a';         //前进
      else if (keyType == "DOWN") cmd = 'b';  //后退
      else if (keyType == "LEFT") cmd = 'f';  //左转
      else if (keyType == "RIGHT") cmd = 'g'; //右转
      else if (keyType == "OK") cmd = 'e';    //停止
      else if (keyType == "1") cmd = 'c';     //原地左转
      else if (keyType == "2") cmd = 'd';     //原地右转
      else if (keyType == "3") cmd = 'j';     //原地蹲起
      else if (keyType == "4") cmd = 'k';     //跳舞
      else if (keyType == "5") {              //温湿度功能开关
        dhtEnabled = !dhtEnabled; 
        OLED_ClearRow(3);
        OLED_ClearRow(4);
        Serial.println("\n[操作] 温湿度监测已" + String(dhtEnabled ? "启用" : "禁用"));
      }           
      else if (keyType == "6") {              //WiFi 功能开关
        wifiEnabled = !wifiEnabled;
        Serial.println("\n[WiFi] " + String(wifiEnabled ? "尝试连接" : "断开连接"));
        if(!wifiEnabled) {
          WiFi.disconnect(true);
          wifiConnected = false; 
        }
      }                            
      else if (keyType == "0") ESP.restart(); //主控板重置
      else if (keyType == "STAR") cmd = 'h';  //步态切换
      else if (keyType == "HASH") cmd = 'i';  //身高切换
    }
    irrecv.resume();  //继续接收红外信号
  }
}
/***********************红外遥控处理***********************/

/***********************阿里云连接处理*********************/
void handleAliyun() {
  AliyunIoTSDK::begin(espClient, PRODUCT_KEY, DEVICE_NAME, DEVICE_SECRET, REGION_ID);
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  while (!mqttClient.connected()) {
      Serial.println("Connecting to MQTT Server...");
      if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD)) {
          Serial.println("MQTT Connected!");
      } else {
          Serial.print("MQTT Connect failed, rc=");
          Serial.print(mqttClient.state());
          Serial.println(" Retrying in 3 seconds...");
          delay(3000);
      }
  }
}
/***********************阿里云连接处理*********************/

/***********************WiFi连接处理***********************/
void handleWiFi() {
  //状态切换处理
  static bool lastWifiState = false;
  
  if(wifiEnabled != lastWifiState){
    if(wifiEnabled){
      Serial.println("[WiFi] 正在连接至SSID: " + String(ssid));
      WiFi.begin(ssid, password);
      lastConnectTry = millis();
    }
    lastWifiState = wifiEnabled;
  }

  //连接过程处理
  if(wifiEnabled && !wifiConnected){
    if(WiFi.status() == WL_CONNECTED){
      wifiConnected = true;
      Serial.println("[WiFi] 连接成功! IP: " + WiFi.localIP().toString());
      syncTime(); //WiFi连接成功后同步时间
      handleAliyun(); //初始化阿里云连接
    }
    else if(millis() - lastConnectTry > 10000){ //10秒超时
      Serial.println("[WiFi] 连接超时");
      wifiEnabled = false; //自动禁用
    }
  }

  //连接保持检测
  if(wifiEnabled && wifiConnected && WiFi.status() != WL_CONNECTED){
    Serial.println("[WiFi] 检测到连接丢失，尝试重连...");
    wifiConnected = false;
    WiFi.reconnect();
    lastConnectTry = millis();
  }

 //处理客户端连接
 if (server.hasClient()) {
    for (uint8_t i = 0; i < MAX_SRV_CLIENTS; i++) {
      if (!serverClients[i] || !serverClients[i].connected()) {
        if (serverClients[i]) serverClients[i].stop();
        serverClients[i] = server.available();
      break;
      }
    }
  }

  //读取客户端数据
  for (uint8_t i = 0; i < MAX_SRV_CLIENTS; i++) {
    if (serverClients[i] && serverClients[i].connected()) {
      while (serverClients[i].available()) {
        cmd = serverClients[i].read(); // 接收指令
        delay(1); // 防止数据粘包
      }
    }
  }
}
/***********************WiFi连接处理***********************/

/***********************温湿度监测处理*********************/
void handleDHT() {
  static uint8_t refreshStage = 0; //当前刷新阶段
  static unsigned long lastStageTime = 0; //记录每个阶段的开始时间
  static bool lastDHTState = dhtEnabled; //记录上次状态

  //状态切换时立即处理显示
  if (dhtEnabled != lastDHTState) {
    if (!dhtEnabled) { //关闭功能时清屏
      OLED_ClearRow(0);
      OLED_ClearRow(1);
      OLED_ClearRow(2);
      OLED_ShowString(16, 3, "DHT Disabled", 16);
    }
    lastDHTState = dhtEnabled;
    refreshStage = 0; //重置刷新阶段
    return;
  }

  if (!dhtEnabled) return; //功能关闭时直接返回

  //每 2 秒触发一次刷新
  if (millis() - lastDHTUpdate > 2000 && refreshStage == 0) {
    refreshStage = 1; //开始刷新
    lastStageTime = millis(); //记录当前阶段开始时间
  }

  //每个阶段处理完后立即返回，避免阻塞
  switch (refreshStage) {
    case 1: { //读取传感器数据
      TempAndHumidity data = dht.getTempAndHumidity(); //使用 DHTesp 库读取数据
      if (dht.getStatus() != DHTesp::ERROR_NONE) { //检查读取是否成功
        Serial.println("传感器读取失败");
        refreshStage = 0; //重置刷新阶段
        return;
      }
      currentTemp = data.temperature;
      currentHumi = data.humidity;
      Serial.println("\n==================================");
      Serial.print("当前温度："); Serial.print(currentTemp); Serial.println("°C");
      Serial.print("当前湿度："); Serial.print(currentHumi); Serial.println("%");
      Serial.println("==================================");
      refreshStage++; //进入下一阶段
      lastStageTime = millis(); //记录当前阶段开始时间
      break;
    }

    case 2: { //更新温度显示
      OLED_ClearRow(0);
      OLED_ShowChinese(0, 0, 0, 16);  //"温"
      OLED_ShowChinese(16, 0, 1, 16); //"度"
      OLED_ShowString(32, 0, ": ", 16);
      refreshStage++; //进入下一阶段
      lastStageTime = millis(); //记录当前阶段开始时间
      break;
    }

    case 3: { //显示温度数值
      char tempStr[10];
      dtostrf(currentTemp, 4, 2, tempStr);
      OLED_ShowString(70, 0, tempStr, 16);
      OLED_ShowString(120, 0, "C", 16);
      refreshStage++; //进入下一阶段
      lastStageTime = millis(); //记录当前阶段开始时间
      break;
    }

    case 4: { //更新湿度显示
      OLED_ClearRow(2);
      OLED_ShowChinese(0, 2, 2, 16);  //"湿"
      OLED_ShowChinese(16, 2, 3, 16); //"度"
      OLED_ShowString(32, 2, ": ", 16);
      refreshStage++; //进入下一阶段
      lastStageTime = millis(); //记录当前阶段开始时间
      break;
    }

    case 5: { //显示湿度数值
      char humiStr[10];
      dtostrf(currentHumi, 4, 2, humiStr);
      OLED_ShowString(70, 2, humiStr, 16);
      OLED_ShowString(120, 2, "%", 16);
      refreshStage++; //进入下一阶段
      lastStageTime = millis(); //记录当前阶段开始时间
      break;
    }

    case 6: { //显示警告信息
      if (currentTemp > TEMP_MAX || currentHumi > HUMI_MAX) {
        OLED_ShowString(32, 4, "Warning", 16);
      } else {
        OLED_ClearRow(4);
      }
      //上传数到阿里云
      if (wifiConnected && mqttClient.connected()) {
        AliyunIoTSDK::send((char*)"humidity", currentHumi);
        AliyunIoTSDK::send((char*)"temperature", currentTemp);
        AliyunIoTSDK::loop(); //处理阿里云通信
    }
      lastDHTUpdate = millis(); //记录最后一次刷新时间
      refreshStage = 0; //完成一轮刷新
      break;
    }
  }

  //防止某个阶段卡死，超时重置
  if (millis() - lastStageTime > 500) {
    refreshStage = 0; //超时重置刷新阶段
  }
}
/***********************温湿度监测处理*********************/

/***********************机器人控制处理*********************/
void handleRobot(){
  switch(cmd){
    case 'a':  //前进
    {
      if(gait==0)//如果是三角步态
      {
        if(gait_zhen>39)
        {
          gait_zhen = 0;
        }
        if(body == 0)
        {
          for(int i=0;i<18;i++)
          Servo_p[i] = forwarda[gait_zhen][i];
        }
        else if(body == 1)
        {
          for(int i=0;i<18;i++)
          Servo_p[i] = forwarda2[gait_zhen][i];
        }
        else if(body == 2)
        {
          for(int i=0;i<18;i++)
          Servo_p[i] = forwarda3[gait_zhen][i];
        }
        
        robot.framewrite(Servo_p);
        delay(robot.speed);
        ESP.wdtFeed(); //喂狗防止复位
        gait_zhen++;
        }
      else if(gait==1)//如果是波动步态
      {
        if(gait_zhen>59)
        gait_zhen = 0;
      
        for(int i=0;i<18;i++)
        Servo_p[i] = forwardaF2[gait_zhen][i];
        
        robot.framewrite(Servo_p);
        delay(robot.speed);
        ESP.wdtFeed(); //喂狗防止复位
        gait_zhen++;
      }
    }break;
    case 'b':
    {
      if(gait==0)//如果是三角步态
      {
        if(gait_zhen<0)
        gait_zhen = 39;
        
        if(body == 0)
        {
          for(int i=0;i<18;i++)
          Servo_p[i] = forwarda[gait_zhen][i];
        }
        else if(body == 1)
        {
          for(int i=0;i<18;i++)
          Servo_p[i] = forwarda2[gait_zhen][i];
        }
        else if(body == 2)
        {
          for(int i=0;i<18;i++)
          Servo_p[i] = forwarda3[gait_zhen][i];
        }
        
        robot.framewrite(Servo_p);
        ESP.wdtFeed(); //喂狗防止复位
        delay(robot.speed);
        //delay(deltr);
        gait_zhen--;
      }
      else if(gait==1)//如果是波动步态
      {
        if(gait_zhen<0)
        gait_zhen = 59;

        for(int i=0;i<18;i++)
        Servo_p[i] = forwardaF2[gait_zhen][i];
        
        robot.framewrite(Servo_p);
        ESP.wdtFeed(); //喂狗防止复位
        delay(deltr);
        gait_zhen--;
      }
    }break;
    case 'c':
    {
      if(gait==0)//如果是三角步态
      {
        if(body == 0)
        {
          for(int j=0;j<20;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda[20+j][i];

            Servo_p[15] = forwarda[20+j][15];
            Servo_p[16] = forwarda[20+j][16];
            Servo_p[17] = forwarda[20+j][17];
            
            //左边三条腿       
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda[39-j][i];
                
            robot.framewrite(Servo_p);
            delay(robot.speed);
              ESP.wdtFeed(); //喂狗防止复位
              delayMicroseconds(del);
          } 
          for(int j=20;j<40;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda[j-20][i];

            Servo_p[15] = forwarda[j-20][15];
            Servo_p[16] = forwarda[j-20][16];
            Servo_p[17] = forwarda[j-20][17];
            
            //左边三条腿       
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda[39-j][i];
                
            robot.framewrite(Servo_p);
            delay(robot.speed);
              ESP.wdtFeed(); //喂狗防止复位
              delayMicroseconds(del);
          }    
        }
        else if(body == 1)
        {
          for(int j=0;j<20;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda2[20+j][i];

            Servo_p[15] = forwarda2[20+j][15];
            Servo_p[16] = forwarda2[20+j][16];
            Servo_p[17] = forwarda2[20+j][17];
            
            //左边三条腿       
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda2[39-j][i];
                
            robot.framewrite(Servo_p);
            delay(robot.speed);
              ESP.wdtFeed(); //喂狗防止复位
              delayMicroseconds(del);
          } 
          for(int j=20;j<40;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda2[j-20][i];

            Servo_p[15] = forwarda2[j-20][15];
            Servo_p[16] = forwarda2[j-20][16];
            Servo_p[17] = forwarda2[j-20][17];
            
            //左边三条腿       
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda2[39-j][i];
                
            robot.framewrite(Servo_p);
            delay(robot.speed);
              ESP.wdtFeed(); //喂狗防止复位
              delayMicroseconds(del);
          }    
        }
        else if(body == 2)
        {
          for(int j=0;j<20;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda3[20+j][i];

            Servo_p[15] = forwarda3[20+j][15];
            Servo_p[16] = forwarda3[20+j][16];
            Servo_p[17] = forwarda3[20+j][17];
            
            //左边三条腿       
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda3[39-j][i];
                
            robot.framewrite(Servo_p);
            delay(robot.speed);
              ESP.wdtFeed(); //喂狗防止复位
              delayMicroseconds(del);
          } 
          for(int j=20;j<40;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda3[j-20][i];

            Servo_p[15] = forwarda3[j-20][15];
            Servo_p[16] = forwarda3[j-20][16];
            Servo_p[17] = forwarda3[j-20][17];
            
            //左边三条腿       
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda3[39-j][i];
                
            robot.framewrite(Servo_p);
            delay(robot.speed);
              ESP.wdtFeed();  //喂狗防止复位
              delayMicroseconds(del);
          }    
        }
      }
      else if(gait==1)//如果是波动步态
      {
          for(int j=0;j<60;j++)
          {
            //右边腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forwardaF2[j][i];

            Servo_p[15] = forwardaF2[j][15];
            Servo_p[16] = forwardaF2[j][16];
            Servo_p[17] = forwardaF2[j][17];

            //左边腿
            for(int i=6;i<15;i++)
            Servo_p[i] = forwardaF2[59-j][i];

            robot.framewrite(Servo_p);
              delay(robot.speed);
              ESP.wdtFeed(); //喂狗防止复位
              delay(0);
          }        
      }
    }break;
    case 'd':
    {
      if(gait==0)//如果是三角步态
      {
        if(body == 0)
        {
          for(int j=0;j<20;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda[39-j][i];

            Servo_p[15] = forwarda[39-j][15];
            Servo_p[16] = forwarda[39-j][16];
            Servo_p[17] = forwarda[39-j][17];

            //左边三条腿
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda[20+j][i];

            robot.framewrite(Servo_p);
            delay(robot.speed);
            ESP.wdtFeed(); //喂狗防止复位
            delayMicroseconds(del);
          } 
          for(int j=20;j<40;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda[39-j][i];

            Servo_p[15] = forwarda[39-j][15];
            Servo_p[16] = forwarda[39-j][16];
            Servo_p[17] = forwarda[39-j][17];

            //左边三条腿
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda[j-20][i];

            robot.framewrite(Servo_p);
            delay(robot.speed);
            ESP.wdtFeed(); //喂狗防止复位
          }
        }
        else if(body == 1)
        {
          for(int j=0;j<20;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda2[39-j][i];

            Servo_p[15] = forwarda2[39-j][15];
            Servo_p[16] = forwarda2[39-j][16];
            Servo_p[17] = forwarda2[39-j][17];

            //左边三条腿
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda2[20+j][i];

            robot.framewrite(Servo_p);
            delay(robot.speed);
            ESP.wdtFeed(); //喂狗防止复位
          } 
          for(int j=20;j<40;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda2[39-j][i];

            Servo_p[15] = forwarda2[39-j][15];
            Servo_p[16] = forwarda2[39-j][16];
            Servo_p[17] = forwarda2[39-j][17];

            //左边三条腿
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda2[j-20][i];

            robot.framewrite(Servo_p);
            delay(robot.speed);
            ESP.wdtFeed(); //喂狗防止复位
          }
        }
        else if(body == 2)
        {
          for(int j=0;j<20;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda3[39-j][i];

            Servo_p[15] = forwarda3[39-j][15];
            Servo_p[16] = forwarda3[39-j][16];
            Servo_p[17] = forwarda3[39-j][17];

            //左边三条腿
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda3[20+j][i];

            robot.framewrite(Servo_p);
            delay(robot.speed);
            ESP.wdtFeed(); //喂狗防止复位
          } 
          for(int j=20;j<40;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda3[39-j][i];

            Servo_p[15] = forwarda3[39-j][15];
            Servo_p[16] = forwarda3[39-j][16];
            Servo_p[17] = forwarda3[39-j][17];

            //左边三条腿
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda3[j-20][i];

            robot.framewrite(Servo_p);
            delay(robot.speed);
            ESP.wdtFeed(); //喂狗防止复位
          }
        }
      }
      else if(gait==1)//如果是波动步态
      {
          for(int j=0;j<60;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forwardaF2[59-j][i];

            Servo_p[15] = forwardaF2[59-j][15];
            Servo_p[16] = forwardaF2[59-j][16];
            Servo_p[17] = forwardaF2[59-j][17];
            
            //左边三条腿
            for(int i=6;i<15;i++)
            Servo_p[i] = forwardaF2[j][i];

            robot.framewrite(Servo_p);
            delay(robot.speed);
            ESP.wdtFeed(); //喂狗防止复位
            delay(0);
          }        
      }    
    }break;
    case 'f':
    {
      for(int j=0;j<60;j++)
      {
          robot.framewrite(forwardaFH[j]);
          delay(robot.speed);
          ESP.wdtFeed(); //喂狗防止复位
      }
    }break;
    case 'g':
    {
      for(int j=0;j<60;j++)
      {
          robot.framewrite(forwardaFH[59-j]);
          delay(robot.speed);
          ESP.wdtFeed(); //喂狗防止复位
      } 
    }break;
    case 'h':
    {
      gait=!gait;
    }break;
    case 'i':
    {
      body++;
      body=body%3;
    }break;
    case 'j':
    {
      for(int j=0;j<40;j++)
          {
              robot.framewrite(dunqia[j]);
              delay(deltr*10);
              ESP.wdtFeed(); //喂狗防止复位
          }
    }break;
    case 'k':
    {
      for(int j=0;j<40;j++)
          {
              robot.framewrite(benga[j]);
              delay(deltr*30);
              ESP.wdtFeed(); //喂狗防止复位
          }
    }break;
    default:
    {
      delay(100);
      ESP.wdtFeed(); //喂狗防止复位
    }
  }
}
/***********************机器人控制处理*********************/

/***********************舵机校准处理***********************/
void handleServoCalibration() {
  //检查串口缓存中是否有数据
  while (Serial.available() > 0) {
    //读取串口缓存中的数据到舵机当前的关节变量 cmd 中
    cmd = Serial.read();

    //如果命令是 'x'，执行角度校准
    if (cmd == 'x') {
      //再次检查串口缓存中是否有数据，检测在校准过程输入 'x' 后是否有后续命令执行
      while (Serial.available() > 0) {
        //解析舵机编号，Serial.parseInt() 的作用是读取一个 Int 型的数据，并将其作为参数传入 servo_id
        int servo_id = Serial.parseInt();

        //读取校准值存储于机器人关节标定变量数组
        robot.rec[servo_id] = Serial.read();

        //转换成 signed char（范围 -128~127），ESP8266 默认 char 是 unsigned char，范围 0~255
        robot.rec[servo_id] = Serial.parseInt();

        //将角度值循环遍历写入 EEPROM 的内存当中
        EEPROM.write(Servo_Addr + servo_id, robot.rec[servo_id]);
        EEPROM.commit();
      }

      //打印校准数据
      robot.printcaliData();

      //将校准值更新到 Servo_p 数组中
      for (int i = 0; i < 18; i++) {
        Servo_p[i] = float(rec[i]);
      }

      //将舵机移动到初始位置
      robot.framewrite(robot.Servo0);
      delay(100);
    }
  }
}
/***********************舵机校准处理***********************/

/***********************主程序入口setup()在Arduion中一般用于对程序的各个模块进行初始化操作,因为它只会执行一次***********************/
void setup() {
  /***********************定义各个关节舵机接口号***********************/
  //右侧关节
  robot.Jointservo[0] = 6;robot.Jointservo[1] = 7;robot.Jointservo[2] = 8;
  robot.Jointservo[3] = 9;robot.Jointservo[4] = 10;robot.Jointservo[5] = 11;
  robot.Jointservo[15] = 3;robot.Jointservo[16] = 4;robot.Jointservo[17] = 5;
  //左侧关节
  robot.Jointservo[6] = 25;robot.Jointservo[7] = 26;robot.Jointservo[8] = 27;
  robot.Jointservo[9] = 22;robot.Jointservo[10] = 23;robot.Jointservo[11] = 24;
  robot.Jointservo[12] = 19;robot.Jointservo[13] = 20;robot.Jointservo[14] = 21;
  /***********************定义各个关节舵机接口号***********************/


  /***********************设备初始化***********************/
  //设置LED引脚为输出模式
  pinMode(led, OUTPUT);        

  //LED为低电平时，灯亮；高电平时，灯灭
  digitalWrite(led, 1);

  //初始化串口通信，设置波特率为115200
  Serial.begin(115200);
  Serial.println();  //打印空行

  //开启IIC通信
  Wire.begin();  
  iic_device_test();  //在I2C上执行设备检测
  Serial.println("I2C---OK!");  //I2C通信成功，打印提示信息

  //启动两个PWM控制器，用于控制舵机
  pwm1.begin();
  pwm2.begin();  
  pwm1.setPWMFreq(50);  //设置PWM频率为50Hz，适用于模拟舵机
  pwm2.setPWMFreq(50);  
  Serial.println("\nPCA9685---OK!");  //PCA9685舵机芯片检测成功，打印提示信息

  //初始化EEPROM，设置其大小为0-4094字节存储下来0度的偏移量，这里不申请到4095字节，是为了防止内存溢出导致程序异常
  EEPROM.begin(4095);
  Serial.println("\nEEPROM---OK!"); //EEPROM初始化成功，打印提示信息

  //红外接收初始化
  irrecv.enableIRIn();
  Serial.println("\nIRrecv---OK!"); //红外接收初始化成功，打印提示信息

  //初始化OLED屏幕
  OLED_Init();
  OLED_ColorTurn(0); //0正常显示 1反色显示
  OLED_DisplayTurn(0); //0正常显示 1翻转180度显示
  Serial.println("\nOLED---OK!\n"); //OELD初始化成功，打印提示信息
  OLED_ShowString(16, 3, "DHT Disabled", 16); //显示温湿度功能未启用

  //读取EEPROM在地址4094的字节数据，用于后面来判断其是否已经初始化
  rec_lin = EEPROM.read(4094);

  //给读取留一个缓冲的时间确保数据读取完毕
  delay(1);

  //在EEPROM的地址4094读取一个字节数据，这一个代表初始化标志,127为已初始化，否则为未初始化
  if(rec_lin != 127)
  {
    //对机器人的舵机关节进行遍历，将其初始数据写入EEPROM
    for(int i=0;i<18;i++)
    {
      //将EEPROM的第Servo_Addr+i位置写入0
      EEPROM.write(Servo_Addr+i,0);

      /*在使用EEPROM.write(...)之后，其存入的数据是存放在数据缓冲区中，需要调用EEPROM.commit()确保数据写入EEPROM，再将数据写入EEPROM
      即这里实际上就是进行了一个确保数据正确保存在EEPROM的操作，否则如果程序异常退出，下次启动时可能读取到错误的数据*/
      EEPROM.commit();  

      //给写入留一个缓冲的时间确保数据写入完毕
      delay(1);    
    }
    //设置给EEPROM的地址4094写入一个字节数据，代表初始化标志为127
    EEPROM.write(4094,127);

    /*在使用EEPROM.write(...)之后，其存入的数据是存放在数据缓冲区中，需要调用EEPROM.commit()确保数据写入EEPROM，再将数据写入EEPROM
      即这里实际上就是进行了一个确保数据正确保存在EEPROM的操作，否则如果程序异常退出，下次启动时可能读取到错误的数据*/
    EEPROM.commit();  
  }
  
  //循环读取并打印出EEPROM中存储的所有舵机校准数据
  for(int i=0;i<18;i++)
  {
    rec_lin = EEPROM.read(i);
    rec[i] = rec_lin;
    Serial.print("|");
    Serial.print(rec[i]);
    Serial.print("|");
    delay(1);
  }  
  Serial.println();

  //将机器人当前关节变量临时值rec数组中的整数值转换为浮点数，并存储到Servo_p 数组中，用于设置舵机的当前位置
  for(int i=0;i<18;i++)
    Servo_p[i] = float(rec[i]);

  //喂狗操作，防止看门狗复位，确保程序不会因为响应时间过长而被自动重启
  ESP.wdtFeed();    

  //为舵机移动提供足够的响应时间 
  delay(100);

  //将LED引脚设为低电平,亮灯
  digitalWrite(led, 0);

  //初始化robot对象，包括设置初始状态、配置舵机等
  robot.init();
  robot.framewrite(zero_p); //将机器人零点位置zero_p数组的舵机角度写入到舵机控制器，设置舵机到初始位置
  delay(100); //延迟100毫秒，等待舵机移动到初始位置
  robot.framewrite(zero_p); //重复操作一次确保确实移动完成
  delay(100); //延迟100毫秒，等待舵机移动到初始位置

  //温湿度传感器初始化
  //dht.begin();
  dht.setup(DHT_PIN, DHTesp::DHT_TYPE); //初始化温湿度传感器
  delay(500); //给传感器稳定时间
  Serial.println("\n温湿度传感器初始化完成！"); //温湿度传感器初始化成功，打印提示信息
  /***********************设备初始化***********************/

  /***********************初始化完成后的收尾工作***********************/
  //在上述WIFI连接成功后延时0.5秒，确保舵机和连接成功
  delay(500);

  //将机器人当前关节变量Servo_p数组的所有元素初始化为浮点数0.0，这里Servo_p存储了18个舵机的目标角度
  for(int i=0;i<18;i++)
    Servo_p[i] = 0.0;
  /***********************初始化完成后的收尾工作***********************/


  /***********************从标定姿态过渡到前进准备姿态***********************/
  //将舵机目标角度设置为机器人当前关节变量Servo_p数组中的值，即将所有舵机置于设定的初始位置
  robot.framewrite(Servo_p);

  //将舵机从当前位置初始的机器人当前关节变量Servo_p到新的目标位置初始的三角步态forwarda[0]过渡
  robot.frame2frame(Servo_p,forwarda[0]);

  //这一步用于计算从当前姿态到前进准备姿态的舵机运动，舵机的目标角度设置为三角步态forwarda[0] 数组中的值，实现机器人的前进准备动作
  robot.framewrite(forwarda[0]);  
  /***********************从标定姿态过渡到前进准备姿态***********************/

  //启动TCP服务器
  server.begin();
  server.setNoDelay(true);//优化网络连接
}
/***********************主程序入口setup()在Arduion中一般用于对程序的各个模块进行初始化操作,因为它只会执行一次***********************/

/****************************主程序循环loop()在Arduion中一般用于程序的主循环,它会一直执行,直到程序结束*****************************/
void loop() {
  static unsigned long lastLoopTime = millis();
  static unsigned long lastCloudUpdate = 0;
  handleRobot();  //处理机器人动作
  handleIR();     //处理红外遥控
  blink();        //控制LED的闪烁
  handleWiFi();   //处理WiFi连接
  handleDHT();    //处理温湿度监测
  
  //阿里云保活（低优先级）
  if (millis() - lastCloudUpdate > 5000 && wifiConnected) {
    mqttClient.loop(); //MQTT心跳
    lastCloudUpdate = millis();
  }
  handleServoCalibration(); //处理舵机校准
  
  //控制循环频率在50Hz以上（20ms周期）
  if(millis() - lastLoopTime < 20) {
    delay(1);     //避免过度占用CPU
  }
  lastLoopTime = millis();
  yield();        //释放CPU时间片，确保其他任务能够及时执行
}
/****************************主程序循环loop()在Arduion中一般用于程序的主循环,它会一直执行,直到程序结束*****************************/