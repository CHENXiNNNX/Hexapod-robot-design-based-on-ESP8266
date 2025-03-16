Robot.ino 代码模块详解
1. 库函数定义
功能：引入依赖的第三方库和自定义头文件
关键库：
  机器人控制：Robot.h（自定义机器人核心逻辑）
  网络通信：ESP8266WiFi.h（WiFi 连接）、PubSubClient.h（MQTT 协议）、AliyunIoTSDK.h（阿里云 IoT 服务）
  硬件驱动：Adafruit_PWMServoDriver.h（舵机控制）、DHTesp.h（温湿度传感器）、IRremoteESP8266.h（红外接收）
  存储与时间：EEPROM.h（存储校准数据）、NTPClient.h（网络时间同步）

3. 全局变量定义
机器人控制：
  robot：机器人主对象，管理舵机动作和步态
  cmd 和 last_cmd：当前和上一个动作指令（如前进、后退、停止）
  gait 和 body：步态类型（波动/三角步态）和机身高度（低/中/高）
传感器与通信：
  irrecv：红外接收器对象，用于遥控指令
  dht：温湿度传感器对象，读取环境数据
  mqttClient 和 AliyunIoTSDK：阿里云 IoT 连接参数及 MQTT 配置
硬件配置：
  pwm1 和 pwm2：舵机驱动控制器（I2C 地址 0x40 和 0x41）
  SERVOMIN/MID/MAX：舵机脉冲宽度定义（对应 0°、90°、180°）

4. WiFi 模块
功能：管理 WiFi 连接和 TCP 服务器
关键变量：
  ssid 和 password：WiFi 的 SSID 和密码
  server：TCP 服务器（端口 8266），支持最多 3 个客户端连接
逻辑：
  自动重连机制，超时处理
  客户端指令解析（通过 TCP 接收动作指令）

5. 舵机控制
功能：驱动 18 个舵机，实现机器人关节运动
核心函数：
  iic_device_test()：检测 I2C 设备（舵机驱动器）是否正常
  handleServoCalibration()：通过串口校准舵机角度，数据存储至 EEPROM
关键数据：
  Servo_p 和 zero_p：当前舵机角度和零点位置
  direct：舵机方向反相参数，适配不同安装方向

6. 机器人动作处理
功能：解析指令并执行动作（如前进、转向、蹲起）
关键逻辑：
  步态控制：三角步态（forwarda）和波动步态（forwardaF2）的帧数据切换
  多高度适配：根据 body 值选择不同高度的步态数据
  实时控制：通过 handleRobot() 处理动作帧序列，控制舵机运动速度

7. 温湿度监测
功能：读取 DHT11 传感器数据并显示在 OLED 屏
关键逻辑：
  定时刷新数据（每 2 秒），阈值报警（温度 >25°C 或湿度 >50%）
  数据通过 MQTT 上传至阿里云 IoT 平台
  OLED 分阶段刷新显示（温度、湿度、警告信息）

8. 红外遥控
功能：解析红外信号并转换为控制指令
按键映射：
  方向上---a：前进
  方向下---b：后退
  方向左---f：左移
  方向右---g：右移
  确定键---e：停止机器人
  数字1----c：原地左转
  数字2----d：原地右转
  数字3----j：原地蹲起
  数字4----k：跳舞
  数字5------温湿度开关
  数字6------WIFI开关
  数字0------主控板重置
  星号键---h：步态切换
  ＃号键---i：高度切换

9. 阿里云 IoT 集成
功能：将温湿度数据上传至阿里云平台
关键配置：
  PRODUCT_KEY、DEVICE_NAME、DEVICE_SECRET：设备三元组
  AliyunIoTSDK::send()：发送数据到指定属性（如 humidity 和 temperature）

9. 时间同步
功能：通过 NTP 服务器同步系统时间
实现：syncTime() 调用 timeClient.update() 获取网络时间

10. 主程序结构
setup()：
  初始化硬件（舵机、传感器、OLED、WiFi等）
  加载 EEPROM 中的舵机校准数据
  启动 TCP 服务器和红外接收
loop()：
  周期性处理任务：机器人动作、红外指令、WiFi 连接、温湿度监测
  控制循环频率（50Hz），避免 CPU 过载
