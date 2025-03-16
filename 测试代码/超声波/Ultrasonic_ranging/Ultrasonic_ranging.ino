// HC-SR04 超声波传感器 ESP12F 示例代码
const int TrigPin = 12;  // GPIO12 (对应开发板上的D6)
const int EchoPin = 14;  // GPIO14 (对应开发板上的D7)
float cm;

void setup() {
  Serial.begin(115200);
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
}

void loop() {
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);

  // 读取回波高电平时间并转换为厘米
  cm = pulseIn(EchoPin, HIGH) / 58.0;
  cm = (int(cm * 100.0)) / 100.0; // 保留两位小数
  Serial.print(cm);
  Serial.print(" cm");
  Serial.println();
  delay(1000);
}