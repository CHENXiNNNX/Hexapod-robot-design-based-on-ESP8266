/*******************************************************
   功能：六足机器人及机械臂目标抓取程序从k210获取目标信息，然后操作机械臂进行抓取。
 *  程序前面的定义顺序：
 *  1.引入库文件
 *  2.定义宏
 *  3.定义变量
 *  4.新建对象
 *  5.定义函数
 *  void setup()和void loop()函数
*******************************************************/
#include <math.h>
#include <Robot.h>
#include <EEPROM.h>  //引入库文件
#include <SoftwareSerial.h>
#include "Kinematics.h"
#include <MatrixMath.h>
//2.定义宏----------------------------------------------------------------------
//关节标定数据存储起始位置
#define arm_eeprom 200
#define pi 3.14159
#define N 4  //矩阵阶数
// 定义软串口的引脚
#define RX_PIN 14
#define TX_PIN 12
#define magnet 16
#define leftfpin 13
#define rightfpin 0

//定义摄像头像素和传感器尺寸
#define XpixelL 320.0
#define YpixelL 240.0
#define XpixelL2 160.0
#define YpixelL2 120.0

#define XsensorL 4.5
#define YsensorL 3.4
#define XsensorL2 2.25
#define YsensorL2 1.7

#define focal 4.725         //摄像头焦距
//机械臂尺寸
#define L1 65.5
#define L2 80.0
#define L3 80.0

#define magfengxi 15
//#define maglength 46.86
#define maglength 72.00
#define zbuchang 10.5
//#define xbuchang 31.029
#define xbuchang 15.029

#define armh 85.5

#define pi4 0.78539816325  //pi/4

#define l1 34         //腿部连杆1
#define l2 43.51      //腿部连杆2
#define l3 93.07      //腿部连杆3

#define ll 94.93715   //身体斜线一半
#define lm 50.8       //身体中间横线一半

#define jointnum 6    //机器人连杆数量
//3.定义变量----------------------------------------------------------------------
// 定义结构体
struct Goalball {
  int targetColor;//目标颜色，1红色，2绿色，3蓝色，-1结束
  float xCoordinate;//x坐标
  float yCoordinate;//y坐标
  bool executionFlag;//是否已经取走，1为存储在目标结构体数组，0为不在
};

//定义10个
struct Goalball myball[10];

//MDH Parameter:alpha , a , theta , d
float Mdh[8][4] = {
  {0.0, 0.0, 0.0, 65.5},      //theta1
  {-pi/2, 0.0, -pi/2, 0.0},   //theta2
  {0.0, 80.0, pi/2, 0.0},     //theta3
  {0.0, 80.0, pi/2, 0.0},
  {0.0, 7.0, -pi/2, 0.0},
  {0.0, 44.355, -pi/2, 0.0},
  {-pi/2, 32.5, 0, 35.82},
  {pi/2, 0, 0.0, 0.0}
};
//相邻两个坐标系的齐次矩阵
mtx_type matrixOa8[8][4][4] = {
  {1.00 ,0.00 ,0.00 ,0.00},
  {0.00 ,1.00 ,0.00 ,0.00},
  {0.00 ,0.00 ,1.00 ,0.00},
  {0.00 ,0.00 ,0.00 ,1.00}
};

//当前坐标系相对于Oa0的齐次矩阵
mtx_type matrixOa08[8][4][4] = {
  {1.00 ,0.00 ,0.00 ,0.00},
  {0.00 ,1.00 ,0.00 ,0.00},
  {0.00 ,0.00 ,1.00 ,0.00},
  {0.00 ,0.00 ,0.00 ,1.00}
};
//目标相对于Oca8矩阵
mtx_type matrixOa89[4][4] = {
  {1.00 ,0.00 ,0.00 ,0.00},
  {0.00 ,1.00 ,0.00 ,0.00},
  {0.00 ,0.00 ,1.00 ,0.00},
  {0.00 ,0.00 ,0.00 ,1.00}
};
//电磁铁相对于Oa4坐标系齐次矩阵
mtx_type matrixOca410[4][4] = {
  {1.00 ,0.00 ,0.00 ,0.00},
  {0.00 ,1.00 ,0.00 ,0.00},
  {0.00 ,0.00 ,1.00 ,0.00},
  {0.00 ,0.00 ,0.00 ,1.00}
};
//相机坐标系到机械臂坐标系的变换;Pa=matrixOca*Pc
//目标相对于机械臂基坐标系齐次矩阵
mtx_type matrixOca09[4][4] = {
  {1.00 ,0.00 ,0.00 ,0.00},
  {0.00 ,1.00 ,0.00 ,0.00},
  {0.00 ,0.00 ,1.00 ,0.00},
  {0.00 ,0.00 ,0.00 ,1.00}
};

float alpha = 0;
float a = 0;
float theta = 0;
float d = 0;
//目标的像素坐标和颜色
int pixel_x,pixel_y,goalcolor;
float dir_pixel[2] = {-1,1};//坐标变换方向
//定义点Pc的坐标
mtx_type Pc[4][1] = {
  0,
  0,
  0,
  1
  };
//定义点Pa的坐标
mtx_type Pa[4][1] = {
  0,
  0,
  0,
  1
  };
float h_camera = 0.0;

//机械臂需要用到的变量
signed char rec_lin;
char cmd = 'e',last_cmd = 'e';

//机器人当前关节变量
float theta_p[4] = {
  0.0 , 0.0 , 0.0 , 0.0
};
//机器人目标关节变量
float theta_r[4] = {
  0.0 , 0.0 , 0.0 , 0.0
};
//机器人当前关节变量对应的舵机角度
float Servo_p[4] = {
  0.0 , 0.0 , 0.0 , 0.0
};
//机器人目标关节变量对应的舵机角度
float Servo_r[4] = {
  0.0 , 0.0 , 0.0 , 0.0
};

float Servo0[4] = {
  0.0 , 0.0 , 0.0 , 0.0
};

//关节变量方向与舵机方向变换
int dir[4] = {
  1 , 1 , -1 , 1
};

float body_yzuobiao[4][3]={
0.0 , 0.0 , 0.0 ,
0.0 , 0.0 , 0.0 ,
0.0 , 0.0 , 0.0 ,
0.0 , 0.0 , 0.0
};

float stepx = 5;
float stepy = 5;
float stepyl = 5.0;
float stepyr = 5.0;
float stepz = 5;
int forward_delay = 2;
int backward_delay = 2;
int leftward_delay = 5;
int rightward_delay = 5;
float baseanglex = 40.0;//机身倾角
float baseangley = 40.0;
float baseanglez = 50.0;
//关节变量
float thetah[6][3] = {
  0.0 ,0.0, 0.0,
  0.0 ,0.0, 0.0,
  0.0 ,0.0, 0.0,
  0.0 ,0.0, 0.0,
  0.0 ,0.0, 0.0,
  0.0 ,0.0, 0.0
};
//Endpose_3,这个点是foot相对于leg根部的坐标
float Endpose_3[6][4][1]=
{
  0.0 ,0.0 ,0.0 , 1.0,
  0.0 ,0.0 ,0.0 , 1.0,
  0.0 ,0.0 ,0.0 , 1.0,
  0.0 ,0.0 ,0.0 , 1.0,
  0.0 ,0.0 ,0.0 , 1.0,
  0.0 ,0.0 ,0.0 , 1.0
};
//六足机器人单腿逆解三个关节变量暂存数组
float theta_rh[3];
//六足机器人步态帧
float joint_ss[18]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float joint_ss1[18]={0.0,30.0,-30.0,0.0,30.0,-30.0,0.0,30.0,-30.0,0.0,30.0,-30.0,0.0,30.0,-30.0,0.0,30.0,-30.0};

//leg根部到foot的逆矩阵
mtx_type Base_footM[6][4][4];
mtx_type Base_legM[6][4][4];
mtx_type Leg_footM[6][4][4];

//4.创建对象----------------------------------------------------------------------
RobotClass robot;
// 创建软串口对象
SoftwareSerial espSerial(RX_PIN, TX_PIN);

RobotArmClass arm;//机械臂对象

//六条腿的运动学模型(MDH参数法)
Kinematics kinRmLeg(jointnum);
Kinematics kinRfLeg(jointnum);
Kinematics kinLfLeg(jointnum);
Kinematics kinLmLeg(jointnum);
Kinematics kinLbLeg(jointnum);
Kinematics kinRbLeg(jointnum);

//矩阵相关函数----------------------------------------------------------------------

//存储数据到数组
void storeDataInArray(struct Goalball newData) {
  // 遍历结构体数组
  for (int i = 0; i < 10; i++) {
    // 检查是否有空位置
    if (!myball[i].executionFlag) {
      // 将新的数据存储到数组中
      myball[i] = newData;
      myball[i].executionFlag = true;  // 设置 executionFlag 为 true，表示已存储
      Serial.println("Data stored in array.");
      return;  // 存储完成后退出函数
    }
  }

  // 如果没有空位置
  Serial.println("Array is full. Data not stored.");
}

//删除数组所有数据
void eraseDataInArray() {
  // 遍历结构体数组
  for (int i = 0; i < 10; i++) {
    // 检查是否有空位置
    myball[i].executionFlag = false;
  }

  // 如果没有空位置
  Serial.println("Array is erased. Data deleted.");
}

void printTargetArray() {
  Serial.println("Printing Target Array:");
  
  // 遍历结构体数组并输出每个元素的四个成员
  for (int i = 0; i < 10; i++) {
    //如果说有flag为0，那么说明打印完了，退出
    if(myball[i].executionFlag==0)
    return;
    Serial.print("Element ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print("Color: ");
    Serial.print(myball[i].targetColor);
    Serial.print(", X: ");
    Serial.print(myball[i].xCoordinate);
    Serial.print(", Y: ");
    Serial.print(myball[i].yCoordinate);
    Serial.print(", Flag: ");
    Serial.println(myball[i].executionFlag);
  }
  Serial.println("End of Target Array");
}
//获取k210数据相关函数----------------------------------------------------------------------
String GetDataFromK210()
{
    espSerial.flush();
    // 发送 "Robot Get!" 到软串口请求数据
    Serial.println("Send Request!");  // 输出接收到的字符串  
    delay(10); 
    espSerial.print("-Robot Get!-");
    delay(10);
    long time_lastk210 = millis();
    while(true)
    {
      if (espSerial.available() > 0) {
        // 如果串口有可用数据
        String receivedString = espSerial.readString();  // 读取所有字符串
        Serial.print("Received String: " + receivedString);  // 输出接收到的字符串
        return receivedString;
      }
      else
      {
        if((millis()-time_lastk210)>1000)
        {
          Serial.println("Send Request!");  // 输出接收到的字符串  
          delay(10);
          espSerial.print("-Robot Get!-");
          delay(10);
          time_lastk210 = millis();
        }
        delay(10);
      }
    }
}
//机械臂相关函数----------------------------------------------------------------------
//机械臂打印四个关节标定量
void print_jointjz(){
  Serial.println();
  Serial.println("The following is the BiaoDing :");
  Serial.print(" ");
  for(int i=0;i<4;i++)
  {
    Serial.print(i);
    Serial.print("    ");
  }
  Serial.println();
  for(int i=0;i<4;i++)
  {
    rec_lin = EEPROM.read(arm_eeprom+i);
    Serial.print(rec_lin);
    Serial.print("  ");
  }
  Serial.println();
}

//机械臂正运动学
void forwardKinetic(){
  
  body_yzuobiao[0][0] = 0.0;
  body_yzuobiao[0][1] = 0.0;
  body_yzuobiao[0][2] = 0.0;

  body_yzuobiao[1][0] = 0.0;
  body_yzuobiao[1][1] = 0.0;
  body_yzuobiao[1][2] = 65.50;

  body_yzuobiao[2][0] = L2*sin(theta_p[1]*pi/180.0)*cos(theta_p[0]*pi/180.0);
  body_yzuobiao[2][1] = L2*sin(theta_p[1]*pi/180.0)*sin(theta_p[0]*pi/180.0);
  body_yzuobiao[2][2] = body_yzuobiao[1][2] + L2*cos(theta_p[1]*pi/180.0);

  body_yzuobiao[3][0] = body_yzuobiao[2][0] + L3*cos((theta_p[1]+theta_p[2])*pi/180.0)*cos(theta_p[0]*pi/180.0);
  body_yzuobiao[3][1] = body_yzuobiao[2][1] + L3*cos((theta_p[1]+theta_p[2])*pi/180.0)*sin(theta_p[0]*pi/180.0);
  body_yzuobiao[3][2] = body_yzuobiao[2][2] - L3*sin((theta_p[1]+theta_p[2])*pi/180.0);

//  Serial.print("2x:");
//  Serial.print(body_yzuobiao[2][0]);
//  Serial.print("   ");
//  Serial.print("y:");
//  Serial.print(body_yzuobiao[2][1]);
//  Serial.print("   ");
//  Serial.print("z:");
//  Serial.print(body_yzuobiao[2][2]);
//  Serial.print("   ");
  
  Serial.print("3x:");
  Serial.print(body_yzuobiao[3][0]);
  Serial.print("   ");
  Serial.print("y:");
  Serial.print(body_yzuobiao[3][1]);
  Serial.print("   ");
  Serial.print("z:");
  Serial.println(body_yzuobiao[3][2]);
};

//机械臂逆运动学，传入参数为zuobiao[3]，目标的xyz三个坐标，
//解算结果存储在theta_r[0]，theta_r[1]，theta_r[2]这三个变量中,根据Oa4坐标逆解
void ik4a(float zuobiao[3]){
  float x3 = zuobiao[0];
  float y3 = zuobiao[1];
  float z3 = zuobiao[2];
  float z1 = L1;

  //先求θ1
  theta_r[0] = atan(y3/x3)*180.0/pi;
  //再求θ3
  float angleP321 = 0;//定义中间变量角∠P3P2P1，
  angleP321 = acos((L2*L2+L3*L3-x3*x3-y3*y3-z3*z3-z1*z1+2*z1*z3)/(2*L2*L3))*180.0/pi;
  theta_r[2] = 90 - angleP321;
  //再求θ2
  float angleP310 = 0.0;//定义中间变量角∠P3P1P0，
  float angleP312 = 0.0;//定义中间变量角∠P3P1P2，
  float P13 = 0.0;//定义P1P3
  P13 = sqrt(x3*x3+y3*y3+(z3-z1)*(z3-z1));
  angleP310 = atan((sqrt(x3*x3+y3*y3))/(z1-z3))*180.0/pi;
  if(angleP310<0)
  angleP310+=180.0;
  angleP312 = asin((L3/P13)*sin(angleP321*pi/180.0))*180.0/pi;

  theta_r[1] = 180 - angleP310 - angleP312;
  
  Serial.print("theta_r1:");
  Serial.print(theta_r[0]);
  Serial.print("   ");
  Serial.print("theta_r2:");
  Serial.print(theta_r[1]);
  Serial.print("   ");
  Serial.print("theta_r3:");
  Serial.println(theta_r[2]);  
  
}
//机械臂逆运动学，传入参数为zuobiao[3]，目标的xyz三个坐标，
//解算结果存储在theta_r[0]，theta_r[1]，theta_r[2]这三个变量中,根据Oa9坐标逆解
void ik9(float zuobiao[3]){
  float x3 = zuobiao[0];
  float y3 = zuobiao[1];
  float z3 = zuobiao[2];
  float z1 = L1;
  //求一下theta_0
  theta_r[0] = atan(y3/x3)*180.0/pi;
  
  x3 = x3 - xbuchang * cos(theta_r[0]*pi/180.0);//x
  y3 = y3 - xbuchang * sin(theta_r[0]*pi/180.0);//y
  z3 = z3 + magfengxi + maglength + zbuchang;//z,磁铁壳短了25.5 + armh - 25.5
  
  //先求θ1
  theta_r[0] = atan(y3/x3)*180.0/pi;
  //再求θ3
  float angleP321 = 0;//定义中间变量角∠P3P2P1，
  angleP321 = acos((L2*L2+L3*L3-x3*x3-y3*y3-z3*z3-z1*z1+2*z1*z3)/(2*L2*L3))*180.0/pi;
  theta_r[2] = 90 - angleP321;
  //再求θ2
  float angleP310 = 0.0;//定义中间变量角∠P3P1P0，
  float angleP312 = 0.0;//定义中间变量角∠P3P1P2，
  float P13 = 0.0;//定义P1P3
  P13 = sqrt(x3*x3+y3*y3+(z3-z1)*(z3-z1));
  angleP310 = atan((sqrt(x3*x3+y3*y3))/(z1-z3))*180.0/pi;
  if(angleP310<0)
  angleP310+=180.0;
  angleP312 = asin((L3/P13)*sin(angleP321*pi/180.0))*180.0/pi;

  theta_r[1] = 180 - angleP310 - angleP312;
  
//  Serial.print("theta_r1:");
//  Serial.print(theta_r[0]);
//  Serial.print("   ");
//  Serial.print("theta_r2:");
//  Serial.print(theta_r[1]);
//  Serial.print("   ");
//  Serial.print("theta_r3:");
//  Serial.println(theta_r[2]);  
}

void calculateMDHMatrix(float theta[4]) {
  Serial.println("Start MDH2 Transformer---------------");
  Mdh[0][2] = radians(theta[0]);
  Mdh[1][2] = -pi/2 + radians(theta[1]);
  Mdh[2][2] = pi/2 + radians(theta[2]);
  Mdh[4][2] = -pi/2 - radians(theta[1]) - radians(theta[2]);
  Mdh[5][2] = -pi/2 + radians(theta[3]);
  Mdh[7][2] = -radians(theta[3]);
  
  for(int i=0;i<8;i++)
  {
    float alpha, a, theta, d;
    // Convert degrees to radians
    alpha = Mdh[i][0];
    a = Mdh[i][1];
    theta = Mdh[i][2];
    d = Mdh[i][3];
    
    float alpha_rad = alpha;//radians(alpha);
    float theta_rad = theta;//radians(theta);
    //定义一个中间变量
    mtx_type zhongjian[4][4] = {
    {cos(theta_rad), -sin(theta_rad), 0, a},
    {sin(theta_rad)* cos(alpha_rad), cos(theta_rad) * cos(alpha_rad), -sin(alpha_rad), -d * sin(alpha_rad)},
    {sin(theta_rad)* sin(alpha_rad), cos(theta_rad)* sin(alpha_rad), cos(alpha_rad), d * cos(alpha_rad)},
    {0, 0, 0, 1}
  };
  //Matrix.Copy((mtx_type*)A, N, N, (mtx_type*)B);B=A;齐次矩阵赋值
  Matrix.Copy((mtx_type*)zhongjian, N, N, (mtx_type*)matrixOa8[i]);
  Matrix.Print((mtx_type*)matrixOa8[i], N, N, String(i+1));//打印中间齐次矩阵
  }
  Serial.println("End MDH2 Transformer---------------");
}

void TFforward()
{
  //计算matrixOca09
  //C=A*B;
  //Multiply(mtx_type* A, mtx_type* B, int m, int p, int n, mtx_type* C)
  Serial.println("Start Forward Transformer---------------");
  mtx_type zhongjian[4][4] = {
      {1.00 ,0.00 ,0.00 ,0.00},
      {0.00 ,1.00 ,0.00 ,0.00},
      {0.00 ,0.00 ,1.00 ,0.00},
      {0.00 ,0.00 ,0.00 ,1.00}
    };
    mtx_type jieguo[4][4] = {
      {1.00 ,0.00 ,0.00 ,0.00},
      {0.00 ,1.00 ,0.00 ,0.00},
      {0.00 ,0.00 ,1.00 ,0.00},
      {0.00 ,0.00 ,0.00 ,1.00}
    };
  for(int i=0;i<8;i++)
  {
//    Matrix.Print((mtx_type*)zhongjian, N, N, "zhongjian");//打印中间齐次矩阵
//    Matrix.Print((mtx_type*)matrixOa8[i], N, N, "zhongjian");//打印中间齐次矩阵
    Matrix.Multiply((mtx_type*)zhongjian, (mtx_type*)matrixOa8[i], N, N, N, (mtx_type*)jieguo);
    
    Matrix.Print((mtx_type*)jieguo, N, N, String(i+1));//打印中间齐次矩阵
    //Matrix.Copy((mtx_type*)A, N, N, (mtx_type*)B);B=A;齐次矩阵赋值
    Matrix.Copy((mtx_type*)jieguo, N, N, (mtx_type*)zhongjian);
    Matrix.Copy((mtx_type*)jieguo, N, N, (mtx_type*)matrixOa08[i]);
    
//    Matrix.Print((mtx_type*)zhongjian, N, N, String(i+1));//打印中间齐次矩阵
  }
  Serial.println("Start Forward Transformer---------------");
}
//关节空间到驱动空间
void theta2servo()
{
  Servo_p[0] = theta_p[0];
  Servo_p[1] = theta_p[1];
  Servo_p[2] = theta_p[2]+theta_p[1];
  Servo_p[3] = theta_p[3];      //变换关节空间theta_r到驱动空间Servo_r，30是开
  
  Servo_r[0] = theta_r[0];
  Servo_r[1] = theta_r[1];
  Servo_r[2] = theta_r[2]+theta_r[1];
  Servo_r[3] = theta_r[3];      //变换关节空间theta_r到驱动空间Servo_r，30是开
}

void getTheta_r()
{
  Serial.print("theta_r1:");
  Serial.print(theta_r[0]);
  Serial.print("   ");
  Serial.print("theta_r2:");
  Serial.print(theta_r[1]);
  Serial.print("   ");
  Serial.print("theta_r3:");
  Serial.print(theta_r[2]);
  Serial.print("   ");
  Serial.print("theta_r4:");
  Serial.println(theta_r[3]);
}

void go()
{
  theta2servo();
  arm.flame2frame(Servo_p,Servo_r); //驱动舵机到Servo_r
  arm.framewrite(Servo_r);             //驱动舵机到Servo_r
  theta_p[0]=theta_r[0];                 //记录当前Servo_p
  theta_p[1]=theta_r[1];
  theta_p[2]=theta_r[2];
  theta_p[3]=theta_r[3];
  
  Servo_p[0]=Servo_r[0];                 //记录当前Servo_p
  Servo_p[1]=Servo_r[1];
  Servo_p[2]=Servo_r[2];
  Servo_p[3]=Servo_r[3];
}

void gohome()
{
  theta_r[0] = 0.0;
  theta_r[1] = 0.0;
  theta_r[2] = 0.0;
  theta2servo();
  arm.flame2frame(Servo_p,Servo_r); //驱动舵机到Servo_r
  arm.framewrite(Servo_r);             //驱动舵机到Servo_r
  
  theta_p[0]=theta_r[0];                 //记录当前Servo_p
  theta_p[1]=theta_r[1];
  theta_p[2]=theta_r[2];
  theta_p[3]=theta_r[3];
  
  Servo_p[0]=Servo_r[0];                 //记录当前Servo_p
  Servo_p[1]=Servo_r[1];
  Servo_p[2]=Servo_r[2];
}

void shouqi()
{
  theta_r[0] = 0.0;
  theta_r[1] = -55.0;
  theta_r[2] = 70.0;
  theta2servo();
  arm.flame2frame(Servo_p,Servo_r); //驱动舵机到Servo_r
  arm.framewrite(Servo_r);             //驱动舵机到Servo_r
  theta_p[0]=theta_r[0];                 //记录当前Servo_p
  theta_p[1]=theta_r[1];
  theta_p[2]=theta_r[2];
  theta_p[3]=theta_r[3];
  
  Servo_p[0]=Servo_r[0];                 //记录当前Servo_p
  Servo_p[1]=Servo_r[1];
  Servo_p[2]=Servo_r[2];
}

//判断是否有目标且为红色
bool isGoal()
{
  for (int i = 0; i < 10; ++i) {
    if ((myball[i].executionFlag == 1)&&(myball[i].targetColor == 2)) 
      return true;
    }
    return false;
}
//判断是否有目标且为红色
bool isRedGoal()
{
  for (int i = 0; i < 10; ++i) {
    if ((myball[i].executionFlag == 1)&&(myball[i].targetColor == 2)) 
      return true;
    }
    return false;
}

//抓取目标,
bool getGoal()
{
  //接下来，像素坐标转换到相机坐标系，相机坐标系，再转换到机械臂坐标系。
  //这里找一个目标，查找第一个存储的结构体数据,然后进行坐标系转换
  for (int i = 0; i < 10; ++i) {
    //如果有目标且为红色
    if ((myball[i].executionFlag == 1)&&(myball[i].targetColor == 2)) {
      Serial.print("--------The ");Serial.print(i);Serial.println("Color kuai--------");
      // 打印颜色和坐标信息
      Serial.print("Color: ");
      Serial.println(myball[i].targetColor);
      Serial.print("X Coordinate: ");
      Serial.println(myball[i].xCoordinate);
      Serial.print("Y Coordinate: ");
      Serial.println(myball[i].yCoordinate);
      
      pixel_x = myball[i].xCoordinate;
      pixel_y = myball[i].yCoordinate;
      goalcolor = myball[i].targetColor;
      delay(1000);  // 延迟1秒，避免连续输出

  theta_r[0] = 0.0;
  theta_r[1] = 0.0;
  theta_r[2] = 0.0;
  theta_r[3] = 90.0;
  calculateMDHMatrix(theta_r);
  TFforward();
  h_camera = matrixOa08[7][2][3];
  //像素坐标转相机坐标：
  //坐标x   x=(h/f)*(XsensorL2/XpixelL2)*(pixel_x-120);
  Pc[0][0] =  -h_camera-armh;
  //坐标y   y=(h/f)*(YsensorL2/YpixelL2)*(pixel_y-160);
  Pc[1][0] =  ((h_camera+armh)/focal)*(XsensorL2/XpixelL2)*(pixel_x-160.0)*dir_pixel[0];
  Pc[2][0] = ((h_camera+armh)/focal)*(YsensorL2/YpixelL2)*(pixel_y-120.0)*dir_pixel[1];//坐标z
  Pc[3][0] = 1;
  
  Matrix.Print((mtx_type*)matrixOa08[7], 4, 4, "Oca");
  Matrix.Print((mtx_type*)Pc, 4, 1, "Pc");
  Matrix.Print((mtx_type*)Pa, 4, 1, "Pa");
  //矩阵 A 的行数    矩阵 A 的列数（也是矩阵 B 的行数）   矩阵 B 的列数。
  Matrix.Multiply((mtx_type*)matrixOa08[7], (mtx_type*)Pc, 4, 4, 1, (mtx_type*)Pa);
  Matrix.Print((mtx_type*)Pa, 4, 1, "Pa");

  float zuobiao[3];//给定目标坐标
  zuobiao[0] = Pa[0][0];//x
  zuobiao[1] = Pa[1][0];//y
  zuobiao[2] = Pa[2][0];//z
  delay(1000);
  theta_r[3] = 0.0;
  ik9(zuobiao);  
  
  go();
  
  delay(1000);
  digitalWrite(magnet,HIGH);//吸起来
  delay(1000);

  shouqi();
//  zuobiao[2] = 60.0;//z
//  ik9(zuobiao);  
//  go();
//  delay(1000);
//
//  zuobiao[0] = 100.0;//x
//  zuobiao[1] = 80.0;//y
//  zuobiao[2] = 60.0;//z
//  ik9(zuobiao);  
//  go();
//  delay(1000);
//  digitalWrite(magnet,LOW);//吸起来
//  delay(1000);
//  
//  gohome();
//  arm.jointwrite(0,0);
//  arm.jointwrite(1,0);
//  arm.jointwrite(2,0);
//  arm.jointwrite(3,90);
  delay(500);
  myball[i].executionFlag = 0;
//      break;  // 找到第一个存储的数据后退出循环
//  return true;
    }
  }
  return false;
}

//放置目标
bool putGoal()
{
  //接下来，像素坐标转换到相机坐标系，相机坐标系，再转换到机械臂坐标系。
  //这里找一个目标，查找第一个存储的结构体数据,然后进行坐标系转换
  for (int i = 0; i < 10; ++i) {
    //如果有目标且为红色
    if ((myball[i].executionFlag == 1)&&(myball[i].targetColor == 2)) {
      Serial.print("--------The ");Serial.print(i);Serial.println("Color kuai--------");
      // 打印颜色和坐标信息
      Serial.print("Color: ");
      Serial.println(myball[i].targetColor);
      Serial.print("X Coordinate: ");
      Serial.println(myball[i].xCoordinate);
      Serial.print("Y Coordinate: ");
      Serial.println(myball[i].yCoordinate);
      
      pixel_x = myball[i].xCoordinate;
      pixel_y = myball[i].yCoordinate;
      goalcolor = myball[i].targetColor;
      delay(1000);  // 延迟1秒，避免连续输出

      theta_r[0] = 0.0;
      theta_r[1] = 0.0;
      theta_r[2] = 0.0;
      theta_r[3] = 90.0;
      calculateMDHMatrix(theta_r);
      TFforward();
      h_camera = matrixOa08[7][2][3];
      //像素坐标转相机坐标：
      //坐标x   x=(h/f)*(XsensorL2/XpixelL2)*(pixel_x-120);
      Pc[0][0] =  -h_camera-armh;
      //坐标y   y=(h/f)*(YsensorL2/YpixelL2)*(pixel_y-160);
      Pc[1][0] =  ((h_camera+armh)/focal)*(XsensorL2/XpixelL2)*(pixel_x-160.0)*dir_pixel[0];
      Pc[2][0] = ((h_camera+armh)/focal)*(YsensorL2/YpixelL2)*(pixel_y-120.0)*dir_pixel[1];//坐标z
      Pc[3][0] = 1;
      
      Matrix.Print((mtx_type*)matrixOa08[7], 4, 4, "Oca");
      Matrix.Print((mtx_type*)Pc, 4, 1, "Pc");
      Matrix.Print((mtx_type*)Pa, 4, 1, "Pa");
      //矩阵 A 的行数    矩阵 A 的列数（也是矩阵 B 的行数）   矩阵 B 的列数。
      Matrix.Multiply((mtx_type*)matrixOa08[7], (mtx_type*)Pc, 4, 4, 1, (mtx_type*)Pa);
      Matrix.Print((mtx_type*)Pa, 4, 1, "Pa");

      float zuobiao[3];//给定目标坐标
      zuobiao[0] = Pa[0][0];//x
      zuobiao[1] = Pa[1][0];//y
      zuobiao[2] = Pa[2][0]+50.0;//z,多抬高50，以便放在被子里
      delay(1000);
      theta_r[3] = 0.0;
      ik9(zuobiao);  
      
      go();
      
      delay(1000);
      digitalWrite(magnet,LOW);//放下来
      delay(1000);

      shouqi();
    //  zuobiao[2] = 60.0;//z
    //  ik9(zuobiao);  
    //  go();
    //  delay(1000);
    //
    //  zuobiao[0] = 100.0;//x
    //  zuobiao[1] = 80.0;//y
    //  zuobiao[2] = 60.0;//z
    //  ik9(zuobiao);  
    //  go();
    //  delay(1000);
    //  digitalWrite(magnet,LOW);//吸起来
    //  delay(1000);
    //  
    //  gohome();
    //  arm.jointwrite(0,0);
    //  arm.jointwrite(1,0);
    //  arm.jointwrite(2,0);
    //  arm.jointwrite(3,90);
      delay(1000);
      myball[i].executionFlag = 0;
    //  break;  // 找到第一个存储的数据后退出循环
      return true;
    }
  }
  return false;
}

//从串口获取帧数据并解析
void getData()
{
    // 如果串口有可用数据
//    String receivedString = Serial.readString();  // 读取所有字符串
//    Serial.println("Start Get request!" + receivedString);  // 输出接收到的字符串
    eraseDataInArray();
    String receivedString;
    receivedString = GetDataFromK210();
    // 在这里对接收到的字符串进行处理,统计帧数，根据字母E统计
    Serial.println(receivedString.length());
    char targetChar = 'E';  // 要查找的字符
      // 计算字符串中字符 'E' 的数量
    int countE = 0;
    for (int i = 0; i < receivedString.length(); i++) {
      if (receivedString.charAt(i) == targetChar) {
        countE++;
      }
    }
    // 输出结果到串口
    Serial.print("Number of 'E' in the string: ");
    Serial.println(countE);

    //解析帧数
    char startChar = 'm';  // 起始字符
    char endChar = 'n';    // 结束字符
    // 找到不包含 'm' 和 'n' 的子字符串
    int startIndex = receivedString.indexOf(startChar);
    int endIndex = receivedString.indexOf(endChar, startIndex);
    // 输出结果到串口
    if (startIndex != -1 && endIndex != -1) {
      String resultSubstring = receivedString.substring(startIndex + 1, endIndex);
      Serial.print("Substring between 'm' and 'n': ");
      Serial.println(resultSubstring);
    } else {
      Serial.println("Substring not found");
    }
    //解析出来的帧数
    String rSubstring = receivedString.substring(startIndex + 1, endIndex);
    int jiexigait = atoi(rSubstring.c_str());
    Serial.println(jiexigait);
    
    //找出每帧数据
    startIndex = 0;
    endIndex = 0;
    while (true) {
    // 在字符串中查找以 's' 开头的位置
    // 用于找到字符 's' 在 receivedString 中的位置，搜索起点是上一次找到的 E 的位置。
    startIndex = receivedString.indexOf('s', endIndex);

    // 如果找不到 's'，退出循环
    if (startIndex == -1) {
      break;
    }

    // 在字符串中查找以 'E' 结尾的位置
    endIndex = receivedString.indexOf('E', startIndex);

    // 如果找不到 'E'，退出循环
    if (endIndex == -1) {
      break;
    }

    // 提取数据帧并输出到串口
    String dataFrame = receivedString.substring(startIndex, endIndex + 1);
    Serial.println("Found Data Frame: " + dataFrame);

      // 定义字符串，将字符串String转化为字符指针char*
    const char* inputString = dataFrame.c_str();
  
    // 定义变量
    char color = 'm';
    int xCoordinate = 0.0;
    int yCoordinate = 0.0;
  
    // 使用 sscanf 解析字符串,返回参数result表示匹配到的参数个数，里面有三个，都匹配上了就是3
    int result = sscanf(inputString, "s%cx%dy%dE", &color, &xCoordinate, &yCoordinate);
  
    // 判断解析结果并打印
    if (result == 3 && (color == 'g' || color == 'r' || color == 'y')) {
      Serial.print("Color: ");
      if (color == 'g') {
        Serial.println("1 (Green)");
      } else if (color == 'r') {
        Serial.println("2 (Red)");
      } else if (color == 'b') {
        Serial.println("3 (Blue)");
      }
  
      Serial.print("X Coordinate: ");
      Serial.println(xCoordinate);
  
      Serial.print("Y Coordinate: ");
      Serial.println(yCoordinate);
    } else {
      Serial.println("Invalid input string.");
    }
    //从dataFrame数据帧提取数据，存入newTarget结构体
    struct Goalball newTarget;
    switch(color)
    {
      case 'g':newTarget.targetColor = 1;break;
      case 'r':newTarget.targetColor = 2;break;
      case 'b':newTarget.targetColor = 3;break;
      default:break;
    }
    newTarget.xCoordinate = xCoordinate;
    newTarget.yCoordinate = yCoordinate;
    newTarget.executionFlag = false; 
    
    storeDataInArray(newTarget);
  }
  
  printTargetArray();
}

bool SerchRedgoal()
{
  theta_r[0] = 60.0;
  theta_r[1] = -55.0;
  theta_r[2] = 70.0;
  theta_r[3] = 30.0;//摄像头从30-70度扫描，机械臂从60到-60度扫描。
  go();
  for(int i=60;i>=-60;i-=60)
  {
    for(int j=30;j<=70;j+=20)
    {
      theta_r[0] = i;
      theta_r[1] = -55.0;
      theta_r[2] = 70.0;
      theta_r[3] = j;//摄像头从30-70度扫描，机械臂从60到-60度扫描。
      go();
      getData();
      if(isRedGoal())
      return true;
      delay(100);
    }
  }
  return false;
}

void trackRedGoal()
{
  while(true)
  {
    getData();
    if(isRedGoal())
    {
      for (int i = 0; i < 10; ++i) 
      {
        if ((myball[i].executionFlag == 1)&&(myball[i].targetColor == 2)) 
        {
          if(myball[i].xCoordinate > 165)
          {
            theta_r[3] += 1;
            arm.jointwrite(3,theta_r[3]);
            theta_p[0]=theta_r[0];                 //记录当前Servo_p
            theta_p[1]=theta_r[1];
            theta_p[2]=theta_r[2];
            theta_p[3]=theta_r[3];
            
            Servo_p[0]=Servo_r[0];                 //记录当前Servo_p
            Servo_p[1]=Servo_r[1];
            Servo_p[2]=Servo_r[2];
//            go();
          }
          else if(myball[i].xCoordinate < 155)
          {
            theta_r[3] -= 1;
            arm.jointwrite(3,theta_r[3]);
            theta_p[0]=theta_r[0];                 //记录当前Servo_p
            theta_p[1]=theta_r[1];
            theta_p[2]=theta_r[2];
            theta_p[3]=theta_r[3];
            
            Servo_p[0]=Servo_r[0];                 //记录当前Servo_p
            Servo_p[1]=Servo_r[1];
            Servo_p[2]=Servo_r[2];
//            go();
          }
          if(myball[i].yCoordinate > 125)
          {
            theta_r[0] += 1;
            arm.jointwrite(0,theta_r[0]);
            theta_p[0]=theta_r[0];                 //记录当前Servo_p
            theta_p[1]=theta_r[1];
            theta_p[2]=theta_r[2];
            theta_p[3]=theta_r[3];
            
            Servo_p[0]=Servo_r[0];                 //记录当前Servo_p
            Servo_p[1]=Servo_r[1];
            Servo_p[2]=Servo_r[2];
//            go();
          }
          else if(myball[i].yCoordinate < 115)
          {
            theta_r[0] -= 1;
            arm.jointwrite(0,theta_r[0]);
            theta_p[0]=theta_r[0];                 //记录当前Servo_p
            theta_p[1]=theta_r[1];
            theta_p[2]=theta_r[2];
            theta_p[3]=theta_r[3];
            
            Servo_p[0]=Servo_r[0];                 //记录当前Servo_p
            Servo_p[1]=Servo_r[1];
            Servo_p[2]=Servo_r[2];
//            go();
          }
          eraseDataInArray();
        }
      }
      delay(100);
    }
  }
}


//六足机器人相关函数---------------------------------------------------------------------
void printFootPoint()
{
  kinRmLeg.forwardMDH();
  kinRfLeg.forwardMDH();
  kinLfLeg.forwardMDH();
  kinLmLeg.forwardMDH();
  kinLbLeg.forwardMDH();
  kinRbLeg.forwardMDH();
  
  kinRmLeg.PrintEnd_effectorPoint();
  kinRfLeg.PrintEnd_effectorPoint();
  kinLfLeg.PrintEnd_effectorPoint();
  kinLmLeg.PrintEnd_effectorPoint();
  kinLbLeg.PrintEnd_effectorPoint();
  kinRbLeg.PrintEnd_effectorPoint();
}

void printBaseFootpose()
{
  char buffer[50]; // Adjust the size based on your needs
  Serial.println("---------------------------------");
  for(int i=0;i<6;i++)
  {
    Serial.print("Leg Base_footPoint:");
    Serial.print("|");
    snprintf(buffer, sizeof(buffer), " x: %-9.2f y: %-9.2f z: %-9.2f ", Base_footM[i][0][3], Base_footM[i][1][3], Base_footM[i][2][3]);
    Serial.print(buffer);
    Serial.println(); 
  }
}

void printLegFootpose()
{
  char buffer[50]; // Adjust the size based on your needs
  for(int i=0;i<6;i++)
  {
    Serial.print("Leg End_effectorPoint:");
    Serial.print("|");
    snprintf(buffer, sizeof(buffer), " x: %-9.2f y: %-9.2f z: %-9.2f ", Endpose_3[i][0][0], Endpose_3[i][1][0], Endpose_3[i][2][0]);
    Serial.print(buffer);
    Serial.println(); 
  }
}

//求Endpose_3,这个点是foot相对于leg根部的坐标
void CalLeg_footpose()
{
  //首先找到leg根部到baselink矩阵，赋值给Base_legM
  Matrix.Copy((mtx_type*)kinRmLeg.MatrixOb[2], N, N, (mtx_type*)Base_legM[0]);//B=A;齐次矩阵赋值
  Matrix.Copy((mtx_type*)kinRfLeg.MatrixOb[2], N, N, (mtx_type*)Base_legM[1]);//B=A;齐次矩阵赋值
  Matrix.Copy((mtx_type*)kinLfLeg.MatrixOb[2], N, N, (mtx_type*)Base_legM[2]);//B=A;齐次矩阵赋值
  Matrix.Copy((mtx_type*)kinLmLeg.MatrixOb[2], N, N, (mtx_type*)Base_legM[3]);//B=A;齐次矩阵赋值
  Matrix.Copy((mtx_type*)kinLbLeg.MatrixOb[2], N, N, (mtx_type*)Base_legM[4]);//B=A;齐次矩阵赋值
  Matrix.Copy((mtx_type*)kinRbLeg.MatrixOb[2], N, N, (mtx_type*)Base_legM[5]);//B=A;齐次矩阵赋值
  
  //然后求leg根部到baselink逆矩阵
  Matrix.Invert((mtx_type*)Base_legM[0], N);
  Matrix.Invert((mtx_type*)Base_legM[1], N);
  Matrix.Invert((mtx_type*)Base_legM[2], N);
  Matrix.Invert((mtx_type*)Base_legM[3], N);
  Matrix.Invert((mtx_type*)Base_legM[4], N);
  Matrix.Invert((mtx_type*)Base_legM[5], N);

  //然后逆矩阵右边乘以MatrixOb,得到的就是foot到leg根部的矩阵
  //C=A*B;
  //Multiply(mtx_type* A, mtx_type* B, int m, int p, int n, mtx_type* C)
  //Leg_footM = Base_legM-1 *(Base_legM * Leg_FootM) = Base_legM-1 * Base_FootM
  Matrix.Multiply((mtx_type*)Base_legM[0], (mtx_type*)kinRmLeg.MatrixOb[5], N, N, N, (mtx_type*)Leg_footM[0]);
  Matrix.Multiply((mtx_type*)Base_legM[1], (mtx_type*)kinRfLeg.MatrixOb[5], N, N, N, (mtx_type*)Leg_footM[1]);
  Matrix.Multiply((mtx_type*)Base_legM[2], (mtx_type*)kinLfLeg.MatrixOb[5], N, N, N, (mtx_type*)Leg_footM[2]);
  Matrix.Multiply((mtx_type*)Base_legM[3], (mtx_type*)kinLmLeg.MatrixOb[5], N, N, N, (mtx_type*)Leg_footM[3]);
  Matrix.Multiply((mtx_type*)Base_legM[4], (mtx_type*)kinLbLeg.MatrixOb[5], N, N, N, (mtx_type*)Leg_footM[4]);
  Matrix.Multiply((mtx_type*)Base_legM[5], (mtx_type*)kinRbLeg.MatrixOb[5], N, N, N, (mtx_type*)Leg_footM[5]);

  //foot到leg根部的齐次矩阵里面最后一列就是坐标了
  for(int i=0;i<6;i++)
  {
    Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
    Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
    Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
    Endpose_3[i][3][0] = 1.0;
  }
//  printLegFootpose();
  //根据坐标逆解关节变量，OK了。
  float zuobiao1[3];
  zuobiao1[0] = Endpose_3[0][0][0];
  zuobiao1[1] = Endpose_3[0][1][0];
  zuobiao1[2] = Endpose_3[0][2][0];

  ik4(zuobiao1);
  ik4(zuobiao1);
  ik4(zuobiao1);
  ik4(zuobiao1);
  ik4(zuobiao1);
  ik4(zuobiao1);
  
}

//机械臂逆运动学，传入参数为zuobiao[3]，目标的xyz三个坐标，
//解算结果存储在theta_rh[0]，theta_rh[1]，theta_rh[2]这三个变量中,根据Oa4坐标逆解
void ik4(float zuobiao[3]){
  float x3,y3,z3;
  float x1,y1,z1,P1P3_2,P1P3,P321;
  float P210,P213,P310,P3P0_2,P3P0;//P3P0_2是P3P0的平方P310是角P310
  x3 = zuobiao[0];
  y3 = zuobiao[1];
  z3 = zuobiao[2];

  //先求θ1
  theta_rh[0] = atan(y3/x3);
  //再求θ3
  x1 = l1 * cos(theta_rh[0]);
  y1 = l1 * sin(theta_rh[0]);
  z1 = 0.0;
  P1P3_2 = (x3-x1)*(x3-x1)+(y3-y1)*(y3-y1)+(z3-z1)*(z3-z1);
  P1P3 = sqrt(P1P3_2);
  P321=acos((l2*l2+l3*l3-P1P3_2)/(2*l2*l3));
  theta_rh[2] = P321-PI/2;
  
  //最后再求θ2,这里一定记得用余弦定理
  P213=acos((l2*l2+P1P3_2-l3*l3)/(2*l2*P1P3));
  P3P0_2=x3*x3+y3*y3+z3*z3;
  P3P0 = sqrt(P3P0_2);
  P310 = acos((l1*l1+P1P3_2-P3P0_2)/(2*l1*P1P3));
  theta_rh[1] = P213 + P310 - PI;
  
//  Serial.print("theta_rh1:");
//  Serial.print(theta_rh[0]);
//  Serial.print("   ");
//  Serial.print("theta_rh2:");
//  Serial.print(theta_rh[1]);
//  Serial.print("   ");
//  Serial.print("theta_rh3:");
//  Serial.println(theta_rh[2]);  
  
}

//前进起步
void StartForward()
{
  
  ForwardKinematics_GetBasefootM();
  //    kinRmLeg.PrintJointstate();
//    printFootPoint();
//    printBaseFootpose();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] - x/2;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
      //左前腿
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] - x/2;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
      //左后腿
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] - x/2;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(20);
    }
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] + x/2;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
      //左中腿
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] + x/2;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
      //右后腿
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] + x/2;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(20);
    }
}

//前进收腿
void EndForward()
{
  
  ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] + x/2 - stepx*PI;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
      //右前腿
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] + stepx*PI;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] ;
      //左前腿
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] + x/2 - stepx*PI;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
      //左中腿
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] + stepx*PI;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] ;
      //左后腿
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] + x/2 - stepx*PI;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
      //右后腿
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] + stepx*PI;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] ;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(20);
    }
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右前腿
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] - x/2 + stepx*PI;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
      //左中腿
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] - x/2 + stepx*PI;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
      //右后腿
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] - x/2 + stepx*PI;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(20);
    }
}

//这个函数控制机器人前进多少距离
void Robot_forward_y(float ylength)
{
  //首先确定每一步走多少，接下来计算一共走几步，最后一步走多远
  ylength = ylength/1.71;//奇怪的参数，需要调整一下1.6552
  stepx = 5.0;
  int m;
  m = (int)(ylength/(stepx*2*PI));
  //起步
  Serial.println("StartForward");
  StartForward();

  //循环走路
  //步子距离2*PI*stepx
  for(int k = 0;k<m;k++)
  {
    Serial.println("Forward");
    ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3];
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3];
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3];
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(forward_delay);
    }
  
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3];
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3];
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3];
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;

//      printBaseFootpose();
      Forward_GetBaselegMsv();
    
      GetLegfootM();  
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(forward_delay);
    }
  }
  //收腿
    EndForward();

    
  //最后一步
  stepx = (ylength - 2*PI*stepx*m)/(2*PI);
  StartForward();
  ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3];
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3];
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3];
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(forward_delay);
    }
  
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3];
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3];
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3];
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
    
      GetLegfootM();  
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(forward_delay);
    }
    Serial.println("EndForward");
    //收腿
    EndForward();
  
}

//这个函数控制机器人前进多少距离(非对称步态)
void Robot_forward_y1(float ylength)
{
  //首先确定每一步走多少，接下来计算一共走几步，最后一步走多远
  ylength = ylength/1.71;//奇怪的参数，需要调整一下1.6552
  stepx = 5.0;
  int m;
  m = (int)(ylength/(stepx*2*PI));
  //循环走路,步子距离2*PI*stepx
  for(int k = 0;k<m;k++)
  {
    ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] + x;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] - x;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3];
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] + x;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] - x;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3];
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] + x;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] - x;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3];
      printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(forward_delay);
    }
  
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] - x + 2*PI*stepx;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3];
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] + x - 2*PI*stepx;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] - x + 2*PI*stepx;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3];
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] + x - 2*PI*stepx;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] - x + 2*PI*stepx;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3];
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] + x - 2*PI*stepx;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;

      printBaseFootpose();
      Forward_GetBaselegMsv();
    
      GetLegfootM();  
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(forward_delay);
    }
  }
  //最后一步
  stepx = (ylength - 2*PI*stepx*m)/(2*PI);
  ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] + x;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] - x;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3];
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] + x;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] - x;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3];
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] + x;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] - x;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3];
      printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(forward_delay);
    }
  
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] - x + 2*PI*stepx;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3];
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] + x - 2*PI*stepx;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] - x + 2*PI*stepx;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3];
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] + x - 2*PI*stepx;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] - x + 2*PI*stepx;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3];
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] + x - 2*PI*stepx;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
      printBaseFootpose();
      Forward_GetBaselegMsv();
    
      GetLegfootM();  
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(forward_delay);
    }
}

//这个函数控制机器人前进一小步，一步前进多少距离(非对称步态)，当前进距离较小时，用这个函数
void Robot_forward_y2(float ylength)
{
  //首先确定每一步走多少，接下来计算一共走几步，最后一步走多远
  ylength = ylength/1.71;//奇怪的参数，需要调整一下1.6552
  stepx = 5.0;
  //循环走路,步子距离2*PI*stepx
  //最后一步
  stepx = ylength/(2*PI);
  ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
//      Serial.println(millis());
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] + x;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] - x;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3];
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] + x;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] - x;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3];
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] + x;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] - x;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3];
//      printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(forward_delay);
    }
  
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] - x + 2*PI*stepx;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3];
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] + x - 2*PI*stepx;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] - x + 2*PI*stepx;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3];
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] + x - 2*PI*stepx;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] - x + 2*PI*stepx;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3];
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] + x - 2*PI*stepx;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
//      printBaseFootpose();
      Forward_GetBaselegMsv();
    
      GetLegfootM();  
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(forward_delay);
    }
}

//这个函数控制机器人前进多少距离(非对称步态,左右步距不同),这个步态会打滑，会摩擦，落足点有问题。
//这个是根据改变步态左右两边的行走步距来达到转弯效果的，那左右步距不一样，不一定能配合好，可能跟
//身子产生冲突，所以不能用这种方法控制机器人。应该根据机身的移动计算落足点，而不是通过控制落足点控制机身。
void Robot_forward_y3(float ylength)
{
  //首先确定每一步走多少，接下来计算一共走几步，最后一步走多远
  ylength = ylength/1.71;//奇怪的参数，需要调整一下1.6552
  stepy = 5.0;
  stepyr = 5.0;
  stepyl = 1.0;
  int m;
  m = (int)(ylength/(stepy*2*PI));
  Serial.println("Forward");
  //循环走路,步子距离2*PI*stepx
  for(int k = 0;k<m;k++)
  {
    Serial.println("Forward");
    ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float y=stepy*(t-sin(t));
      float yr=stepyr*(t-sin(t));
      float yl=stepyl*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] + yr;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] - yr;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3];
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] + yl;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] - yl;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3];
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] + yl;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] - yr;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3];
//      printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(forward_delay);
    }
  
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float y=stepy*(t-sin(t));
      float yr=stepyr*(t-sin(t));
      float yl=stepyl*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] - yr + 2*PI*stepyr;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3];
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] + yr - 2*PI*stepyr;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] - yl + 2*PI*stepyl;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3];
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] + yl - 2*PI*stepyl;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] - yl + 2*PI*stepyl;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3];
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] + yr - 2*PI*stepyr;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;

//      printBaseFootpose();
      Forward_GetBaselegMsv();
    
      GetLegfootM();  
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(forward_delay);
    }
  }
  //最后一步
  stepx = (ylength - 2*PI*stepx*m)/(2*PI);
  ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float y=stepy*(t-sin(t));
      float yr=stepyr*(t-sin(t));
      float yl=stepyl*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] + yr;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] - yr;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3];
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] + yl;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] - yl;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3];
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] + yl;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] - yr;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3];
//      printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(forward_delay);
    }
  
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float y=stepy*(t-sin(t));
      float yr=stepyr*(t-sin(t));
      float yl=stepyl*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] - yr + 2*PI*stepyr;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3];
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] + yr - 2*PI*stepyr;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] - yl + 2*PI*stepyl;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3];
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] + yl - 2*PI*stepyl;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] - yl + 2*PI*stepyl;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3];
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] + yr - 2*PI*stepyr;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
//      printBaseFootpose();
      Forward_GetBaselegMsv();
    
      GetLegfootM();  
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(forward_delay);
    }
    Serial.println("EndForward");
}

//后退起步
void StartBackward()
{
  
  ForwardKinematics_GetBasefootM();
  //    kinRmLeg.PrintJointstate();
//    printFootPoint();
//    printBaseFootpose();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] + x/2;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
      //左前腿
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] + x/2;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
      //左后腿
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] + x/2;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(20);
    }
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] - x/2;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
      //左中腿
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] - x/2;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
      //右后腿
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] - x/2;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(20);
    }
    
}

//后退收腿
void EndBackward()
{
  
  ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] - x/2 + stepx*PI;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
      //右前腿
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] - stepx*PI;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] ;
      //左前腿
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] - x/2 + stepx*PI;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
      //左中腿
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] - stepx*PI;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] ;
      //左后腿
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] - x/2 + stepx*PI;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
      //右后腿
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] - stepx*PI;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] ;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(20);
    }
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右前腿
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] + x/2 - stepx*PI;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
      //左中腿
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] + x/2 - stepx*PI;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
      //右后腿
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] + x/2 - stepx*PI;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(20);
    }
}

//这个函数控制机器人后退多少距离
void Robot_backward_y(float ylength)
{
  //首先确定每一步走多少，接下来计算一共走几步，最后一步走多远
  ylength = ylength/1.71;//奇怪的参数，需要调整一下1.6552
  stepx = 5.0;
  int m;
  m = (int)(ylength/(stepx*2*PI));
  //起步
  Serial.println("StartBackward");
  StartBackward();

  //循环走路
  //步子距离2*PI*stepx
  for(int k = 0;k<m;k++)
  {
    Serial.println("Backward");
    ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3];
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3];
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3];
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(backward_delay);
    }
  
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3];
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3];
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3];
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;

//      printBaseFootpose();
      Forward_GetBaselegMsv();
    
      GetLegfootM();  
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(backward_delay);
    }
  }
  //收腿
    EndBackward();

    
  //最后一步
  stepx = (ylength - 2*PI*stepx*m)/(2*PI);
  StartBackward();
  ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3];
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3];
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3];
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(backward_delay);
    }
  
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3];
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3];
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3];
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
    
      GetLegfootM();  
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(backward_delay);
    }
    Serial.println("EndBackward");
    //收腿
    EndBackward();
  
}

//右移起步
void StartRightward()
{
  
  ForwardKinematics_GetBasefootM();
  //    kinRmLeg.PrintJointstate();
//    printFootPoint();
//    printBaseFootpose();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3] - x/2;
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3];
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
      //左前腿
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3] - x/2;
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3];
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
      //左后腿
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3] - x/2;
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3];
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(20);
    }
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] + x/2;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
      //左中腿
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] + x/2;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
      //右后腿
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] + x/2;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(20);
    }
    
}

//右移收腿
void EndRightward()
{
  
  ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3] + x/2 - stepx*PI;
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3];
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
      //右前腿
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] + stepx*PI;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] ;
      //左前腿
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3] + x/2 - stepx*PI;
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3];
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
      //左中腿
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] + stepx*PI;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] ;
      //左后腿
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3] + x/2 - stepx*PI;
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3];
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
      //右后腿
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] + stepx*PI;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] ;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(20);
    }
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右前腿
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] - x/2 + stepx*PI;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
      //左中腿
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] - x/2 + stepx*PI;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
      //右后腿
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] - x/2 + stepx*PI;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(20);
    }
}

//这个函数控制机器人右移多少距离
void Robot_rightward_x(float ylength)
{
  //首先确定每一步走多少，接下来计算一共走几步，最后一步走多远
  ylength = ylength/1.71;//奇怪的参数，需要调整一下1.6552
  stepx = 5.0;
  int m;
  m = (int)(ylength/(stepx*2*PI));
  //起步
  Serial.println("StartForward");
  StartRightward();

  //循环走路
  //步子距离2*PI*stepx
  for(int k = 0;k<m;k++)
  {
    Serial.println("Forward");
    ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3];
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3];
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3];
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3];
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3];
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3];
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(forward_delay);
    }
  
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3];
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3];
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3];
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3];
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3];
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3];
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;

//      printBaseFootpose();
      Forward_GetBaselegMsv();
    
      GetLegfootM();  
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(forward_delay);
    }
  }
  //收腿
    EndRightward();

    
  //最后一步
  stepx = (ylength - 2*PI*stepx*m)/(2*PI);
  StartRightward();
  ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3];
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3];
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3];
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3];
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3];
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3];
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(forward_delay);
    }
  
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3];
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3];
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3];
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3];
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3];
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3];
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
    
      GetLegfootM();  
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(forward_delay);
    }
    Serial.println("EndForward");
    //收腿
    EndRightward();
  
}

//左移起步
void StartLeftward()
{
  
  ForwardKinematics_GetBasefootM();
  //    kinRmLeg.PrintJointstate();
//    printFootPoint();
//    printBaseFootpose();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3] + x/2;
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3];
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
      //左前腿
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3] + x/2;
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3];
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
      //左后腿
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3] + x/2;
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3];
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(20);
    }
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] - x/2;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
      //左中腿
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] - x/2;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
      //右后腿
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] - x/2;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(20);
    }
}

//左移收腿
void EndLeftward()
{
  
  ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3] - x/2 + stepx*PI;
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3];
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
      //右前腿
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] - stepx*PI;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] ;
      //左前腿
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3] - x/2 + stepx*PI;
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3];
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
      //左中腿
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] - stepx*PI;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] ;
      //左后腿
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3] - x/2 + stepx*PI;
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3];
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
      //右后腿
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] - stepx*PI;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] ;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(20);
    }
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右前腿
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] + x/2 - stepx*PI;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
      //左中腿
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] + x/2 - stepx*PI;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
      //右后腿
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] + x/2 - stepx*PI;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(20);
    }
}

//这个函数控制机器人左移多少距离
void Robot_leftward_x(float ylength)
{
  //首先确定每一步走多少，接下来计算一共走几步，最后一步走多远
  ylength = ylength/1.71;//奇怪的参数，需要调整一下1.6552
  stepx = 5.0;
  int m;
  m = (int)(ylength/(stepx*2*PI));
  //起步
  Serial.println("StartBackward");
  StartLeftward();

  //循环走路
  //步子距离2*PI*stepx
  for(int k = 0;k<m;k++)
  {
    Serial.println("Backward");
    ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3];
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3];
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3];
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3];
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3];
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3];
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(backward_delay);
    }
  
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3];
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3];
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3];
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3];
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3];
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3];
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;

//      printBaseFootpose();
      Forward_GetBaselegMsv();
    
      GetLegfootM();  
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(backward_delay);
    }
  }
  //收腿
    EndLeftward();

    
  //最后一步
  stepx = (ylength - 2*PI*stepx*m)/(2*PI);
  StartLeftward();
  ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3];
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3];
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3];
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3];
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3];
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3];
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(backward_delay);
    }
  
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3];
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3];
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3];
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3];
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3];
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3];
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
    
      GetLegfootM();  
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(backward_delay);
    }
    Serial.println("EndBackward");
    //收腿
    EndLeftward();
}

float delayniushenzi_z = 10;
void niushenzi_z()
{
  //扭身子旋转到10度
  for(int i=0;i<=10;i++)
  {
    kinRmLeg.JointState[0] = i/baseanglez;
    kinRfLeg.JointState[0] = i/baseanglez;
    kinLfLeg.JointState[0] = i/baseanglez;
    kinLmLeg.JointState[0] = i/baseanglez;
    kinLbLeg.JointState[0] = i/baseanglez;
    kinRbLeg.JointState[0] = i/baseanglez;

    ForwardKinematics_GetBasefootM();
    Forward_GetBaselegMsv();
    GetLegfootM();
      
    //foot到leg根部的齐次矩阵里面最后一列就是坐标了
    for(int i=0;i<6;i++)
    {
      Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
      Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
      Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
      Endpose_3[i][3][0] = 1.0;
    }
//    printLegFootpose();
    GetLegfootpose_IK2H1mini_Jointstate();
    robot.framewrite(robot.Servo_r);
    delay(delayniushenzi_z);
  }
  //扭身子10到-10
  for(int i=10;i>=-10;i--)
  {
    kinRmLeg.JointState[0] = i/baseanglez;
    kinRfLeg.JointState[0] = i/baseanglez;
    kinLfLeg.JointState[0] = i/baseanglez;
    kinLmLeg.JointState[0] = i/baseanglez;
    kinLbLeg.JointState[0] = i/baseanglez;
    kinRbLeg.JointState[0] = i/baseanglez;

    ForwardKinematics_GetBasefootM();
    Forward_GetBaselegMsv();
    GetLegfootM();
      
    //foot到leg根部的齐次矩阵里面最后一列就是坐标了
    for(int i=0;i<6;i++)
    {
      Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
      Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
      Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
      Endpose_3[i][3][0] = 1.0;
    }
//    printLegFootpose();
    GetLegfootpose_IK2H1mini_Jointstate();
    robot.framewrite(robot.Servo_r);
    delay(delayniushenzi_z);
  }
  //-10到0
  for(int i=-10;i<=0;i++)
  {
    kinRmLeg.JointState[0] = i/baseanglez;
    kinRfLeg.JointState[0] = i/baseanglez;
    kinLfLeg.JointState[0] = i/baseanglez;
    kinLmLeg.JointState[0] = i/baseanglez;
    kinLbLeg.JointState[0] = i/baseanglez;
    kinRbLeg.JointState[0] = i/baseanglez;

    ForwardKinematics_GetBasefootM();
    Forward_GetBaselegMsv();
    GetLegfootM();
      
    //foot到leg根部的齐次矩阵里面最后一列就是坐标了
    for(int i=0;i<6;i++)
    {
      Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
      Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
      Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
      Endpose_3[i][3][0] = 1.0;
    }
//    printLegFootpose();
    GetLegfootpose_IK2H1mini_Jointstate();
    robot.framewrite(robot.Servo_r);
    delay(delayniushenzi_z);
  }
}

//正运动学然后获取Basefoot矩阵
void ForwardKinematics_GetBasefootM()
{
    SetStartJointstate();
    //正运动学
    kinRmLeg.forwardMDH();
    kinRfLeg.forwardMDH();
    kinLfLeg.forwardMDH();
    kinLmLeg.forwardMDH();
    kinLbLeg.forwardMDH();
    kinRbLeg.forwardMDH();
    //得到每条腿foot到base的齐次矩阵
    Matrix.Copy((mtx_type*)kinRmLeg.MatrixOb[5], N, N, (mtx_type*)Base_footM[0]);//B=A;齐次矩阵赋值
    Matrix.Copy((mtx_type*)kinRfLeg.MatrixOb[5], N, N, (mtx_type*)Base_footM[1]);//B=A;齐次矩阵赋值
    Matrix.Copy((mtx_type*)kinLfLeg.MatrixOb[5], N, N, (mtx_type*)Base_footM[2]);//B=A;齐次矩阵赋值
    Matrix.Copy((mtx_type*)kinLmLeg.MatrixOb[5], N, N, (mtx_type*)Base_footM[3]);//B=A;齐次矩阵赋值
    Matrix.Copy((mtx_type*)kinLbLeg.MatrixOb[5], N, N, (mtx_type*)Base_footM[4]);//B=A;齐次矩阵赋值
    Matrix.Copy((mtx_type*)kinRbLeg.MatrixOb[5], N, N, (mtx_type*)Base_footM[5]);//B=A;齐次矩阵赋值

}

float delayniushenzi_x = 10;
void niushenzi_x()
{
  //扭身子旋转到10度
  for(int i=0;i<=10;i++)
  {
    kinRmLeg.Mdh[0][0] = i/baseanglex;
    kinRfLeg.Mdh[0][0] = i/baseanglex;
    kinLfLeg.Mdh[0][0] = i/baseanglex;
    kinLmLeg.Mdh[0][0] = i/baseanglex;
    kinLbLeg.Mdh[0][0] = i/baseanglex;
    kinRbLeg.Mdh[0][0] = i/baseanglex;

    ForwardKinematics_GetBasefootM();
    Forward_GetBaselegMsv();
    GetLegfootM();
      
    //foot到leg根部的齐次矩阵里面最后一列就是坐标了
    for(int i=0;i<6;i++)
    {
      Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
      Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
      Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
      Endpose_3[i][3][0] = 1.0;
    }
//    printLegFootpose();
    GetLegfootpose_IK2H1mini_Jointstate();
    robot.framewrite(robot.Servo_r);
    delay(delayniushenzi_x);
  }
  //扭身子10到-10
  for(int i=10;i>=-10;i--)
  {
    kinRmLeg.Mdh[0][0] = i/baseanglex;
    kinRfLeg.Mdh[0][0] = i/baseanglex;
    kinLfLeg.Mdh[0][0] = i/baseanglex;
    kinLmLeg.Mdh[0][0] = i/baseanglex;
    kinLbLeg.Mdh[0][0] = i/baseanglex;
    kinRbLeg.Mdh[0][0] = i/baseanglex;

    ForwardKinematics_GetBasefootM();
    Forward_GetBaselegMsv();
    GetLegfootM();
    
    //foot到leg根部的齐次矩阵里面最后一列就是坐标了
    for(int i=0;i<6;i++)
    {
      Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
      Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
      Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
      Endpose_3[i][3][0] = 1.0;
    }
//    printLegFootpose();
    GetLegfootpose_IK2H1mini_Jointstate();
    robot.framewrite(robot.Servo_r);
    delay(delayniushenzi_x);
  }
  //-10到0
  for(int i=-10;i<=0;i++)
  {
    kinRmLeg.Mdh[0][0] = i/baseanglex;
    kinRfLeg.Mdh[0][0] = i/baseanglex;
    kinLfLeg.Mdh[0][0] = i/baseanglex;
    kinLmLeg.Mdh[0][0] = i/baseanglex;
    kinLbLeg.Mdh[0][0] = i/baseanglex;
    kinRbLeg.Mdh[0][0] = i/baseanglex;
    
    ForwardKinematics_GetBasefootM();
    Forward_GetBaselegMsv();
    GetLegfootM();
      
    //foot到leg根部的齐次矩阵里面最后一列就是坐标了
    for(int i=0;i<6;i++)
    {
      Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
      Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
      Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
      Endpose_3[i][3][0] = 1.0;
    }
//    printLegFootpose();
    GetLegfootpose_IK2H1mini_Jointstate();
    robot.framewrite(robot.Servo_r);
    delay(delayniushenzi_x);
  }
}

float delayniushenzi_y = 10;
void niushenzi_y()
{
  //扭身子旋转到10度
  for(int i=0;i<=10;i++)
  {
    kinRmLeg.Mdh[1][0] = i/baseangley;
    kinRfLeg.Mdh[1][0] = i/baseangley;
    kinLfLeg.Mdh[1][0] = i/baseangley;
    kinLmLeg.Mdh[1][0] = i/baseangley;
    kinLbLeg.Mdh[1][0] = i/baseangley;
    kinRbLeg.Mdh[1][0] = i/baseangley;
    
    ForwardKinematics_GetBasefootM();
    Forward_GetBaselegMsv();
    GetLegfootM();
    
    //foot到leg根部的齐次矩阵里面最后一列就是坐标了
    for(int i=0;i<6;i++)
    {
      Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
      Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
      Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
      Endpose_3[i][3][0] = 1.0;
    }
//    printLegFootpose();
    GetLegfootpose_IK2H1mini_Jointstate();
    robot.framewrite(robot.Servo_r);
    delay(delayniushenzi_y);
  }
  //扭身子10到-10
  for(int i=10;i>=-10;i--)
  {
    kinRmLeg.Mdh[1][0] = i/baseangley;
    kinRfLeg.Mdh[1][0] = i/baseangley;
    kinLfLeg.Mdh[1][0] = i/baseangley;
    kinLmLeg.Mdh[1][0] = i/baseangley;
    kinLbLeg.Mdh[1][0] = i/baseangley;
    kinRbLeg.Mdh[1][0] = i/baseangley;

    ForwardKinematics_GetBasefootM();
    Forward_GetBaselegMsv();
    GetLegfootM();
    
    //foot到leg根部的齐次矩阵里面最后一列就是坐标了
    for(int i=0;i<6;i++)
    {
      Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
      Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
      Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
      Endpose_3[i][3][0] = 1.0;
    }
//    printLegFootpose();
    GetLegfootpose_IK2H1mini_Jointstate();
    robot.framewrite(robot.Servo_r);
    delay(delayniushenzi_y);
  }
  //-10到0
  for(int i=-10;i<=0;i++)
  {
    kinRmLeg.Mdh[1][0] = i/baseangley;
    kinRfLeg.Mdh[1][0] = i/baseangley;
    kinLfLeg.Mdh[1][0] = i/baseangley;
    kinLmLeg.Mdh[1][0] = i/baseangley;
    kinLbLeg.Mdh[1][0] = i/baseangley;
    kinRbLeg.Mdh[1][0] = i/baseangley;

    ForwardKinematics_GetBasefootM();
    Forward_GetBaselegMsv();
    GetLegfootM();
    
    //foot到leg根部的齐次矩阵里面最后一列就是坐标了
    for(int i=0;i<6;i++)
    {
      Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
      Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
      Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
      Endpose_3[i][3][0] = 1.0;
    }
//    printLegFootpose();
    GetLegfootpose_IK2H1mini_Jointstate();
    robot.framewrite(robot.Servo_r);
    delay(delayniushenzi_y);
  }
}

//根据得到的Endpose_3这个列向量得知Legfoot三维坐标，
//通过ik4算法计算关节变量，然后再赋值给H1mini关节变量的18个成员
void GetLegfootpose_IK2H1mini_Jointstate()
{
    //根据坐标逆解关节变量，OK了。
    //右中腿
    float zuobiao1[3];
    zuobiao1[0] = Endpose_3[0][0][0];
    zuobiao1[1] = Endpose_3[0][1][0];
    zuobiao1[2] = Endpose_3[0][2][0];
    ik4(zuobiao1);
    robot.Servo_r[0] = theta_rh[0]*180.0/PI;
    robot.Servo_r[1] = theta_rh[1]*180.0/PI;
    robot.Servo_r[2] = theta_rh[2]*180.0/PI;
    //右前腿
    zuobiao1[0] = Endpose_3[1][0][0];
    zuobiao1[1] = Endpose_3[1][1][0];
    zuobiao1[2] = Endpose_3[1][2][0];
    ik4(zuobiao1);
    robot.Servo_r[3] = theta_rh[0]*180.0/PI;
    robot.Servo_r[4] = theta_rh[1]*180.0/PI;
    robot.Servo_r[5] = theta_rh[2]*180.0/PI;
  
    //左前腿
    zuobiao1[0] = Endpose_3[2][0][0];
    zuobiao1[1] = Endpose_3[2][1][0];
    zuobiao1[2] = Endpose_3[2][2][0];
    ik4(zuobiao1);
    robot.Servo_r[6] = theta_rh[0]*180.0/PI;
    robot.Servo_r[7] = theta_rh[1]*180.0/PI;
    robot.Servo_r[8] = theta_rh[2]*180.0/PI;
  
    //左中腿
    zuobiao1[0] = Endpose_3[3][0][0];
    zuobiao1[1] = Endpose_3[3][1][0];
    zuobiao1[2] = Endpose_3[3][2][0];
    ik4(zuobiao1);
    robot.Servo_r[9] = theta_rh[0]*180.0/PI;
    robot.Servo_r[10] = theta_rh[1]*180.0/PI;
    robot.Servo_r[11] = theta_rh[2]*180.0/PI;
  
    //左后腿
    zuobiao1[0] = Endpose_3[4][0][0];
    zuobiao1[1] = Endpose_3[4][1][0];
    zuobiao1[2] = Endpose_3[4][2][0];
    ik4(zuobiao1);
    robot.Servo_r[12] = theta_rh[0]*180.0/PI;
    robot.Servo_r[13] = theta_rh[1]*180.0/PI;
    robot.Servo_r[14] = theta_rh[2]*180.0/PI;
  
    //左后腿
    zuobiao1[0] = Endpose_3[5][0][0];
    zuobiao1[1] = Endpose_3[5][1][0];
    zuobiao1[2] = Endpose_3[5][2][0];
    ik4(zuobiao1);
    robot.Servo_r[15] = theta_rh[0]*180.0/PI;
    robot.Servo_r[16] = theta_rh[1]*180.0/PI;
    robot.Servo_r[17] = theta_rh[2]*180.0/PI;
//    for(int i=0;i<18;i++)
//    {
//      Serial.print("S");
//      Serial.print(i);
//      Serial.print(":");
//      Serial.print(robot.Servo_r[i]);
//      Serial.print("  ");
//    }
//    Serial.println();
}

//正向运动学并得到BaselegM初始矩阵和其逆矩阵BaselegMatrixStartInvert
//Baseleg表示leg到base的矩阵，Start表示alpha和theta都是0.0，Inver表示逆矩阵
void Forward_GetBaselegMsv()
{
      kinRmLeg.Mdh[0][0] = 0.0;
      kinRfLeg.Mdh[0][0] = 0.0;
      kinLfLeg.Mdh[0][0] = 0.0;
      kinLmLeg.Mdh[0][0] = 0.0;
      kinLbLeg.Mdh[0][0] = 0.0;
      kinRbLeg.Mdh[0][0] = 0.0;
      
      kinRmLeg.Mdh[1][0] = 0.0;
      kinRfLeg.Mdh[1][0] = 0.0;
      kinLfLeg.Mdh[1][0] = 0.0;
      kinLmLeg.Mdh[1][0] = 0.0;
      kinLbLeg.Mdh[1][0] = 0.0;
      kinRbLeg.Mdh[1][0] = 0.0;

      kinRmLeg.JointState[0] = 0.0;
      kinRfLeg.JointState[0] = 0.0;
      kinLfLeg.JointState[0] = 0.0;
      kinLmLeg.JointState[0] = 0.0;
      kinLbLeg.JointState[0] = 0.0;
      kinRbLeg.JointState[0] = 0.0;
    
      //正运动学
      kinRmLeg.forwardMDH();
      kinRfLeg.forwardMDH();
      kinLfLeg.forwardMDH();
      kinLmLeg.forwardMDH();
      kinLbLeg.forwardMDH();
      kinRbLeg.forwardMDH();
  
      //首先找到leg根部到baselink矩阵，赋值给Base_legM
      Matrix.Copy((mtx_type*)kinRmLeg.MatrixOb[2], N, N, (mtx_type*)Base_legM[0]);//B=A;齐次矩阵赋值
      Matrix.Copy((mtx_type*)kinRfLeg.MatrixOb[2], N, N, (mtx_type*)Base_legM[1]);//B=A;齐次矩阵赋值
      Matrix.Copy((mtx_type*)kinLfLeg.MatrixOb[2], N, N, (mtx_type*)Base_legM[2]);//B=A;齐次矩阵赋值
      Matrix.Copy((mtx_type*)kinLmLeg.MatrixOb[2], N, N, (mtx_type*)Base_legM[3]);//B=A;齐次矩阵赋值
      Matrix.Copy((mtx_type*)kinLbLeg.MatrixOb[2], N, N, (mtx_type*)Base_legM[4]);//B=A;齐次矩阵赋值
      Matrix.Copy((mtx_type*)kinRbLeg.MatrixOb[2], N, N, (mtx_type*)Base_legM[5]);//B=A;齐次矩阵赋值

      //然后求leg根部到baselink逆矩阵
      Matrix.Invert((mtx_type*)Base_legM[0], N);
      Matrix.Invert((mtx_type*)Base_legM[1], N);
      Matrix.Invert((mtx_type*)Base_legM[2], N);
      Matrix.Invert((mtx_type*)Base_legM[3], N);
      Matrix.Invert((mtx_type*)Base_legM[4], N);
      Matrix.Invert((mtx_type*)Base_legM[5], N);
}

void GetLegfootM()
{
  //然后逆矩阵右边乘以MatrixOb,得到的就是foot到leg根部的矩阵
  //C=A*B;
  //Multiply(mtx_type* A, mtx_type* B, int m, int p, int n, mtx_type* C)
  //Leg_footM = Base_legM-1 *(Base_legM * Leg_FootM) = Base_legM-1 * Base_FootM
  Matrix.Multiply((mtx_type*)Base_legM[0], (mtx_type*)Base_footM[0], N, N, N, (mtx_type*)Leg_footM[0]);
  Matrix.Multiply((mtx_type*)Base_legM[1], (mtx_type*)Base_footM[1], N, N, N, (mtx_type*)Leg_footM[1]);
  Matrix.Multiply((mtx_type*)Base_legM[2], (mtx_type*)Base_footM[2], N, N, N, (mtx_type*)Leg_footM[2]);
  Matrix.Multiply((mtx_type*)Base_legM[3], (mtx_type*)Base_footM[3], N, N, N, (mtx_type*)Leg_footM[3]);
  Matrix.Multiply((mtx_type*)Base_legM[4], (mtx_type*)Base_footM[4], N, N, N, (mtx_type*)Leg_footM[4]);
  Matrix.Multiply((mtx_type*)Base_legM[5], (mtx_type*)Base_footM[5], N, N, N, (mtx_type*)Leg_footM[5]);
}

float delayniushenzi_xy = 2;
void niushenzi_xy()
{
  //扭身子绕着x轴前后倾斜旋转到10度，前低后高，起始步骤1
  for(int i=0;i<=10;i++)
  {
    kinRmLeg.Mdh[0][0] = i/baseanglex;
    kinRfLeg.Mdh[0][0] = i/baseanglex;
    kinLfLeg.Mdh[0][0] = i/baseanglex;
    kinLmLeg.Mdh[0][0] = i/baseanglex;
    kinLbLeg.Mdh[0][0] = i/baseanglex;
    kinRbLeg.Mdh[0][0] = i/baseanglex;

    ForwardKinematics_GetBasefootM();
    Forward_GetBaselegMsv();
    GetLegfootM();

    //foot到leg根部的齐次矩阵里面最后一列就是坐标了
    for(int i=0;i<6;i++)
    {
      Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
      Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
      Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
      Endpose_3[i][3][0] = 1.0;
    }
//    printLegFootpose();
    GetLegfootpose_IK2H1mini_Jointstate();
    robot.framewrite(robot.Servo_r);
    delay(delayniushenzi_xy);
  }
  for(int m = 0;m<=10;m++)
  {
    //同时绕着x轴和y轴,x轴从10到0,y轴从0到10，y轴从0到-10，就是第二个关节坐标系，步骤2
    for(int i=0;i<=10;i++)
    {
      kinRmLeg.Mdh[0][0] = (10-i)/baseanglex;
      kinRfLeg.Mdh[0][0] = (10-i)/baseanglex;
      kinLfLeg.Mdh[0][0] = (10-i)/baseanglex;
      kinLmLeg.Mdh[0][0] = (10-i)/baseanglex;
      kinLbLeg.Mdh[0][0] = (10-i)/baseanglex;
      kinRbLeg.Mdh[0][0] = (10-i)/baseanglex;
      
      kinRmLeg.Mdh[1][0] = i/baseangley;
      kinRfLeg.Mdh[1][0] = i/baseangley;
      kinLfLeg.Mdh[1][0] = i/baseangley;
      kinLmLeg.Mdh[1][0] = i/baseangley;
      kinLbLeg.Mdh[1][0] = i/baseangley;
      kinRbLeg.Mdh[1][0] = i/baseangley;

      ForwardKinematics_GetBasefootM();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(delayniushenzi_xy);
    }
    //步骤3
    for(int i=0;i<=10;i++)
    {
      kinRmLeg.Mdh[0][0] = (0-i)/baseanglex;
      kinRfLeg.Mdh[0][0] = (0-i)/baseanglex;
      kinLfLeg.Mdh[0][0] = (0-i)/baseanglex;
      kinLmLeg.Mdh[0][0] = (0-i)/baseanglex;
      kinLbLeg.Mdh[0][0] = (0-i)/baseanglex;
      kinRbLeg.Mdh[0][0] = (0-i)/baseanglex;
      
      kinRmLeg.Mdh[1][0] = (10-i)/baseangley;
      kinRfLeg.Mdh[1][0] = (10-i)/baseangley;
      kinLfLeg.Mdh[1][0] = (10-i)/baseangley;
      kinLmLeg.Mdh[1][0] = (10-i)/baseangley;
      kinLbLeg.Mdh[1][0] = (10-i)/baseangley;
      kinRbLeg.Mdh[1][0] = (10-i)/baseangley;

      ForwardKinematics_GetBasefootM();
      Forward_GetBaselegMsv();
      GetLegfootM();

      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(delayniushenzi_xy);
    }
    //步骤4
    for(int i=0;i<=10;i++)
    {
      kinRmLeg.Mdh[0][0] = (i-10)/baseanglex;
      kinRfLeg.Mdh[0][0] = (i-10)/baseanglex;
      kinLfLeg.Mdh[0][0] = (i-10)/baseanglex;
      kinLmLeg.Mdh[0][0] = (i-10)/baseanglex;
      kinLbLeg.Mdh[0][0] = (i-10)/baseanglex;
      kinRbLeg.Mdh[0][0] = (i-10)/baseanglex;
      
      kinRmLeg.Mdh[1][0] = (0-i)/baseangley;
      kinRfLeg.Mdh[1][0] = (0-i)/baseangley;
      kinLfLeg.Mdh[1][0] = (0-i)/baseangley;
      kinLmLeg.Mdh[1][0] = (0-i)/baseangley;
      kinLbLeg.Mdh[1][0] = (0-i)/baseangley;
      kinRbLeg.Mdh[1][0] = (0-i)/baseangley;

      ForwardKinematics_GetBasefootM();
      Forward_GetBaselegMsv();
      GetLegfootM();
          
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(delayniushenzi_xy);
    }
    //步骤5
    for(int i=0;i<=10;i++)
    {
      kinRmLeg.Mdh[0][0] = i/baseanglex;
      kinRfLeg.Mdh[0][0] = i/baseanglex;
      kinLfLeg.Mdh[0][0] = i/baseanglex;
      kinLmLeg.Mdh[0][0] = i/baseanglex;
      kinLbLeg.Mdh[0][0] = i/baseanglex;
      kinRbLeg.Mdh[0][0] = i/baseanglex;
      
      kinRmLeg.Mdh[1][0] = (i-10)/baseangley;
      kinRfLeg.Mdh[1][0] = (i-10)/baseangley;
      kinLfLeg.Mdh[1][0] = (i-10)/baseangley;
      kinLmLeg.Mdh[1][0] = (i-10)/baseangley;
      kinLbLeg.Mdh[1][0] = (i-10)/baseangley;
      kinRbLeg.Mdh[1][0] = (i-10)/baseangley;

      ForwardKinematics_GetBasefootM();
      Forward_GetBaselegMsv();
      GetLegfootM();
          
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(delayniushenzi_xy);
    }
  }
  //步骤6
  for(int i=0;i<=10;i++)
  {
    kinRmLeg.Mdh[0][0] = (10-i)/baseanglex;
    kinRfLeg.Mdh[0][0] = (10-i)/baseanglex;
    kinLfLeg.Mdh[0][0] = (10-i)/baseanglex;
    kinLmLeg.Mdh[0][0] = (10-i)/baseanglex;
    kinLbLeg.Mdh[0][0] = (10-i)/baseanglex;
    kinRbLeg.Mdh[0][0] = (10-i)/baseanglex;
    
    kinRmLeg.Mdh[1][0] = 0.0;
    kinRfLeg.Mdh[1][0] = 0.0;
    kinLfLeg.Mdh[1][0] = 0.0;
    kinLmLeg.Mdh[1][0] = 0.0;
    kinLbLeg.Mdh[1][0] = 0.0;
    kinRbLeg.Mdh[1][0] = 0.0;

    ForwardKinematics_GetBasefootM();
    Forward_GetBaselegMsv();
    GetLegfootM();
      
    //foot到leg根部的齐次矩阵里面最后一列就是坐标了
    for(int i=0;i<6;i++)
    {
      Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
      Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
      Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
      Endpose_3[i][3][0] = 1.0;
    }
//    printLegFootpose();
    GetLegfootpose_IK2H1mini_Jointstate();
    robot.framewrite(robot.Servo_r);
    delay(delayniushenzi_xy);
  }
}

void start_turnleft()
{
    //准备姿态,0到10度。扭身子10到-10
    for(int i=0;i<=10;i++)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10到-10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(leftward_delay);
    }
  
}

void end_turnleft()
{
    //准备姿态,10到0度
    for(int i=10;i>=0;i--)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10到-10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(leftward_delay);
    }
    robot.framewrite(joint_ss1);
    delay(leftward_delay);
}

float zhuanwanjiaodu = 0.6981;//30度转弧度：0.5236；40度转弧度：0.6981
//左转多少度
void turnleft_angle(float angleleft)
{
   angleleft = angleleft*1.2456;//不知名比例，要消去1.2456
  //计算循环旋转多少次
  //旋转起步
  angleleft = ((angleleft/180.0)*PI);
//  baseanglez = 400.0/angleleft;//每次40，循环10次，所以这里是400
  //这里计算循环多少次，每次旋转至少30度，弧度为zhuanwanjiaodu弧度
  int left_time = (int)(angleleft/(zhuanwanjiaodu));
  baseanglez = 40.0/zhuanwanjiaodu;
  start_turnleft();
  delay(500);
  //原地左转
  for(int m = 0;m<left_time;m++)
  {
    //扭身子10到-10
    for(int i=10;i>=-10;i--)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10到-10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(leftward_delay);
    }
    //扭身子-10到10
    for(int i=-10;i<=10;i++)
    {
      kinRmLeg.JointState[0] = i/baseanglez;
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3] + 5.0 + z;
      Base_footM[1][2][3] = Base_footM[1][2][3];
      Base_footM[2][2][3] = Base_footM[2][2][3] + 5.0 + z;
      Base_footM[3][2][3] = Base_footM[3][2][3];
      Base_footM[4][2][3] = Base_footM[4][2][3] + 5.0 + z;
      Base_footM[5][2][3] = Base_footM[5][2][3];
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(leftward_delay);
    }
  }
  delay(500);
  end_turnleft();
  delay(500);
  //最后一步旋转角度:
  //旋转收腿
  angleleft = angleleft - zhuanwanjiaodu * left_time;
  baseanglez = 40.0/angleleft;
  start_turnleft();
  delay(500);
  //扭身子10到-10
    for(int i=10;i>=-10;i--)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10到-10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(leftward_delay);
    }
    //扭身子10到-10
    for(int i=-10;i<=10;i++)
    {
      kinRmLeg.JointState[0] = i/baseanglez;
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3] + 5.0 + z;
      Base_footM[1][2][3] = Base_footM[1][2][3];
      Base_footM[2][2][3] = Base_footM[2][2][3] + 5.0 + z;
      Base_footM[3][2][3] = Base_footM[3][2][3];
      Base_footM[4][2][3] = Base_footM[4][2][3] + 5.0 + z;
      Base_footM[5][2][3] = Base_footM[5][2][3];
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(leftward_delay);
    }
    end_turnleft();
    delay(500);
}


//一次性左转多少度,一步完成,通常左转角度较小时应用,对称步态，需要起步和止步
//去掉多余的起步和止步，因为只是左转一步，为什么起步两次呢？当left_time为0时，不应该起步
void turnleft_angle1(float angleleft)
{
   angleleft = angleleft*1.2456;//不知名比例，要消去1.2456
  //计算循环旋转多少次
  //旋转起步
  angleleft = ((angleleft/180.0)*PI);
  //一步旋转角度，就是我们要转的角度
  baseanglez = 40.0/angleleft;
  start_turnleft();
  //扭身子10到-10
    for(int i=10;i>=-10;i--)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10到-10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(leftward_delay);
    }
    //扭身子10到-10
    for(int i=-10;i<=10;i++)
    {
      kinRmLeg.JointState[0] = i/baseanglez;
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3] + 5.0 + z;
      Base_footM[1][2][3] = Base_footM[1][2][3];
      Base_footM[2][2][3] = Base_footM[2][2][3] + 5.0 + z;
      Base_footM[3][2][3] = Base_footM[3][2][3];
      Base_footM[4][2][3] = Base_footM[4][2][3] + 5.0 + z;
      Base_footM[5][2][3] = Base_footM[5][2][3];
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(leftward_delay);
    }
    end_turnleft();
}

//一次性左转多少度,一步完成,通常左转角度较小时应用,非对称步态，不需要起步和止步
//去掉多余的起步和止步，因为只是左转一步，为什么起步两次呢？当left_time为0时，不应该起步
void turnleft_angle2(float angleleft)
{
   angleleft = angleleft*1.2456;//不知名比例，要消去1.2456
  //计算循环旋转多少次
  //旋转起步
  angleleft = ((angleleft/180.0)*PI);
  //一步旋转角度，就是我们要转的角度
  baseanglez = 40.0/angleleft;
  //扭身子10到-10
    for(int i=0;i>=-20;i--)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(0到-10)
      kinRfLeg.JointState[0] = -i/baseanglez;//(0到10)
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(-i)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;

//      printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(leftward_delay);
    }
    //扭身子10到-10
    for(int i=-20;i<=0;i++)
    {
      kinRmLeg.JointState[0] = i/baseanglez;
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
      float t = 2.0*PI*(i+20.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3] + 5.0 + z;
      Base_footM[1][2][3] = Base_footM[1][2][3];
      Base_footM[2][2][3] = Base_footM[2][2][3] + 5.0 + z;
      Base_footM[3][2][3] = Base_footM[3][2][3];
      Base_footM[4][2][3] = Base_footM[4][2][3] + 5.0 + z;
      Base_footM[5][2][3] = Base_footM[5][2][3];

//      printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(leftward_delay);
    }
}

void start_turnright()
{
    //准备姿态,10到0度。扭身子10到-10
    for(int i=0;i>=-10;i--)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10到-10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(leftward_delay);
    }
  
}

void end_turnright()
{
    //准备姿态,10到0度
    for(int i=-10;i<=0;i++)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10到-10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(leftward_delay);
    }
    robot.framewrite(joint_ss1);
    delay(leftward_delay);
}

void turnright_angle(float angleright)
{
  angleright = angleright*1.2456;//不知名比例，要消去
  //计算循环旋转多少次
  //旋转起步
  angleright = ((angleright/180.0)*PI);
//  baseanglez = 400.0/angleright;//每次40，循环10次，所以这里是400
  //这里计算循环多少次，每次旋转至少30度，弧度为zhuanwanjiaodu弧度
  int right_time = (int)(angleright/(zhuanwanjiaodu));
  baseanglez = 40.0/zhuanwanjiaodu;
  start_turnright();
  delay(500);
  //原地右转
  for(int m = 0;m<right_time;m++)
  {
    //扭身子-10到10
    for(int i=-10;i<=10;i++)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10到-10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(rightward_delay);
    }
    //扭身子10到-10
    for(int i=10;i>=-10;i--)
    {
      kinRmLeg.JointState[0] = i/baseanglez;
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3] + 5.0 + z;
      Base_footM[1][2][3] = Base_footM[1][2][3];
      Base_footM[2][2][3] = Base_footM[2][2][3] + 5.0 + z;
      Base_footM[3][2][3] = Base_footM[3][2][3];
      Base_footM[4][2][3] = Base_footM[4][2][3] + 5.0 + z;
      Base_footM[5][2][3] = Base_footM[5][2][3];
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(rightward_delay);
    }
  }
  delay(500);
  end_turnright();
  delay(500);
  
  //最后一步旋转角度:
  //旋转收腿
  angleright = angleright - zhuanwanjiaodu * right_time;
  baseanglez = 40.0/angleright;
  start_turnright();
  delay(500);
  //最后一步
  //扭身子-10到10
    for(int i=-10;i<=10;i++)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10到-10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(rightward_delay);
    }
    //扭身子10到-10
    for(int i=10;i>=-10;i--)
    {
      kinRmLeg.JointState[0] = i/baseanglez;
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3] + 5.0 + z;
      Base_footM[1][2][3] = Base_footM[1][2][3];
      Base_footM[2][2][3] = Base_footM[2][2][3] + 5.0 + z;
      Base_footM[3][2][3] = Base_footM[3][2][3];
      Base_footM[4][2][3] = Base_footM[4][2][3] + 5.0 + z;
      Base_footM[5][2][3] = Base_footM[5][2][3];
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(rightward_delay);
    }
    delay(500);
    end_turnright();
    delay(500);
}

//一次性左转多少度,一步完成,通常左转角度较小时应用
//去掉多余的起步和止步，因为只是左转一步，为什么起步两次呢？当right_time为0时，不应该起步
void turnright_angle1(float angleright)
{
  angleright = angleright*1.2456;//不知名比例，要消去
  //计算循环旋转多少次
  //旋转起步
  angleright = ((angleright/180.0)*PI);
  //最后一步旋转角度:
  baseanglez = 40.0/angleright;
  start_turnright();
  //最后一步
  //扭身子-10到10
    for(int i=-10;i<=10;i++)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10到-10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(rightward_delay);
    }
    //扭身子10到-10
    for(int i=10;i>=-10;i--)
    {
      kinRmLeg.JointState[0] = i/baseanglez;
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3] + 5.0 + z;
      Base_footM[1][2][3] = Base_footM[1][2][3];
      Base_footM[2][2][3] = Base_footM[2][2][3] + 5.0 + z;
      Base_footM[3][2][3] = Base_footM[3][2][3];
      Base_footM[4][2][3] = Base_footM[4][2][3] + 5.0 + z;
      Base_footM[5][2][3] = Base_footM[5][2][3];
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(rightward_delay);
    }
    end_turnright();
}

//一次性右转多少度,一步完成,通常右转角度较小时应用,非对称步态，不需要起步和止步
//去掉多余的起步和止步，因为只是右转一步，为什么起步两次呢？当right_time为0时，不应该起步
void turnright_angle2(float angleright)
{
  angleright = angleright*1.2456;//不知名比例，要消去
  //计算循环旋转多少次
  //旋转起步
  angleright = ((angleright/180.0)*PI);
  //最后一步旋转角度:
  baseanglez = 40.0/angleright;
  //最后一步
  //扭身子-10到10
    for(int i=0;i<=20;i++)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10到-10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(rightward_delay);
    }
    //扭身子10到-10
    for(int i=20;i>=0;i--)
    {
      kinRmLeg.JointState[0] = i/baseanglez;
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
      float t = 2.0*PI*(20-i)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3] + 5.0 + z;
      Base_footM[1][2][3] = Base_footM[1][2][3];
      Base_footM[2][2][3] = Base_footM[2][2][3] + 5.0 + z;
      Base_footM[3][2][3] = Base_footM[3][2][3];
      Base_footM[4][2][3] = Base_footM[4][2][3] + 5.0 + z;
      Base_footM[5][2][3] = Base_footM[5][2][3];
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      robot.framewrite(robot.Servo_r);
      delay(rightward_delay);
    }
}

void SetStartJointstate()
{
  kinRmLeg.JointState[3] = PI/6;
  kinRmLeg.JointState[4] = -PI/6;
  
  kinRfLeg.JointState[3] = PI/6;
  kinRfLeg.JointState[4] = -PI/6;
  
  kinLfLeg.JointState[3] = PI/6;
  kinLfLeg.JointState[4] = -PI/6;
  
  kinLmLeg.JointState[3] = PI/6;
  kinLmLeg.JointState[4] = -PI/6;
  
  kinLbLeg.JointState[3] = PI/6;
  kinLbLeg.JointState[4] = -PI/6;
  
  kinRbLeg.JointState[3] = PI/6;
  kinRbLeg.JointState[4] = -PI/6;
}

//单纯通过寻迹传感器盲走找到迹，然后寻这个迹
void find_track_track()
{
  //先写第一种情况，最简单的，机器人前进时刚好两个寻迹传感器都碰到黑线，黑线还是直线
  //这个时候前进一段距离，然后左转90°或者右转90°就刚好夹住黑线，就可以寻迹了。
  //碰到黑线灯熄灭，输出高电平，正常灯亮，输出低电平。
  while((!digitalRead(leftfpin))&&(!digitalRead(rightfpin)))
  Robot_forward_y2(30.0);
  delay(1000);
  Robot_forward_y(85.0);
  turnright_angle(90.0);
}

void setup() {
  // put your setup code here, to run once:
  pinMode(leftfpin,INPUT);
  pinMode(rightfpin,INPUT);
  pinMode(magnet,OUTPUT);
  digitalWrite(magnet,LOW);
  Serial.begin(115200);
  Serial.setTimeout(1000);  // 设置串口超时时间，单位为毫秒
  // 初始化软串口
  espSerial.begin(115200);
//  espSerial.end();
  // 初始化结构体数组
  for (int i = 0; i < 10; i++) {
    myball[i].executionFlag = false;  // 将所有元素的 executionFlag 初始化为 false
  }

  kinRmLeg.AddJointMDH( 0.0 , 0.0,  PI/2, 0.0 );
  kinRmLeg.AddJointMDH( 0.0 , 0.0, -PI/2, 0.0 );
  kinRmLeg.AddJointMDH( 0.0 , lm ,  0.0 , 0.0 );
  kinRmLeg.AddJointMDH( PI/2, l1 ,  0.0 , 0.0 );
  kinRmLeg.AddJointMDH( 0.0 , l2 , -PI/2, 0.0 );
  kinRmLeg.AddJointMDH( 0.0 , l3 ,  0.0 , 0.0 );

  kinRfLeg.AddJointMDH( 0.0 , 0.0,  PI/2, 0.0 );
  kinRfLeg.AddJointMDH( 0.0 , 0.0, -0.4437, 0.0 );
  kinRfLeg.AddJointMDH( 0.0 , ll , -0.3416, 0.0 );
  kinRfLeg.AddJointMDH( PI/2, l1 ,  0.0 , 0.0 );
  kinRfLeg.AddJointMDH( 0.0 , l2 , -PI/2, 0.0 );
  kinRfLeg.AddJointMDH( 0.0 , l3 ,  0.0 , 0.0 );

  kinLfLeg.AddJointMDH( 0.0 , 0.0,  PI/2, 0.0 );
  kinLfLeg.AddJointMDH( 0.0 , 0.0, 0.4437 , 0.0 );
  kinLfLeg.AddJointMDH( 0.0 , ll , 0.3416 , 0.0 );
  kinLfLeg.AddJointMDH( PI/2, l1 ,  0.0 , 0.0 );
  kinLfLeg.AddJointMDH( 0.0 , l2 , -PI/2, 0.0 );
  kinLfLeg.AddJointMDH( 0.0 , l3 ,  0.0 , 0.0 );

  kinLmLeg.AddJointMDH( 0.0 , 0.0,  PI/2, 0.0 );
  kinLmLeg.AddJointMDH( 0.0 , 0.0,  PI/2, 0.0 );
  kinLmLeg.AddJointMDH( 0.0 , lm ,  0.0 , 0.0 );
  kinLmLeg.AddJointMDH( PI/2, l1 ,  0.0 , 0.0 );
  kinLmLeg.AddJointMDH( 0.0 , l2 , -PI/2, 0.0 );
  kinLmLeg.AddJointMDH( 0.0 , l3 ,  0.0 , 0.0 );

  kinLbLeg.AddJointMDH( 0.0 , 0.0,  PI/2, 0.0 );
  kinLbLeg.AddJointMDH( 0.0 , 0.0, 2.6978 , 0.0 );
  kinLbLeg.AddJointMDH( 0.0 , ll ,-0.3416 , 0.0 );
  kinLbLeg.AddJointMDH( PI/2, l1 ,  0.0 , 0.0 );
  kinLbLeg.AddJointMDH( 0.0 , l2 , -PI/2, 0.0 );
  kinLbLeg.AddJointMDH( 0.0 , l3 ,  0.0 , 0.0 );

  kinRbLeg.AddJointMDH( 0.0 , 0.0,  PI/2, 0.0 );
  kinRbLeg.AddJointMDH( 0.0 , 0.0, 3.5853 , 0.0 );
  kinRbLeg.AddJointMDH( 0.0 , ll , 0.3416 , 0.0 );
  kinRbLeg.AddJointMDH( PI/2, l1 ,  0.0 , 0.0 );
  kinRbLeg.AddJointMDH( 0.0 , l2 , -PI/2, 0.0 );
  kinRbLeg.AddJointMDH( 0.0 , l3 ,  0.0 , 0.0 );
  

  robot.Jointservo[0] = 3;robot.Jointservo[1] = 4;robot.Jointservo[2] = 5;
  robot.Jointservo[3] = 13;robot.Jointservo[4] = 14;robot.Jointservo[5] = 15;
  robot.Jointservo[6] = 29;robot.Jointservo[7] = 30;robot.Jointservo[8] = 31;
  robot.Jointservo[9] = 19;robot.Jointservo[10] = 20;robot.Jointservo[11] = 21;
  robot.Jointservo[12] = 16;robot.Jointservo[13] = 17;robot.Jointservo[14] = 18;
  robot.Jointservo[15] = 0;robot.Jointservo[16] = 1;robot.Jointservo[17] = 2;

  robot.direct[0] = 1;
  robot.direct[1] = -1;
  robot.direct[2] = 1;
  robot.direct[3] = 1;
  robot.direct[4] = -1;
  robot.direct[5] = 1;
  robot.direct[6] = 1;
  robot.direct[7] = -1;
  robot.direct[8] = 1;
  robot.direct[9] = 1;
  robot.direct[10] = -1;
  robot.direct[11] = 1;
  robot.direct[12] = 1;
  robot.direct[13] = -1;
  robot.direct[14] = 1;
  robot.direct[15] = 1;
  robot.direct[16] = -1;
  robot.direct[17] = 1;

  //初始化机械臂
  arm.init();
  arm.speed=50;
  arm.jointwrite(0,0);
  arm.jointwrite(1,-55);
  arm.jointwrite(2,15);
  arm.jointwrite(3,0);
//  robot.framewrite(Servo_p);
  if (Serial.available() > 0)
  String receivedString = Serial.readString();  // 读取所有字符串
  delay(1000);

  arm.jointwrite(3,0);
  delay(1000);
//  arm.jointwrite(3,90);
//  delay(1000);
//  arm.jointwrite(3,0);
//  delay(1000);
  theta_r[0] = 0.0;
  theta_r[1] = 0.0;
  theta_r[2] = 0.0;
  theta_r[3] = 0.0;
  calculateMDHMatrix(theta_r);
  TFforward();
  digitalWrite(magnet,LOW);
  delay(3000);
  
  robot.init();
  robot.framewrite(robot.Servo0);
  
  robot.framewrite(joint_ss);
  delay(1000);
  robot.frame2frame(joint_ss,joint_ss1);
  delay(3000);
  robot.framewrite(joint_ss1);
  delay(1000);
  Serial.println();
  
  SetStartJointstate();

  theta_p[0] = 0.0;
  theta_p[1] = -55.0;
  theta_p[2] = 70.0;
  theta_p[3] = 0.0;
  theta2servo();
  theta_r[0] = 0.0;
  theta_r[1] = -55.0;
  theta_r[2] = 70.0;
  theta_r[3] = 40.0;
  go();
//  delay(100000);
}

void loop() {
  // put your main code here, to run repeatedly:
//  Serial.println("hello");
    //机械臂标定
//  if (Serial.available() > 0) {
//    cmd = Serial.read();
//    delay(1);
//    if(cmd=='x')//校准舵机角度
//    {
//      while(Serial.available()>0)
//      {
//        int joint_id=Serial.parseInt();
//        arm.rec[joint_id]=Serial.read();
//        //转换成signed char(范围-128~127),esp8266默认char是unsigned char，范围0~255
//        arm.rec[joint_id]=Serial.parseInt();
//        Serial.println(joint_id);
//        Serial.println(arm.rec[joint_id]);
//        EEPROM.write(arm_eeprom+joint_id,arm.rec[joint_id]);
//        EEPROM.commit();
//      }
//      print_jointjz();
//      arm.framewrite(arm.Servo0);
//      delay(100);
//    }
//  }


//  //六足机器人标定
//  while(Serial.available()>0)
//  {
//    cmd = Serial.read();
//    delay(1);
//    if(cmd=='x')//校准舵机角度
//    {
//      while(Serial.available()>0)
//      {
//        int joint_id=Serial.parseInt();
//        robot.rec[joint_id]=Serial.read();
//        //转换成signed char(范围-128~127),esp8266默认char是unsigned char，范围0~255
//        robot.rec[joint_id]=Serial.parseInt();
//        Serial.println(joint_id);
//        Serial.println(robot.rec[joint_id]);
//        EEPROM.write(0+joint_id,robot.rec[joint_id]);
//        EEPROM.commit();
//      }
//      robot.printcaliData();
//      robot.framewrite(Servo0);
//      delay(100);
//    }    
//  }


////  robot.stepheight = 1;
////  robot.forward(5);
//  Robot_forward_y(500.0);
//  delay(3000);
//  
////  robot.turnleft(5);
//  turnleft_angle(90.0);
//  delay(1000);
////  robot.forward(1);
//  Robot_forward_y(100.0);
//  
//  theta_p[0] = 0.0;
//  theta_p[1] = -55.0;
//  theta_p[2] = 70.0;
//  theta_p[3] = 0.0;
//  theta2servo();
//  theta_r[0] = 0.0;
//  theta_r[1] = 0.0;
//  theta_r[2] = 0.0;
//  theta_r[3] = 90.0;
//  go();
//  delay(5000);
//  getData();
//  //到这里，串口发送过来的所有的目标点的像素坐标和颜色都有了
//  //接下来判断是否有目标物，如果有目标物，则过去抓取
//  if(isGoal())
//  {
//    getGoal();
//    eraseDataInArray();
////    robot.turnright(9);
//    turnright_angle(180.0);
//    delay(1000);
////    robot.forward(2);
//    Robot_forward_y(200.0);
////    delay(3000000);
//    theta_r[0] = 0.0;
//    theta_r[1] = 0.0;
//    theta_r[2] = 0.0;
//    theta_r[3] = 90.0;
//    go();
//    delay(5000);
//    getData();
//    if(isGoal())
//    {
//      putGoal();
//    }
//  }
//  
//  delay(3000000);

  
//  SerchRedgoal();
//  trackRedGoal();
//  delay(1000000);
//  Robot_forward_y(300.0);
//  delay(1000);
//  Robot_rightward_x(300.0);
//  delay(1000);
//  while(true)
//  {
//    getData();
//    if(isRedGoal())
//    {
//      for (int i = 0; i < 10; ++i) 
//      {
//        if ((myball[i].executionFlag == 1)&&(myball[i].targetColor == 2)) 
//        {
//          if(myball[i].xCoordinate > 165)
//          {
//            theta_r[3] += 1;
//            arm.jointwrite(3,theta_r[3]);
//            theta_p[0]=theta_r[0];                 //记录当前Servo_p
//            theta_p[1]=theta_r[1];
//            theta_p[2]=theta_r[2];
//            theta_p[3]=theta_r[3];
//            
//            Servo_p[0]=Servo_r[0];                 //记录当前Servo_p
//            Servo_p[1]=Servo_r[1];
//            Servo_p[2]=Servo_r[2];
////            go();
//          }
//          else if(myball[i].xCoordinate < 155)
//          {
//            theta_r[3] -= 1;
//            arm.jointwrite(3,theta_r[3]);
//            theta_p[0]=theta_r[0];                 //记录当前Servo_p
//            theta_p[1]=theta_r[1];
//            theta_p[2]=theta_r[2];
//            theta_p[3]=theta_r[3];
//            
//            Servo_p[0]=Servo_r[0];                 //记录当前Servo_p
//            Servo_p[1]=Servo_r[1];
//            Servo_p[2]=Servo_r[2];
////            go();
//          }
//          if(myball[i].yCoordinate > 125)
//          {
//            theta_r[0] += 1;
//            arm.jointwrite(0,theta_r[0]);
//            theta_p[0]=theta_r[0];                 //记录当前Servo_p
//            theta_p[1]=theta_r[1];
//            theta_p[2]=theta_r[2];
//            theta_p[3]=theta_r[3];
//            
//            Servo_p[0]=Servo_r[0];                 //记录当前Servo_p
//            Servo_p[1]=Servo_r[1];
//            Servo_p[2]=Servo_r[2];
////            go();
//          }
//          else if(myball[i].yCoordinate < 115)
//          {
//            theta_r[0] -= 1;
//            arm.jointwrite(0,theta_r[0]);
//            theta_p[0]=theta_r[0];                 //记录当前Servo_p
//            theta_p[1]=theta_r[1];
//            theta_p[2]=theta_r[2];
//            theta_p[3]=theta_r[3];
//            
//            Servo_p[0]=Servo_r[0];                 //记录当前Servo_p
//            Servo_p[1]=Servo_r[1];
//            Servo_p[2]=Servo_r[2];
////            go();
//          }
//          eraseDataInArray();
//        }
//      }
//      delay(100);
//    }
//  }
//  getTheta_r();
//  delay(1000);
//  theta_r[0] = 0.0;
//  theta_r[1] = -55.0;
//  theta_r[2] = 70.0;
//  theta_r[3] = 0.0;
//  go();

//  Robot_forward_y(300.0);
//  delay(1000);
//  
//  theta_p[0] = 0.0;
//  theta_p[1] = -55.0;
//  theta_p[2] = 70.0;
//  theta_p[3] = 0.0;
//  theta2servo();
//  theta_r[0] = 0.0;
//  theta_r[1] = 0.0;
//  theta_r[2] = 0.0;
//  theta_r[3] = 90.0;
//  go();
//  delay(1000);
//  getData();
//  //到这里，串口发送过来的所有的目标点的像素坐标和颜色都有了
//  //接下来判断是否有目标物，如果有目标物，则过去抓取
//  if(isGoal())
//  {
//    getGoal();
//    eraseDataInArray();
////    robot.turnright(9);
//    delay(1000);
////    robot.forward(2);
////    Robot_backward_y(50.0);
//    Robot_rightward_x(300.0);
////    Robot_forward_y(50.0);
////    delay(3000000);
//    theta_r[0] = 0.0;
//    theta_r[1] = 0.0;
//    theta_r[2] = 0.0;
//    theta_r[3] = 90.0;
//    go();
//    delay(1000);
//    getData();
//    if(isGoal())
//    {
//      putGoal();
//    }
//  }
//  Serial.print("left:");
//  Serial.println(digitalRead(leftfpin));
//  Serial.print("right:");
//  Serial.println(digitalRead(rightfpin));
  //碰到黑线灯熄灭，输出高电平，正常灯亮，输出低电平。
//  if((digitalRead(leftfpin))&&(!digitalRead(rightfpin)))
//    delay(1000);//turnleft_angle(20.0);
//  else if((digitalRead(rightfpin))&&(!digitalRead(leftfpin)))
//    delay(1000);//turnright_angle(20.0);


//  find_track_track();
//
  while(1)
  {
    if(digitalRead(leftfpin)||digitalRead(rightfpin))
    {
  //    delay(1000);//turnleft_angle(20.0);
      if(digitalRead(leftfpin))
      turnleft_angle2(20.0);
      else
      turnright_angle2(20.0);
    }
    else
    Robot_forward_y2(30.0);    
  }

//  Robot_forward_y1(40.0);
//  Robot_forward_y2(40.0);
//  Robot_forward_y3(40.0);
//  delay(200);

//  turnright_angle2(20.0);
//  delay(1000);
//  turnleft_angle2(20.0);
//  delay(1000);
  
//  robot.stepheight = 2;
//  robot.forward(3);
//  delay(3000);
}
