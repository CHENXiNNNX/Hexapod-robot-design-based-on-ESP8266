#include <Robot.h>  // 机器人控制库
#include "Kinematics.h" // 运动学计算库
#include <MatrixMath.h> // 矩阵运算库
#define N 4 // 齐次矩阵维度4x4

/* 数学常量定义 */
#define pi4 0.78539816325  //pi/4弧度值，用于45度计算

/* 机械结构参数定义 */
#define l1 34         // 大腿连杆长度（单位：mm
#define l2 43.51      // 小腿第一段长度
#define l3 93.07      // 小腿第二段长度
#define ll 94.93715   // 机身斜边半长（用于侧边腿安装位置计算）
#define lm 50.8       // 机身中间横梁半长（用于中间腿安装位置）
#define jointnum 5    // 每条腿的关节数量

/* 步态参数 */
float stepx = 5;  // X方向步长
float stepz = 10; // Z方向抬腿高度

/* 关节角度存储数组 
   [6条腿][每条腿3个关节] */
// 关节变量
float theta[6][3] = { // 初始化全为0
  0.0 ,0.0, 0.0,
  0.0 ,0.0, 0.0,
  0.0 ,0.0, 0.0,
  0.0 ,0.0, 0.0,
  0.0 ,0.0, 0.0,
  0.0 ,0.0, 0.0
};

/* 末端执行器位置存储 
   [6条腿][x,y,z,1][齐次坐标] */
// Endpose_3,这个点是foot相对于leg根部的坐标
float Endpose_3[6][4][1]=
{
  0.0 ,0.0 ,0.0 , 1.0,
  0.0 ,0.0 ,0.0 , 1.0,
  0.0 ,0.0 ,0.0 , 1.0,
  0.0 ,0.0 ,0.0 , 1.0,
  0.0 ,0.0 ,0.0 , 1.0,
  0.0 ,0.0 ,0.0 , 1.0
};

float theta_r[3]; // 临时存储单条腿逆解结果

/* 坐标变换矩阵 */
// leg根部到foot的逆矩阵
mtx_type Base_footM[6][4][4]; // 基坐标系到脚底的变换矩阵
mtx_type Base_legM[6][4][4];  // 基坐标系到腿根部的变换矩阵
mtx_type Leg_footM[6][4][4];  // 腿根部到脚底的变换矩阵

// 六条腿的运动学模型(MDH参数法)
Kinematics kinRmLeg(jointnum);  // 右中腿
Kinematics kinRfLeg(jointnum);  // 右前腿
Kinematics kinLfLeg(jointnum);  // 左前腿
Kinematics kinLmLeg(jointnum);  // 左中腿
Kinematics kinLbLeg(jointnum);  // 左后腿
Kinematics kinRbLeg(jointnum);  // 右后腿

RobotClass robot; // 机器人控制实例化

/* 函数：打印所有腿的末端位置 */
void printRobotFootPoint()
{
  // 执行正运动学计算
  kinRmLeg.forwardMDH();
  kinRfLeg.forwardMDH();
  kinLfLeg.forwardMDH();
  kinLmLeg.forwardMDH();
  kinLbLeg.forwardMDH();
  kinRbLeg.forwardMDH();
  
  // 打印各腿末端位置
  kinRmLeg.PrintEnd_effectorPoint();
  kinRfLeg.PrintEnd_effectorPoint();
  kinLfLeg.PrintEnd_effectorPoint();
  kinLmLeg.PrintEnd_effectorPoint();
  kinLbLeg.PrintEnd_effectorPoint();
  kinRbLeg.PrintEnd_effectorPoint();
}

/* printBaseFootpose函数：打印基座到脚的坐标 */
void printBaseFootpose()
{
  char buffer[50]; // 根据需要调整大小
  for(int i=0;i<6;i++)
  {
    Serial.print("Leg Base_footPoint:");
    Serial.print("|");
    snprintf(buffer, sizeof(buffer), " x: %-9.2f y: %-9.2f z: %-9.2f ", Base_footM[i][0][3], Base_footM[i][1][3], Base_footM[i][2][3]);
    Serial.print(buffer);
    Serial.println(); 
  }
}

/* printLegFootpose函数：打印腿根部到脚的坐标 */
void printLegFootpose()
{
  char buffer[50]; // 根据需要调整大小
  for(int i=0;i<6;i++)
  {
    Serial.print("Leg End_effectorPoint:");
    Serial.print("|");
    snprintf(buffer, sizeof(buffer), " x: %-9.2f y: %-9.2f z: %-9.2f ", Endpose_3[i][0][0], Endpose_3[i][1][0], Endpose_3[i][2][0]);
    Serial.print(buffer);
    Serial.println(); 
  }
}

/* CalEndpose_3函数：计算脚相对于腿根部的坐标 */
// 求Endpose_3,这个点是foot相对于leg根部的坐标
void CalEndpose_3()
{
  /* 获取各腿根部到基座的变换矩阵*/
  // 首先找到leg根部到baselink矩阵，赋值给Base_legM
  Matrix.Copy((mtx_type*)kinRmLeg.MatrixOb[1], N, N, (mtx_type*)Base_legM[0]);//B=A;齐次矩阵赋值
  Matrix.Copy((mtx_type*)kinRfLeg.MatrixOb[1], N, N, (mtx_type*)Base_legM[1]);//B=A;齐次矩阵赋值
  Matrix.Copy((mtx_type*)kinLfLeg.MatrixOb[1], N, N, (mtx_type*)Base_legM[2]);//B=A;齐次矩阵赋值
  Matrix.Copy((mtx_type*)kinLmLeg.MatrixOb[1], N, N, (mtx_type*)Base_legM[3]);//B=A;齐次矩阵赋值
  Matrix.Copy((mtx_type*)kinLbLeg.MatrixOb[1], N, N, (mtx_type*)Base_legM[4]);//B=A;齐次矩阵赋值
  Matrix.Copy((mtx_type*)kinRbLeg.MatrixOb[1], N, N, (mtx_type*)Base_legM[5]);//B=A;齐次矩阵赋值
  
  /* 计算各腿根部坐标系的逆矩阵*/
  // 然后求leg根部到baselink逆矩阵
  Matrix.Invert((mtx_type*)Base_legM[0], N);
  Matrix.Invert((mtx_type*)Base_legM[1], N);
  Matrix.Invert((mtx_type*)Base_legM[2], N);
  Matrix.Invert((mtx_type*)Base_legM[3], N);
  Matrix.Invert((mtx_type*)Base_legM[4], N);
  Matrix.Invert((mtx_type*)Base_legM[5], N);

  /* 计算腿根部到脚的变换矩阵：Leg_footM = inv(Base_legM) * Base_footM
     然后逆矩阵右边乘以MatrixOb,得到的就是foot到leg根部的矩阵
     C=A*B;
     Multiply(mtx_type* A, mtx_type* B, int m, int p, int n, mtx_type* C)
     Leg_footM = Base_legM-1 *(Base_legM * Leg_FootM) = Base_legM-1 * Base_FootM */
  Matrix.Multiply((mtx_type*)Base_legM[0], (mtx_type*)kinRmLeg.MatrixOb[4], N, N, N, (mtx_type*)Leg_footM[0]);
  Matrix.Multiply((mtx_type*)Base_legM[1], (mtx_type*)kinRfLeg.MatrixOb[4], N, N, N, (mtx_type*)Leg_footM[1]);
  Matrix.Multiply((mtx_type*)Base_legM[2], (mtx_type*)kinLfLeg.MatrixOb[4], N, N, N, (mtx_type*)Leg_footM[2]);
  Matrix.Multiply((mtx_type*)Base_legM[3], (mtx_type*)kinLmLeg.MatrixOb[4], N, N, N, (mtx_type*)Leg_footM[3]);
  Matrix.Multiply((mtx_type*)Base_legM[4], (mtx_type*)kinLbLeg.MatrixOb[4], N, N, N, (mtx_type*)Leg_footM[4]);
  Matrix.Multiply((mtx_type*)Base_legM[5], (mtx_type*)kinRbLeg.MatrixOb[4], N, N, N, (mtx_type*)Leg_footM[5]);

  /* 提取坐标值到Endpose_3数组 */
  // foot到leg根部的齐次矩阵里面最后一列就是坐标了
  for(int i=0;i<6;i++)
  {
    Endpose_3[i][0][0] = Leg_footM[i][0][3];// x
    Endpose_3[i][1][0] = Leg_footM[i][1][3];// y
    Endpose_3[i][2][0] = Leg_footM[i][2][3];// z
    Endpose_3[i][3][0] = 1.0;               // 齐次坐标
  }
  //printLegFootpose();
  // 根据坐标对每条腿进行逆运动学计算关节变量
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

/* 逆运动学计算函数（几何法）
   参数：zuobiao[3] - 目标位置的x,y,z坐标 */
//机械臂逆运动学，传入参数为zuobiao[3]，目标的xyz三个坐标，
//解算结果存储在theta_r[0]，theta_r[1]，theta_r[2]这三个变量中,根据Oa4坐标逆解
void ik4(float zuobiao[3]){
  float x3,y3,z3;
  float x1,y1,z1,P1P3_2,P1P3,P321;
  float P210,P213,P310,P3P0_2,P3P0;//P3P0_2是P3P0的平方P310是角P310
  x3 = zuobiao[0];
  y3 = zuobiao[1];
  z3 = zuobiao[2];

  // 计算θ1（绕Z轴旋转）
  theta_r[0] = atan(y3/x3);
  
  // 计算θ3（肘关节角度）
  x1 = l1 * cos(theta_r[0]);
  y1 = l1 * sin(theta_r[0]);
  z1 = 0.0;
  P1P3_2 = (x3-x1)*(x3-x1)+(y3-y1)*(y3-y1)+(z3-z1)*(z3-z1);
  P1P3 = sqrt(P1P3_2);
  P321=acos((l2*l2+l3*l3-P1P3_2)/(2*l2*l3));
  theta_r[2] = P321-PI/2;
  
  // 最后计算θ2（肩关节角度）
  P213=asin((l3/P1P3)*sin(P321));
  P3P0_2=x3*x3+y3*y3+z3*z3;
  P3P0 = sqrt(P3P0_2);
  P310 = acos((l1*l1+P1P3_2-P3P0_2)/(2*l1*P1P3));
  theta_r[1] = P213 + P310 - PI;
  
  /*
  Serial.print("theta_r1:");
  Serial.print(theta_r[0]);
  Serial.print("   ");
  Serial.print("theta_r2:");
  Serial.print(theta_r[1]);
  Serial.print("   ");
  Serial.print("theta_r3:");
  Serial.println(theta_r[2]);  
  */

}

/* 机器人前向运动学控制函数 */
void RobotClass_forward_k()
{
   // 执行正运动学计算
  kinRmLeg.forwardMDH();
  kinRfLeg.forwardMDH();
  kinLfLeg.forwardMDH();
  kinLmLeg.forwardMDH();
  kinLbLeg.forwardMDH();
  kinRbLeg.forwardMDH();

  //得到每条腿foot到base的齐次矩阵
  Matrix.Copy((mtx_type*)kinRmLeg.MatrixOb[4], N, N, (mtx_type*)Base_footM[0]);//B=A;齐次矩阵赋值
  Matrix.Copy((mtx_type*)kinRfLeg.MatrixOb[4], N, N, (mtx_type*)Base_footM[1]);//B=A;齐次矩阵赋值
  Matrix.Copy((mtx_type*)kinLfLeg.MatrixOb[4], N, N, (mtx_type*)Base_footM[2]);//B=A;齐次矩阵赋值
  Matrix.Copy((mtx_type*)kinLmLeg.MatrixOb[4], N, N, (mtx_type*)Base_footM[3]);//B=A;齐次矩阵赋值
  Matrix.Copy((mtx_type*)kinLbLeg.MatrixOb[4], N, N, (mtx_type*)Base_footM[4]);//B=A;齐次矩阵赋值
  Matrix.Copy((mtx_type*)kinRbLeg.MatrixOb[4], N, N, (mtx_type*)Base_footM[5]);//B=A;齐次矩阵赋值

  // 生成摆线轨迹
  float t = 2.0*PI*3.0/20.0;
  float x=stepx*(t-sin(t)); // X方向摆线运动
  float z=stepz*(1-cos(t)); // Z方向抛物线运动
  
  // 坐标变换计算，更新各腿末端位置
  Base_footM[0][0][0] += x;
  Base_footM[0][1][0] += 0.0;
  Base_footM[0][2][0] += z;
  Serial.print("x:");
  Serial.println(Base_footM[0][0][0]);

  Base_footM[1][0][0] += x;
  Base_footM[1][1][0] += 0.0;
  Base_footM[1][2][0] += z;

  Base_footM[2][0][0] += x;
  Base_footM[2][1][0] += 0.0;
  Base_footM[2][2][0] += z;

  Base_footM[3][0][0] += x;
  Base_footM[3][1][0] += 0.0;
  Base_footM[3][2][0] += z;

  Base_footM[4][0][0] += x;
  Base_footM[4][1][0] += 0.0;
  Base_footM[4][2][0] += z;

  Base_footM[5][0][0] += x;
  Base_footM[5][1][0] += 0.0;
  Base_footM[5][2][0] += z;

  /* 获取各腿根部到基座的变换矩阵 */
  // 首先找到leg根部到baselink矩阵，赋值给Base_legM
  Matrix.Copy((mtx_type*)kinRmLeg.MatrixOb[1], N, N, (mtx_type*)Base_legM[0]);//B=A;齐次矩阵赋值
  Matrix.Copy((mtx_type*)kinRfLeg.MatrixOb[1], N, N, (mtx_type*)Base_legM[1]);//B=A;齐次矩阵赋值
  Matrix.Copy((mtx_type*)kinLfLeg.MatrixOb[1], N, N, (mtx_type*)Base_legM[2]);//B=A;齐次矩阵赋值
  Matrix.Copy((mtx_type*)kinLmLeg.MatrixOb[1], N, N, (mtx_type*)Base_legM[3]);//B=A;齐次矩阵赋值
  Matrix.Copy((mtx_type*)kinLbLeg.MatrixOb[1], N, N, (mtx_type*)Base_legM[4]);//B=A;齐次矩阵赋值
  Matrix.Copy((mtx_type*)kinRbLeg.MatrixOb[1], N, N, (mtx_type*)Base_legM[5]);//B=A;齐次矩阵赋值
  
  /* 计算各腿根部坐标系的逆矩阵 */
  //然后求leg根部到baselink逆矩阵
  Matrix.Invert((mtx_type*)Base_legM[0], N);
  Matrix.Invert((mtx_type*)Base_legM[1], N);
  Matrix.Invert((mtx_type*)Base_legM[2], N);
  Matrix.Invert((mtx_type*)Base_legM[3], N);
  Matrix.Invert((mtx_type*)Base_legM[4], N);
  Matrix.Invert((mtx_type*)Base_legM[5], N);

  /* 计算腿根部到脚的变换矩阵：Leg_footM = inv(Base_legM) * Base_footM
     然后逆矩阵右边乘以MatrixOb,得到的就是foot到leg根部的矩阵
     C=A*B;
     Multiply(mtx_type* A, mtx_type* B, int m, int p, int n, mtx_type* C)
     Leg_footM = Base_legM-1 *(Base_legM * Leg_FootM) = Base_legM-1 * Base_FootM */
  Matrix.Multiply((mtx_type*)Base_legM[0], (mtx_type*)Base_footM[0], N, N, N, (mtx_type*)Leg_footM[0]);
  Matrix.Multiply((mtx_type*)Base_legM[1], (mtx_type*)Base_footM[1], N, N, N, (mtx_type*)Leg_footM[1]);
  Matrix.Multiply((mtx_type*)Base_legM[2], (mtx_type*)Base_footM[2], N, N, N, (mtx_type*)Leg_footM[2]);
  Matrix.Multiply((mtx_type*)Base_legM[3], (mtx_type*)Base_footM[3], N, N, N, (mtx_type*)Leg_footM[3]);
  Matrix.Multiply((mtx_type*)Base_legM[4], (mtx_type*)Base_footM[4], N, N, N, (mtx_type*)Leg_footM[4]);
  Matrix.Multiply((mtx_type*)Base_legM[5], (mtx_type*)Base_footM[5], N, N, N, (mtx_type*)Leg_footM[5]);

  //foot到leg根部的齐次矩阵里面最后一列就是坐标了
  for(int i=0;i<6;i++)
  {
    Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
    Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
    Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
    Endpose_3[i][3][0] = 1.0;
  }
  // 根据坐标对每条腿进行逆运动学计算关节变量
  // 右中腿舵机角度设置
  float zuobiao1[3];
  zuobiao1[0] = Endpose_3[0][0][0];
  zuobiao1[1] = Endpose_3[0][1][0];
  zuobiao1[2] = Endpose_3[0][2][0];
  ik4(zuobiao1);
  robot.Servo_r[0] = theta_r[0]*180.0/PI;
  robot.Servo_r[1] = theta_r[1]*180.0/PI;
  robot.Servo_r[2] = theta_r[2]*180.0/PI;
  // 右前腿舵机角度设置
  zuobiao1[0] = Endpose_3[1][0][0];
  zuobiao1[1] = Endpose_3[1][1][0];
  zuobiao1[2] = Endpose_3[1][2][0];
  ik4(zuobiao1);
  robot.Servo_r[3] = theta_r[0]*180.0/PI;
  robot.Servo_r[4] = theta_r[1]*180.0/PI;
  robot.Servo_r[5] = theta_r[2]*180.0/PI;

  // 左前腿舵机角度设置
  zuobiao1[0] = Endpose_3[2][0][0];
  zuobiao1[1] = Endpose_3[2][1][0];
  zuobiao1[2] = Endpose_3[2][2][0];
  ik4(zuobiao1);
  robot.Servo_r[6] = theta_r[0]*180.0/PI;
  robot.Servo_r[7] = theta_r[1]*180.0/PI;
  robot.Servo_r[8] = theta_r[2]*180.0/PI;

  // 左中腿舵机角度设置
  zuobiao1[0] = Endpose_3[3][0][0];
  zuobiao1[1] = Endpose_3[3][1][0];
  zuobiao1[2] = Endpose_3[3][2][0];
  ik4(zuobiao1);
  robot.Servo_r[9] = theta_r[0]*180.0/PI;
  robot.Servo_r[10] = theta_r[1]*180.0/PI;
  robot.Servo_r[11] = theta_r[2]*180.0/PI;

  // 左后腿舵机角度设置
  zuobiao1[0] = Endpose_3[4][0][0];
  zuobiao1[1] = Endpose_3[4][1][0];
  zuobiao1[2] = Endpose_3[4][2][0];
  ik4(zuobiao1);
  robot.Servo_r[12] = theta_r[0]*180.0/PI;
  robot.Servo_r[13] = theta_r[1]*180.0/PI;
  robot.Servo_r[14] = theta_r[2]*180.0/PI;

  // 左后腿舵机角度设置
  zuobiao1[0] = Endpose_3[5][0][0];
  zuobiao1[1] = Endpose_3[5][1][0];
  zuobiao1[2] = Endpose_3[5][2][0];
  ik4(zuobiao1);
  robot.Servo_r[15] = theta_r[0]*180.0/PI;
  robot.Servo_r[16] = theta_r[1]*180.0/PI;
  robot.Servo_r[17] = theta_r[2]*180.0/PI;
  
}



void setup() {
  Serial.begin(115200);

  // 配置右中腿MDH参数
  kinRmLeg.AddJointMDH( 0.0 , 0.0, -PI/2, 0.0 );  // 关节1：a=0（连杆长度），d=0（连杆偏移），α=-90°（连杆扭转角），θ=0（关节初始角度）
  kinRmLeg.AddJointMDH( 0.0 , lm ,  0.0 , 0.0 );  // 关节2：沿Y轴平移lm（机身中间横线一半长度）
  kinRmLeg.AddJointMDH( PI/2, l1 ,  0.0 , 0.0 );  // 关节3：a=l1（大腿连杆长度），d=0，α=90°，θ=0
  kinRmLeg.AddJointMDH( 0.0 , l2 , -PI/2, 0.0 );  // 关节4：a=l2（小腿第一段长度），d=0，α=-90°，θ=0
  kinRmLeg.AddJointMDH( 0.0 , l3 ,  0.0 , 0.0 );  // 关节5：a=l3（小腿第二段长度），d=0，α=0°，θ=0

  // 配置右前腿的MDH参数
  kinRfLeg.AddJointMDH(0.0, 0.0, -0.4437, 0.0);  // 关节1：α=-0.4437弧度（约-25.4°）
  kinRfLeg.AddJointMDH(0.0, ll, -0.3416, 0.0);   // 关节2：沿Y轴平移ll（机身斜边半长），α=-0.3416弧度（约-19.6°）
  kinRfLeg.AddJointMDH(PI/2, l1, 0.0, 0.0);      // 关节3：a=l1，α=90°
  kinRfLeg.AddJointMDH(0.0, l2, -PI/2, 0.0);     // 关节4：a=l2，α=-90°
  kinRfLeg.AddJointMDH(0.0, l3, 0.0, 0.0);       // 关节5：a=l3，α=0°

  // 配置左前腿的MDH参数
  kinLfLeg.AddJointMDH(0.0, 0.0, 0.4437, 0.0);   // 关节1：α=0.4437弧度（约25.4°）
  kinLfLeg.AddJointMDH(0.0, ll, 0.3416, 0.0);    // 关节2：沿Y轴平移ll，α=0.3416弧度（约19.6°）
  kinLfLeg.AddJointMDH(PI/2, l1, 0.0, 0.0);      // 关节3：a=l1，α=90°
  kinLfLeg.AddJointMDH(0.0, l2, -PI/2, 0.0);     // 关节4：a=l2，α=-90°
  kinLfLeg.AddJointMDH(0.0, l3, 0.0, 0.0);       // 关节5：a=l3，α=0°

  // 配置左中腿的MDH参数
  kinLmLeg.AddJointMDH(0.0, 0.0, PI/2, 0.0);     // 关节1：α=90°
  kinLmLeg.AddJointMDH(0.0, lm, 0.0, 0.0);       // 关节2：沿Y轴平移lm
  kinLmLeg.AddJointMDH(PI/2, l1, 0.0, 0.0);      // 关节3：a=l1，α=90°
  kinLmLeg.AddJointMDH(0.0, l2, -PI/2, 0.0);     // 关节4：a=l2，α=-90°
  kinLmLeg.AddJointMDH(0.0, l3, 0.0, 0.0);       // 关节5：a=l3，α=0°

  // 配置左后腿的MDH参数
  kinLbLeg.AddJointMDH(0.0, 0.0, 2.6978, 0.0);   // 关节1：α=2.6978弧度（约154.6°）
  kinLbLeg.AddJointMDH(0.0, ll, -0.3416, 0.0);   // 关节2：沿Y轴平移ll，α=-0.3416弧度（约-19.6°）
  kinLbLeg.AddJointMDH(PI/2, l1, 0.0, 0.0);      // 关节3：a=l1，α=90°
  kinLbLeg.AddJointMDH(0.0, l2, -PI/2, 0.0);     // 关节4：a=l2，α=-90°
  kinLbLeg.AddJointMDH(0.0, l3, 0.0, 0.0);       // 关节5：a=l3，α=0°

  // 配置右后腿的MDH参数
  kinRbLeg.AddJointMDH(0.0, 0.0, 3.5853, 0.0);   // 关节1：α=3.5853弧度（约205.4°）
  kinRbLeg.AddJointMDH(0.0, ll, 0.3416, 0.0);    // 关节2：沿Y轴平移ll，α=0.3416弧度（约19.6°）
  kinRbLeg.AddJointMDH(PI/2, l1, 0.0, 0.0);      // 关节3：a=l1，α=90°
  kinRbLeg.AddJointMDH(0.0, l2, -PI/2, 0.0);     // 关节4：a=l2，α=-90°
  kinRbLeg.AddJointMDH(0.0, l3, 0.0, 0.0);       // 关节5：a=l3，α=0°

  // 配置右侧关节的舵机引脚
  robot.Jointservo[0] = 6;  // 右中腿关节1对应引脚6
  robot.Jointservo[1] = 7;  // 右中腿关节2对应引脚7
  robot.Jointservo[2] = 8;  // 右中腿关节3对应引脚8
  robot.Jointservo[3] = 9;  // 右前腿关节1对应引脚9
  robot.Jointservo[4] = 10; // 右前腿关节2对应引脚10
  robot.Jointservo[5] = 11; // 右前腿关节3对应引脚11
  robot.Jointservo[15] = 3; // 右后腿关节1对应引脚3
  robot.Jointservo[16] = 4; // 右后腿关节2对应引脚4
  robot.Jointservo[17] = 5; // 右后腿关节3对应引脚5

  // 配置左侧关节的舵机引脚
  robot.Jointservo[6] = 25;  // 左前腿关节1对应引脚25
  robot.Jointservo[7] = 26;  // 左前腿关节2对应引脚26
  robot.Jointservo[8] = 27;  // 左前腿关节3对应引脚27
  robot.Jointservo[9] = 22;  // 左中腿关节1对应引脚22
  robot.Jointservo[10] = 23; // 左中腿关节2对应引脚23
  robot.Jointservo[11] = 24; // 左中腿关节3对应引脚24
  robot.Jointservo[12] = 19; // 左后腿关节1对应引脚19
  robot.Jointservo[13] = 20; // 左后腿关节2对应引脚20
  robot.Jointservo[14] = 21; // 左后腿关节3对应引脚21

  // 配置舵机转向（1为正方向，-1为反向）
  robot.direct[0] = 1;   // 右中腿关节1正转
  robot.direct[1] = -1;  // 右中腿关节2反转
  robot.direct[2] = 1;   // 右中腿关节3正转
  robot.direct[3] = 1;   // 右前腿关节1正转
  robot.direct[4] = -1;  // 右前腿关节2反转
  robot.direct[5] = 1;   // 右前腿关节3正转
  robot.direct[6] = 1;   // 左前腿关节1正转
  robot.direct[7] = -1;  // 左前腿关节2反转
  robot.direct[8] = 1;   // 左前腿关节3正转
  robot.direct[9] = 1;   // 左中腿关节1正转
  robot.direct[10] = -1; // 左中腿关节2反转
  robot.direct[11] = 1;  // 左中腿关节3正转
  robot.direct[12] = 1;  // 左后腿关节1正转
  robot.direct[13] = -1; // 左后腿关节2反转
  robot.direct[14] = 1;  // 左后腿关节3正转
  robot.direct[15] = 1;  // 右后腿关节1正转
  robot.direct[16] = -1; // 右后腿关节2反转
  robot.direct[17] = 1;  // 右后腿关节3正转

  // 初始化机器人舵机控制
  robot.init();
  // 写入初始舵机角度（robot.Servo0为初始角度数组）
  robot.framewrite(robot.Servo0);
  // 等待1秒，确保舵机到位
  delay(1000);
  // 打印空行
  Serial.println();
  // 打印当前时间（毫秒）
  Serial.println(millis());

  // kinRbLeg.forwardMDH();  // 计算右后腿的正运动学
  // Serial.println(millis());  // 打印时间
  // kinRbLeg.PrintMatrixO();  // 打印变换矩阵
  // kinRbLeg.PrintMatrixOb(); // 打印基坐标系变换矩阵
  // kinRbLeg.PrintEnd_effectorPoint(); // 打印末端执行器位置
  // printRobotFootPoint();  // 打印所有腿的末端位置
  // CalEndpose_3();  // 计算脚相对于腿根部的坐标
  // Serial.println(millis());  // 打印时间

  // 打印当前时间
  Serial.println(millis());

  // RobotClass_forward_k();  // 执行机器人前向运动学控制

  // 打印当前时间
  Serial.println(millis());
}
void loop() {
  // 正运动学计算：计算每条腿的末端执行器位置
  kinRmLeg.forwardMDH();  // 右中腿正运动学
  kinRfLeg.forwardMDH();  // 右前腿正运动学
  kinLfLeg.forwardMDH();  // 左前腿正运动学
  kinLmLeg.forwardMDH();  // 左中腿正运动学
  kinLbLeg.forwardMDH();  // 左后腿正运动学
  kinRbLeg.forwardMDH();  // 右后腿正运动学

  // 将每条腿的末端执行器位置（齐次矩阵）复制到Base_footM数组中
  Matrix.Copy((mtx_type*)kinRmLeg.MatrixOb[4], N, N, (mtx_type*)Base_footM[0]); // 右中腿
  Matrix.Copy((mtx_type*)kinRfLeg.MatrixOb[4], N, N, (mtx_type*)Base_footM[1]); // 右前腿
  Matrix.Copy((mtx_type*)kinLfLeg.MatrixOb[4], N, N, (mtx_type*)Base_footM[2]); // 左前腿
  Matrix.Copy((mtx_type*)kinLmLeg.MatrixOb[4], N, N, (mtx_type*)Base_footM[3]); // 左中腿
  Matrix.Copy((mtx_type*)kinLbLeg.MatrixOb[4], N, N, (mtx_type*)Base_footM[4]); // 左后腿
  Matrix.Copy((mtx_type*)kinRbLeg.MatrixOb[4], N, N, (mtx_type*)Base_footM[5]); // 右后腿

  // 循环20次，生成步态轨迹
  for (int i = 1; i <= 20; i++) {
    // 计算摆线轨迹参数
    float t = 2.0 * PI * i / 20.0; // 相位参数（0~2π）
    float x = stepx * (t - sin(t)); // X方向摆线运动
    float z = stepz * (1 - cos(t)); // Z方向抛物线运动

    // 打印调试信息
    Serial.println("hello5");

    // 更新右中腿末端位置
    Base_footM[0][0][3] = kinRmLeg.MatrixOb[4][0][3] + x; // X坐标
    Base_footM[0][1][3] = kinRmLeg.MatrixOb[4][1][3];     // Y坐标不变
    Base_footM[0][2][3] = kinRmLeg.MatrixOb[4][2][3] + z; // Z坐标

    // 更新右前腿末端位置
    Base_footM[1][0][3] = kinRfLeg.MatrixOb[4][0][3] + x;
    Base_footM[1][1][3] = kinRfLeg.MatrixOb[4][1][3];
    Base_footM[1][2][3] = kinRfLeg.MatrixOb[4][2][3] + z;

    // 更新左前腿末端位置
    Base_footM[2][0][3] = kinLfLeg.MatrixOb[4][0][3] + x;
    Base_footM[2][1][3] = kinLfLeg.MatrixOb[4][1][3];
    Base_footM[2][2][3] = kinLfLeg.MatrixOb[4][2][3] + z;

    // 更新左中腿末端位置
    Base_footM[3][0][3] = kinLmLeg.MatrixOb[4][0][3] + x;
    Base_footM[3][1][3] = kinLmLeg.MatrixOb[4][1][3];
    Base_footM[3][2][3] = kinLmLeg.MatrixOb[4][2][3] + z;

    // 更新左后腿末端位置
    Base_footM[4][0][3] = kinLbLeg.MatrixOb[4][0][3] + x;
    Base_footM[4][1][3] = kinLbLeg.MatrixOb[4][1][3];
    Base_footM[4][2][3] = kinLbLeg.MatrixOb[4][2][3] + z;

    // 更新右后腿末端位置
    Base_footM[5][0][3] = kinRbLeg.MatrixOb[4][0][3] + x;
    Base_footM[5][1][3] = kinRbLeg.MatrixOb[4][1][3];
    Base_footM[5][2][3] = kinRbLeg.MatrixOb[4][2][3] + z;

    // 打印调试信息
    Serial.println("hello6");

    // 将每条腿的根部到基座的变换矩阵复制到Base_legM数组中
    Matrix.Copy((mtx_type*)kinRmLeg.MatrixOb[1], N, N, (mtx_type*)Base_legM[0]); // 右中腿
    Matrix.Copy((mtx_type*)kinRfLeg.MatrixOb[1], N, N, (mtx_type*)Base_legM[1]); // 右前腿
    Matrix.Copy((mtx_type*)kinLfLeg.MatrixOb[1], N, N, (mtx_type*)Base_legM[2]); // 左前腿
    Matrix.Copy((mtx_type*)kinLmLeg.MatrixOb[1], N, N, (mtx_type*)Base_legM[3]); // 左中腿
    Matrix.Copy((mtx_type*)kinLbLeg.MatrixOb[1], N, N, (mtx_type*)Base_legM[4]); // 左后腿
    Matrix.Copy((mtx_type*)kinRbLeg.MatrixOb[1], N, N, (mtx_type*)Base_legM[5]); // 右后腿

    // 计算每条腿根部到基座的逆矩阵
    Matrix.Invert((mtx_type*)Base_legM[0], N); // 右中腿
    Matrix.Invert((mtx_type*)Base_legM[1], N); // 右前腿
    Matrix.Invert((mtx_type*)Base_legM[2], N); // 左前腿
    Matrix.Invert((mtx_type*)Base_legM[3], N); // 左中腿
    Matrix.Invert((mtx_type*)Base_legM[4], N); // 左后腿
    Matrix.Invert((mtx_type*)Base_legM[5], N); // 右后腿

    // 计算脚到腿根部的变换矩阵：Leg_footM = inv(Base_legM) * Base_footM
    Matrix.Multiply((mtx_type*)Base_legM[0], (mtx_type*)Base_footM[0], N, N, N, (mtx_type*)Leg_footM[0]); // 右中腿
    Matrix.Multiply((mtx_type*)Base_legM[1], (mtx_type*)Base_footM[1], N, N, N, (mtx_type*)Leg_footM[1]); // 右前腿
    Matrix.Multiply((mtx_type*)Base_legM[2], (mtx_type*)Base_footM[2], N, N, N, (mtx_type*)Leg_footM[2]); // 左前腿
    Matrix.Multiply((mtx_type*)Base_legM[3], (mtx_type*)Base_footM[3], N, N, N, (mtx_type*)Leg_footM[3]); // 左中腿
    Matrix.Multiply((mtx_type*)Base_legM[4], (mtx_type*)Base_footM[4], N, N, N, (mtx_type*)Leg_footM[4]); // 左后腿
    Matrix.Multiply((mtx_type*)Base_legM[5], (mtx_type*)Base_footM[5], N, N, N, (mtx_type*)Leg_footM[5]); // 右后腿

    // 提取脚到腿根部的坐标（齐次矩阵的最后一列）
    for (int i = 0; i < 6; i++) {
      Endpose_3[i][0][0] = Leg_footM[i][0][3]; // X坐标
      Endpose_3[i][1][0] = Leg_footM[i][1][3]; // Y坐标
      Endpose_3[i][2][0] = Leg_footM[i][2][3]; // Z坐标
      Endpose_3[i][3][0] = 1.0;                // 齐次坐标
    }

    // 根据坐标逆解关节变量
    // 右中腿
    float zuobiao1[3];
    zuobiao1[0] = Endpose_3[0][0][0]; // X坐标
    zuobiao1[1] = Endpose_3[0][1][0]; // Y坐标
    zuobiao1[2] = Endpose_3[0][2][0]; // Z坐标
    ik4(zuobiao1); // 逆运动学计算
    robot.Servo_r[0] = theta_r[0] * 180.0 / PI; // 关节1角度（弧度转角度）
    robot.Servo_r[1] = theta_r[1] * 180.0 / PI; // 关节2角度
    robot.Servo_r[2] = theta_r[2] * 180.0 / PI; // 关节3角度

    // 右前腿
    zuobiao1[0] = Endpose_3[1][0][0];
    zuobiao1[1] = Endpose_3[1][1][0];
    zuobiao1[2] = Endpose_3[1][2][0];
    ik4(zuobiao1);
    robot.Servo_r[3] = theta_r[0] * 180.0 / PI;
    robot.Servo_r[4] = theta_r[1] * 180.0 / PI;
    robot.Servo_r[5] = theta_r[2] * 180.0 / PI;

    // 左前腿
    zuobiao1[0] = Endpose_3[2][0][0];
    zuobiao1[1] = Endpose_3[2][1][0];
    zuobiao1[2] = Endpose_3[2][2][0];
    ik4(zuobiao1);
    robot.Servo_r[6] = theta_r[0] * 180.0 / PI;
    robot.Servo_r[7] = theta_r[1] * 180.0 / PI;
    robot.Servo_r[8] = theta_r[2] * 180.0 / PI;

    // 左中腿
    zuobiao1[0] = Endpose_3[3][0][0];
    zuobiao1[1] = Endpose_3[3][1][0];
    zuobiao1[2] = Endpose_3[3][2][0];
    ik4(zuobiao1);
    robot.Servo_r[9] = theta_r[0] * 180.0 / PI;
    robot.Servo_r[10] = theta_r[1] * 180.0 / PI;
    robot.Servo_r[11] = theta_r[2] * 180.0 / PI;

    // 左后腿
    zuobiao1[0] = Endpose_3[4][0][0];
    zuobiao1[1] = Endpose_3[4][1][0];
    zuobiao1[2] = Endpose_3[4][2][0];
    ik4(zuobiao1);
    robot.Servo_r[12] = theta_r[0] * 180.0 / PI;
    robot.Servo_r[13] = theta_r[1] * 180.0 / PI;
    robot.Servo_r[14] = theta_r[2] * 180.0 / PI;

    // 右后腿
    zuobiao1[0] = Endpose_3[5][0][0];
    zuobiao1[1] = Endpose_3[5][1][0];
    zuobiao1[2] = Endpose_3[5][2][0];
    ik4(zuobiao1);
    robot.Servo_r[15] = theta_r[0] * 180.0 / PI;
    robot.Servo_r[16] = theta_r[1] * 180.0 / PI;
    robot.Servo_r[17] = theta_r[2] * 180.0 / PI;

    // 打印所有舵机角度
    for (int i = 0; i < 18; i++) {
      Serial.print("S");
      Serial.print(i);
      Serial.print(" :");
      Serial.print(robot.Servo_r[i]);
      Serial.print(" ");
    }

    // 将舵机角度写入舵机
    robot.framewrite(robot.Servo_r);

    // 延时20ms，控制步态更新频率
    delay(20);
  }

  // 完成一个周期后暂停2秒
  delay(2000);
}