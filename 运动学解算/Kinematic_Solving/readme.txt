Kinematic_Solving.ino 代码模块详解
1. 核心功能概述
该代码用于六足机器人的运动学计算与控制，实现以下功能：
  正运动学：根据关节角度计算机器人末端位置
  逆运动学：根据目标末端位置反解关节角度
  步态生成：生成摆线轨迹控制机器人行走
  舵机控制：将关节角度转换为舵机信号，驱动机器人运动

2. 头文件与依赖库
关键库：
  Robot.h：机器人底层控制（舵机驱动、初始姿态配置）
  Kinematics.h：运动学计算类，封装正/逆运动学算法
  MatrixMath.h：矩阵运算库（矩阵乘法、求逆等）
数学常量：
  pi4：预定义 π/4 弧度（45°），用于简化角度计算

3. 机械参数定义
结构参数（单位：mm）：
  #define l1 34      // 大腿连杆长度
  #define l2 43.51   // 小腿第一段长度
  #define l3 93.07   // 小腿第二段长度
  #define ll 94.937  // 机身斜边半长（侧腿安装位置）
  #define lm 50.8    // 机身中间横梁半长（中腿安装位置）
步态参数：
  float stepx = 5;  // X方向步长（水平移动距离）
  float stepz = 10; // Z方向抬腿高度

4. 运动学模型初始化
MDH 参数配置：
每条腿（右中、右前、左前等）通过 Kinematics 类实例化，配置 5 个关节的 MDH 参数：
参数含义：a（连杆长度）、d（连杆偏移）、α（连杆扭转角）、θ（关节初始角度）
  kinRmLeg.AddJointMDH(0.0, 0.0, -PI/2, 0.0); // 关节1：a=0, d=0, α=-90°, θ=0
舵机引脚与转向配置：
  robot.Jointservo[0] = 6;  // 右中腿关节1对应引脚6
  robot.direct[0] = 1;      // 舵机转向（1为正，-1为反）

5. 核心函数解析
5.1 正运动学计算
  forwardMDH()： //计算从基座到末端的齐次变换矩阵，存储在 MatrixOb 数组中
  例：kinRmLeg.forwardMDH() //计算右中腿的正运动学

5.2 逆运动学计算
  ik4(float zuobiao[3])： //输入目标末端坐标 [x, y, z]，通过几何法解析关节角度 theta_r[3]
 
计算步骤：
  计算 θ₁（绕 Z 轴旋转角）：atan(y/x)
  计算 θ₃（肘关节角）：基于余弦定理
  计算 θ₂（肩关节角）：结合几何关系与正弦定理

5.3 坐标变换与末端位置计算
  CalEndpose_3()： //计算脚相对于腿根部的坐标（Endpose_3 数组）
核心步骤：
  获取腿根部到基座的变换矩阵 Base_legM
  求逆矩阵 inv(Base_legM)
  计算腿根部到脚的变换矩阵：Leg_footM = inv(Base_legM) * Base_footM
  提取坐标值并调用逆运动学

5.4 步态生成与控制
  RobotClass_forward_k()： //生成摆线轨迹：X 方向为摆线运动，Z 方向为抛物线运动，更新各腿末端位置，通过逆运动学计算舵机角度，并写入 robot.Servo_r

6. 主程序流程
6.1 setup() 函数
  初始化串口通信（Serial.begin(115200)
  配置各腿的 MDH 参数和舵机引脚
  设置舵机初始角度并移动到零点位置

6.2 loop() 函数
正运动学计算：
  kinRmLeg.forwardMDH(); // 右中腿正运动学
轨迹生成：
循环 20 次生成摆线轨迹，更新末端坐标：
  float x = stepx * (t - sin(t)); // X方向摆线
  float z = stepz * (1 - cos(t)); // Z方向抬腿
逆运动学解算：
  调用 ik4() 计算每条腿的关节角度，并转换为舵机角度（弧度 → 角度）
舵机控制：
  将角度写入舵机：robot.framewrite(robot.Servo_r)
  控制频率：delay(20) 实现 50Hz 更新

7. 调试与输出
打印函数：
  printRobotFootPoint()：打印所有腿的末端坐标。
  printLegFootpose()：打印腿根部到脚的坐标。
串口输出：
  Serial.print("S0 :"); Serial.print(robot.Servo_r[0]); // 打印舵机角度
