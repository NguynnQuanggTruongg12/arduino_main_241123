#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/time.h>
#include <std_msgs/String.h>

//#include "Motor1.h"
//#include "Motor2.h"
#include <PID_v1.h>
#define LOOPTIME 10
#define MANUAL_MODE 1
#define AUTO_MODE 0
#define LOCKED_MODE 2
#define UNLOCKED_MODE 5

#define led A15

typedef struct {
  byte  R_EN;
  byte L_EN;
  byte L_PWM;
  byte R_PWM;
  byte ENCODE_CA;
  byte ENCODE_CB;
} motor_data;


ros::NodeHandle nh;

float Vx = 0, Vx_out = 0,preVx=0, preVw=0,Vxpro=0,Vwpro=0;
// float Vx=0;
// float Vw=0;
float alphaX = 0, alphaW = 0;
float Vw = 0, Vw_out = 0;
int mode = 0,cntVx=0,cntVw=0;
motor_data motorset1, motorset2;
double v1;
double v2;
unsigned long prevMillis;

//-----------------------
float u_in_2, u_in_1 = 0;
float ev_i_store_2 = 0, ev_i_store_1 = 0;
float v_kalman_2 = 0, v_kalman_1 = 0, P_v_kalman_2 = 1, P_v_kalman_1 = 1;
float _previousTime, _currentTime, _eT;
float _dT = 0.01;
bool motorBlocked = false;

int internet=0, status = 0;

unsigned long currentMillis;
volatile long int pos2 = 0, pos1 = 0;


float eir = 0;

//-------------------------

void cmd_vel_cb(const geometry_msgs::Twist& twist) {
  Vx = twist.linear.x;
  //Vw = twist.angular.z ;
  mode = twist.linear.y;
  Vw = 0.2 * twist.angular.z;
  //    if (Vw == -1 or Vw == 1)/ Vw = 0.5;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb);
geometry_msgs::Vector3Stamped speed_msg;
ros::Publisher speed_pub("speed", &speed_msg);

//char buffer[50];
//
//std_msgs::String str_msg;
//ros::Publisher pub("chatter", &str_msg);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(speed_pub);
  motorBlocked = false;

  ev_i_store_2 = 0; P_v_kalman_2 = 0;
  ev_i_store_1 = 0; P_v_kalman_1 = 0;

  cau_hinh_motor1();


  pinMode(motorset1.ENCODE_CA, INPUT);
  pinMode(motorset1.ENCODE_CB, INPUT);
  pinMode(motorset2.ENCODE_CA, INPUT);
  pinMode(motorset2.ENCODE_CB, INPUT);

  pinMode(motorset1.R_EN,OUTPUT);
  pinMode(motorset1.L_EN,OUTPUT);
  pinMode(motorset1.R_PWM,OUTPUT);
  pinMode(motorset1.L_PWM,OUTPUT);

  pinMode(motorset2.R_EN,OUTPUT);
  pinMode(motorset2.L_EN,OUTPUT);
  pinMode(motorset2.R_PWM,OUTPUT);
  pinMode(motorset2.L_PWM,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(motorset2.ENCODE_CA), ReadEncoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(motorset1.ENCODE_CA), ReadEncoder1, RISING);

  Serial.begin(57600);
  Serial1.begin(115200);   // Giao tiếp với ESP8266 qua Serial1 (TX1, RX1)
  pinMode(led,OUTPUT);
  digitalWrite(led,0);
  Serial.println("====READY====");
  while (!Serial) {
    ; // đợi cổng serial kết nối. Chỉ cần thiết cho cổng USB gốc
  }
  delay(500);
  _currentTime = micros();
}
void nhan_uart_esp8266(){
    // Biến chứa dữ liệu đọc được
  static String dataString = "";

  // Nếu có dữ liệu từ Serial1 (ESP8266)
  while (Serial1.available()) {
    char c = Serial1.read();  // Đọc từng ký tự
    if (c == '\n') {  // Kết thúc chuỗi khi gặp ký tự xuống dòng
      int firstSeparator = dataString.indexOf(';');
      int secondSeparator = dataString.indexOf(';', firstSeparator + 1);
      int thirdSeparator = dataString.indexOf(';', secondSeparator + 1);
      
      if (firstSeparator != -1 && secondSeparator != -1 && thirdSeparator != -1) {
        // Convert the substrings into floating-point numbers and integers
        Vx = dataString.substring(0, firstSeparator).toFloat();
        Vw = dataString.substring(firstSeparator + 1, secondSeparator).toFloat();
//        internet = dataString.substring(secondSeparator + 1, thirdSeparator).toInt();
        status = dataString.substring(thirdSeparator + 1).toInt();

//        if (internet == 1) {
//          // Serial.println("Có internet");
//          digitalWrite(led,1);  // "Có internet" means "Internet available"
//        } else {
//          // Serial.println("No internet");
//          digitalWrite(led,1);
//          delay(2000);
//          digitalWrite(led,0);
//          delay(500);
//        }
      }
              Serial.print("Vx: ");
        Serial.print(Vx);
        Serial.print(" Vw: ");
        Serial.print(Vw);
        Serial.print(" Status: ");
        Serial.println(status);
      dataString = "";  // Reset chuỗi dữ liệu để đọc lần tiếp theo
    } else {
      dataString += c;  // Ghép từng ký tự vào chuỗi
    }
  }
}

void loop() {
  // nhan_uart_esp8266();
  // Vx=0.1;

  _previousTime = _currentTime;
  _currentTime = micros();
  _eT = (_currentTime - _previousTime) / 1000000.0; // Chia cho 1000 để lấy giây
  nhan_uart_esp8266();
  // Serial.println(Vx);

  if (_eT < 0) 
    _eT = _dT;

  while (_eT < _dT) {
    _currentTime = micros();
    _eT = (_currentTime - _previousTime) / 1000000.0;
  }
  int p_pos2_current = get_Encoder2();
  int p_pos1_current = get_Encoder1();
  //------------------------------------------------------------------------------------------------------

  //        float vel_posL_rpm = (((float)(p_posL_current - p_posL) / _eT) / 691.2) * 60;         //  xung/s  chia   xung/vòng  --> vòng/phút
  //        float vel_posR_rpm = (((float)(p_posR_current - p_posR) / _eT) / 691.2) * 60;

//====###############################################OK
  // float vel_pos2_rpm = (( ((float)(p_pos2_current)) / _eT) / 2688) * 60;         //  xung/s  chia   xung/vòng  --> vòng/phút                 xung/vong = 19.2 * 12 * tile pully
  // float vel_pos2_mps = (vel_pos2_rpm / 60) / (1 / (3.14 * 0.145));  // vòng/s   chia    vòng/mét  --> m/s
  // float vel_pos1_mps = (vel_pos1_rpm / 60) / (1 / (3.14 * 0.145));

//#############################################################
// Pulses per wheel revolution (after gearbox)
//const float pulses_per_wheel_rev = 2688 * 168 *3;
//const long pulses_per_wheel_rev = (long)2688 * (long)168;
//
//
//float vel_pos2_rpm = (((float)(p_pos2_current) / _eT) / pulses_per_wheel_rev) * 60;  // Motor 2 RPM at the wheel
//float vel_pos1_rpm = (((float)(p_pos1_current) / _eT) / pulses_per_wheel_rev) * 60;  // Motor 1 RPM at the wheel
//
//// Convert RPM to m/s with a 1:3 pulley ratio and a wheel diameter of 0.145m
//// float vel_pos2_mps = (vel_pos2_rpm / 60) * (3.14 * 0.145 / 3);  // Motor 2 speed in m/s
//// float vel_pos1_mps = (vel_pos1_rpm / 60) * (3.14 * 0.145 / 3);  // Motor 1 speed in m/s
//
//float vel_pos2_mps = (vel_pos2_rpm / 60) * (3.14 * 0.1 );  // Motor 2 speed in m/s
//float vel_pos1_mps = (vel_pos1_rpm / 60) * (3.14 * 0.1 );  // Motor 1 speed in m/s




//const long pulses_per_wheel_rev = (long)168 * (long)12;
//
//float vel_pos2_rpm = (((float)(p_pos2_current) / _eT) / pulses_per_wheel_rev) * 60;  // Motor 2 RPM at the wheel
//float vel_pos1_rpm = (((float)(p_pos1_current) / _eT) / pulses_per_wheel_rev) * 60;  // Motor 1 RPM at the wheel
//
//// Tính toán tốc độ tuyến tính với tỷ lệ truyền 1:3
//float ratio = 1.0 / 3.0;  // Tỷ lệ truyền
//float diameter = 0.1;  // Đường kính bánh xe (m)
//float circumference = 3.14 * diameter;  // Chu vi bánh xe (m)
//
//float vel_pos2_mps = (vel_pos2_rpm * ratio / 60) * circumference;  // Motor 2 speed in m/s
//float vel_pos1_mps = (vel_pos1_rpm * ratio / 60) * circumference;  // Motor 1 speed in m/s




//const long pulses_per_wheel_rev = (long)2688 / (long)3;  // Tổng xung cho một vòng quay bánh xe
//
//float vel_pos2_rpm = (((float)(p_pos2_current) / _eT) / pulses_per_wheel_rev) * 60;  // Motor 2 RPM at the wheel
//float vel_pos1_rpm = (((float)(p_pos1_current) / _eT) / pulses_per_wheel_rev) * 60;  // Motor 1 RPM at the wheel
//
//// Tính toán tốc độ tuyến tính với tỷ lệ truyền 1:3
//float ratio = 1.0 / 3.0;  // Tỷ lệ truyền từ động cơ sang bánh xe
//float diameter = 0.1;  // Đường kính bánh xe (m)
//float circumference = 3.14 * diameter;  // Chu vi bánh xe (m)
//
//float vel_pos2_mps = (vel_pos2_rpm * ratio / 60) * circumference;  // Motor 2 speed in m/s
//float vel_pos1_mps = (vel_pos1_rpm * ratio / 60) * circumference;  // Motor 1 speed in m/s






// if (vel_pos2_mps != vel_pos1_mps){
//   if (vel_pos2_mps > vel_pos1_mps){
//     vel_pos1_mps = vel_pos1_mps + 0.01;
//   }
//   else{
//     vel_pos1_mps = vel_pos1_mps - 0.01;
//   }
// }
//if (vel_pos2_mps != vel_pos1_mps){
//  if (vel_pos2_mps > vel_pos1_mps){
//    vel_pos1_mps = vel_pos1_mps + 0.01;
//  }
//  else{
//    vel_pos1_mps = vel_pos1_mps - 0.01;
//  }
//  if (vel_pos2_mps < vel_pos1_mps){
//    vel_pos2_mps = vel_pos2_mps + 0.01;
//  }
//  else{
//    vel_pos2_mps = vel_pos2_mps - 0.01;
//  }
//}


  float vel_pos2_rpm = (( ((float)(p_pos2_current)) / _eT) / 6048) * 60;         //  xung/s  chia   xung/vòng  --> vòng/phút                 xung/vong = 19.2 * 12 * tile pully
  float vel_pos1_rpm = (( ((float)(p_pos1_current)) / _eT) / 6048) * 60;         // 128 * 12 *3

  float vel_pos2_mps = (vel_pos2_rpm / 60) / (1 / (3.14 * 0.1));  // vòng/s   chia    vòng/mét  --> m/s
  float vel_pos1_mps = (vel_pos1_rpm / 60) / (1 / (3.14 * 0.1));


  //---------------------------------------- LỌC KALMAN -----------------------------------------------
  
  v_kalman_1 = kalmanFilter_bac1(v_kalman_1, vel_pos1_mps, &P_v_kalman_1, 0.1, 5); //mps
  v_kalman_2 = kalmanFilter_bac1(v_kalman_2, vel_pos2_mps, &P_v_kalman_2, 0.1, 5); //mps
//   Vx = 1; Vw = 0;
  float vxm = 0.5 * (v_kalman_2 + v_kalman_1);
  float vwm = -(v_kalman_1 - v_kalman_2);
  float vxc = 0.7 * Vx + 0.7 * (Vx - vxm);
  float vwc = 0.7 * Vw + 0.7* (Vw - vwm);

  //---------------------------------------------- BỘ ĐIỀU KHIỂN PID ------------------------------------------------
  // int vd = 30;
  /*------------- Tính toán V từ Ros------------------*/
  if (status == 1) //Manual mode
  { alphaX = 0.01; alphaW = 0.01;
//   Serial.println(alphaX);
  }
  else //Automode
  {
    alphaX = 0.001 / (0.1+ fabs(Vx)); //0.01
    alphaW = 0.001 / (0.01 + fabs(Vw)); //0.01
//    Serial.println(alphaX);
  }

  Vx_out = (1 - alphaX) * Vx_out + alphaX * vxc;
  Vw_out = (1 - alphaW) * Vw_out + alphaW * vwc;

  double motor2Speed = (Vx_out + Vw_out); // Động cơ trái
  double motor1Speed = (Vx_out - Vw_out); // Động cơ phải

  // motorLSpeed = -0.4; motorRSpeed = 0.2;
  /*--------------------------------*/
  if(mode == UNLOCKED_MODE)
    motorBlocked = false;
  if(mode == LOCKED_MODE)
    motorBlocked = true;
  if(motorBlocked)
  {
    motor2Speed = 0; motor1Speed = 0; //DUNG DONG CO
  }

  u_in_1 = PI_VEL_CONTROLLER(motor1Speed, v_kalman_1, &ev_i_store_1, 400, 20, _dT, 255, -255);
  u_in_2 = PI_VEL_CONTROLLER(motor2Speed, v_kalman_2, &ev_i_store_2, 400, 20, _dT, 255, -255); //100 2     350   30
  



  float dead_zone_dung = 0.03;

  if ((motor2Speed < dead_zone_dung && motor2Speed > -dead_zone_dung) && (v_kalman_2 < dead_zone_dung && v_kalman_2 > -dead_zone_dung)) {
  ev_i_store_2 = 0.9*ev_i_store_2;
  }
  if ((motor1Speed < dead_zone_dung && motor1Speed > -dead_zone_dung) && (v_kalman_1 < dead_zone_dung && v_kalman_1 > -dead_zone_dung)) {
   ev_i_store_1 = 0.9*ev_i_store_1;
  }



 publishSpeed(vxm, vwm, Vx);

//   u_in_1=250;
//   u_in_2=250;
//Serial.println(status);
//  Serial.print("vx: "); Serial.print(v_kalman_1); Serial.print("          vw: "); Serial.println(v_kalman_2); //Serial.print("   "); Serial.println(alphaX);
//Serial.println(p_pos1_current);
  rotate_motor(2, u_in_2);
  //       right.rotate1(u_in_R);  // R
  rotate_motor(1, u_in_1);


  //        prevMillis = currentMillis;
  //    }

  nh.spinOnce();
}
