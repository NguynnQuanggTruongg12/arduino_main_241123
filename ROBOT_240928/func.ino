
void publishSpeed(float leftSpeed, float rightSpeed, float sent_data) {
  speed_msg.header.stamp = nh.now();
  speed_msg.vector.x = leftSpeed;
  speed_msg.vector.y = rightSpeed;
  speed_msg.vector.z = sent_data; // Không sử dụng

  speed_pub.publish(&speed_msg);
}
//void publishString(const char* str) {
//  str_msg.data = str;
//  pub.publish(&str_msg);
//}

float kalmanFilter_bac1(float x_pre, float x_meas, float *P_in, float q_in, float r_in) {
    //Mô hình: xk_1 = xk + Q; z = xk_1 + R
    float xk, k, p = *P_in;
    //Dự báo
    xk = x_pre;
    p = p + q_in;

    //Hiệu chỉnh (correction)
    k = p / (p + r_in);
    xk = xk + k * (x_meas - xk);
    p = (1 - k) * p;

    //xuất ngõ ra
    *P_in = p;
    return xk;
}

float PI_VEL_CONTROLLER(float vd, float v, float *ev_i, float kp, float ki, float dt, float u_max, float u_min) {
  float ev = vd - v;
  float ei = (*ev_i) + 0.01 * ev;
  eir = ei;
  if (ei > 5) ei = 5;
  else if (ei < -5) ei = -5;
  float out = kp * ev + ki * ei + 400 * vd;
  //giới hạn dữ liệu
  out = (out > u_max) ? u_max : ((out < u_min) ? u_min : out);
  //lưu dữ liệu
  *ev_i = ei;
  return out;
}

void ReadEncoder2() {
 // Serial.print("test2");
  if (digitalRead(motorset2.ENCODE_CB) == 0) pos2 += 1;
  else pos2 -= 1;
}

int get_Encoder2() {
  int temp1 = pos2;
  pos2 = 0;
  return temp1;
}

void reset_Encoder2() {
  pos2 = 0;
}

void reset_Encoder1() {
  pos1 = 0;
}

void ReadEncoder1() {
  //Serial.print("test3");
  if (digitalRead(motorset1.ENCODE_CB) == 1) pos1 -= 1;
  else pos1 += 1;
}

int get_Encoder1() {
  int temp1 = pos1;
  pos1 = 0;
  return temp1;
}

void cau_hinh_motor1()
{
  motorset1.L_EN = L_EN1; motorset1.R_EN = R_EN1; motorset1.ENCODE_CA = ENCODER1_CA; motorset1.ENCODE_CB = ENCODER1_CB; motorset1.L_PWM = L_PWM1; motorset1.R_PWM = R_PWM1;//CAU HINH CHAN CHO MOTOR 1
  motorset2.L_EN = L_EN2; motorset2.R_EN = R_EN2; motorset2.ENCODE_CA = ENCODER2_CA; motorset2.ENCODE_CB = ENCODER2_CB; motorset2.L_PWM = L_PWM2; motorset2.R_PWM = R_PWM2;//CAU HINH CHAN CHO MOTOR 1
}
//------------XUAT XUNG DK MOTOR------------------


void rotate_motor(byte motorID, int value)
{
  if (motorID == 1)
  {
    // digitalWrite(motorset1.L_EN, HIGH);
    // digitalWrite(motorset1.R_EN, HIGH);
    if (value >= 0)
    {
      int out = (value > 255) ? 255 : value;
      analogWrite(motorset1.L_PWM, out);
      analogWrite(motorset1.R_PWM, 0);
    }
    else
    {
      int out = (value < -255) ? 255 : (-value);
      analogWrite(motorset1.L_PWM, 0);
      analogWrite(motorset1.R_PWM, out);
    }
  }
  else if (motorID == 2)
  {
    // digitalWrite(motorset2.R_EN, HIGH);
    // digitalWrite(motorset2.L_EN, HIGH);
    if (value >= 0)
    {
      int out = (value > 255) ? 255 : value;
      analogWrite(motorset2.L_PWM, 0);
      analogWrite(motorset2.R_PWM, out);
    }
    else
    {
      int out = (value < -255) ? 255 : (-value);
      analogWrite(motorset2.L_PWM, out);
      analogWrite(motorset2.R_PWM, 0);
    }
  }
}
