#include "Arduino.h"
#include "Motor2.h"


Motor2::Motor2(int R_EN2, int L_EN2, int R_PWM2, int L_PWM2) {
  pinMode(R_EN2,OUTPUT);
  pinMode(L_EN2,OUTPUT);
  pinMode(R_PWM2,OUTPUT);
  pinMode(L_PWM2,OUTPUT);
  
  Motor2::R_EN2 = R_EN2;
  Motor2::L_EN2 = L_EN2;
  Motor2::R_PWM2 = R_PWM2;
  Motor2::L_PWM2 = L_PWM2;

}

void Motor2::rotate2(int value) {
  if(value>=0){
    //Max Voltage with 16V battery with 12V required
    //(12/16)*255 ~=190
//    Serial.println("called");
//    Serial.println(plus);
    //int out = map(value, 0, 100, 0, 250);
    int out = (value > 255)?255:value;
    digitalWrite(R_EN2, HIGH);
    digitalWrite(L_EN2, HIGH); //clockwise
    analogWrite(R_PWM2, out);
    analogWrite(L_PWM2, 0);
    // analogWrite(plus,out);
    //digitalWrite(minus,LOW);
  }else{
    //Max Voltage with 16V battery with 12V required
    //(12/16)*255 ~=190
    //int out = map(value, 0, -100, 0, 250);
    int out = (value <-255)?255:(-value);
    digitalWrite(R_EN2, HIGH);
    digitalWrite(L_EN2, HIGH); //clockwise
    analogWrite(R_PWM2, 0);
    analogWrite(L_PWM2, out);
    // analogWrite(plus,out);
    // digitalWrite(minus,HIGH);
    //digitalWrite(plus,LOW);
  }
}
