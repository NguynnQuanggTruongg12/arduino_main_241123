#include "Arduino.h"
#include "Motor1.h"

Motor1::Motor1(int R_EN1,int L_EN1, int R_PWM1, int L_PWM1) {
  pinMode(R_EN1,OUTPUT);
  pinMode(L_EN1,OUTPUT);
  pinMode(R_PWM1,OUTPUT);
  pinMode(L_PWM1,OUTPUT);
  
  Motor1::R_EN1 = R_EN1;
  Motor1::L_EN1 = L_EN1;
  Motor1::R_PWM1 = R_PWM1;
  Motor1::L_PWM1 = L_PWM1;

}

void Motor1::rotate1(int value) {
  if(value>=0){
    //int out = map(value, 0, 100, 0, 250);
    int out = (value > 255)?255:value;
    digitalWrite(R_EN1, HIGH);
    digitalWrite(L_EN1, HIGH); 
    
    analogWrite(R_PWM1, 0);
    analogWrite(L_PWM1, out);
    
  }else{
    //Max Voltage with 16V battery with 12V required
    //(12/16)*255 ~=190
   // int out = map(value, 0, -100, 0, 250);
   int out = (value <-255)?255:(-value);
    digitalWrite(R_EN1, HIGH);
    digitalWrite(L_EN1, HIGH); //clockwise
    analogWrite(R_PWM1, out);
    analogWrite(L_PWM1, 0);
    // analogWrite(plus,out);
    // digitalWrite(minus,HIGH);
    //digitalWrite(plus,LOW);
  }
}
