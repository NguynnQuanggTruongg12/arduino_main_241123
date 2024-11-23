/*
  Motor.h - Library for working with the Cytron SPG30E-30K.
  Created by Vinay Lanka, January 27, 2021.
*/
#ifndef Motor1_h
#define Motor1_h

#include "Arduino.h"

class Motor1 {
  public:
    //Constructor - Plus and Minus are the Motor output / en_a and en_b are the encoder inputs
    Motor1(int R_EN1, int L_EN1, int R_PWM1, int L_PWM1);
    //Spin the motor with a percentage value
    void rotate1(int value);
    //Motor Outputs - plus is one direction and minus is the other
    int R_EN1;
    int L_EN1;
    int R_PWM1;
    int L_PWM1;

};

#endif
