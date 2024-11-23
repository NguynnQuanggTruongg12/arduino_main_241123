/*
  Motor.h - Library for working with the Cytron SPG30E-30K.
  Created by Vinay Lanka, January 27, 2021.
*/
#ifndef Motor2_h
#define Motor2_h

#include "Arduino.h"

class Motor2 {
  public:
    //Constructor - Plus and Minus are the Motor output / en_a and en_b are the encoder inputs
    Motor2(int R_EN2, int L_EN2, int R_PWM2, int L_PWM2);
    //Spin the motor with a percentage value
    void rotate2(int value);
    //Motor Outputs - plus is one direction and minus is the other
    int R_EN2;
    int L_EN2;
    int R_PWM2;
    int L_PWM2;

};

#endif
