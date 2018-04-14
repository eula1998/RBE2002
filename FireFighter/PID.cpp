/*
 * PID.cpp
 *
 *  Created on: April 14, 2018
 *      Author: Lingrui
 */

#include "PID.h"
#include "Arduino.h"

PID::PID(double p, double i, double d)
:kp(p), ki(i),  kd(d)
{
  ki_sum = 0;
  lasterror = 0;
}

PID::~PID() {
  // TODO Auto-generated destructor stub
}

//Write this function to calculate a control signal from the set velocity 
//and the current velocity 
//returns a range of [0, 255]
double PID::calc(double setVel, double curVel){
    // calculate error
    double error = setVel - curVel; 
        
    // calculate derivative of error
    double derivative = error - lasterror; 
    lasterror = error; 
    
    // calculate integral error. Running average is best but hard to implement
//    pasterror[index] = error; 
//    index = (index+1)%ARRAY_SIZE; 
//    double integral = sum(pasterror, ARRAY_SIZE); 
    ki_sum += error; 
    double integral = ki_sum;



    // sum up the error value to send to the motor based off gain values. 
    float control_signal = (kp*error + ki*integral + kd*derivative);
//    Serial.print("p value:");
//    Serial.println(kp*error);
//    Serial.print("i value:");
//    Serial.println(ki*integral);
//    Serial.print("d value:");
//    Serial.println(kd*derivative);
//    float control_signal = (output_velocity / 1200.0) * 255.0; //full speed at 12V

    // limit control value to 0-254
    if (control_signal > 254){
      control_signal = 254; 
    }else if (control_signal < 0){
      control_signal = 0;
    }
 
    //return the control signal
    return control_signal;
//    return 255;
}
