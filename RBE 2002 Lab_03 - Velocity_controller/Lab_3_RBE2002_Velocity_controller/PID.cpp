/*
 * Messages.h
 *
 *  Created on: 10/1/16
 *      Author: joest
 */
#include "PID.h"
#include "Arduino.h"

//Class constructor
PID::PID(){
  index = 0;
  ki_sum = 0.0;
}

//Function to set PID gain values
void PID::setpid(float P, float I, float D){
  kp=P;
  ki=I;
  kd=D;
}

static double sum(double a[], int s){
//a = array, s = size
  double total = 0.0; 
  for (int x = 0; x < s; x++){
    total += a[x];
  }
  return total;
}

//Write this function to calculate a control signal from the set velocity 
//and the current velocity 
float PID::calc(double setVel, double curVel){

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
    float output_velocity = (kp*error + ki*integral + kd*derivative);
//    Serial.print("Kp value:");
//    Serial.println(kp*error);
//    Serial.print("Ki value:");
//    Serial.println(ki*integral);
//    Serial.print("Kd value:");
//    Serial.println(kd*derivative);
    float control_signal = (output_velocity / 1200.0) * 255.0; //full speed at 12V
    
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
