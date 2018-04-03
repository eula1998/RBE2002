/*
 * Messages.h
 *
 *  Created on: 10/1/16
 *      Author: joest
 */

#ifndef PID_H_
#define PID_H_

#define ARRAY_SIZE 10

class PID {
  
public:
PID();
float kp;
float ki;
float kd;
float last_error = 0;
float sum_error =0;
//set PID constants
void setpid(float P, float I, float D);
//calculate the output control signal
float calc(double setVel, double curVel);

private:
double pasterror[ARRAY_SIZE]; 
int index; 
int lasterror; 
double ki_sum;
};



#endif
