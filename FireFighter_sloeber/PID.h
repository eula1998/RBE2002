/*
 * PID.h
 *  An interface intended for controlling the Pololu motors with a
 *    pololu motor controller with forward and reverse current pin
 *
 *  Created on: April 14, 2018
 *      Author: Lingrui
 */

#ifndef PID_H_
#define PID_H_

#define ARRAY_SIZE (20)

class PID {
public:
  PID(double p, double i, double d);
  virtual ~PID();

  
  double calc(double setVel, double curVel, int max);
private:
  double kp, ki, kd;
  double ki_sum;
  double lasterror;

  int index; 
  double pasterror[ARRAY_SIZE]; 
};

#endif /* POLOLUMOTOR_H_ */
