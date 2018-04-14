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

class PID {
public:
  PID(double p, double i, double d);
  virtual ~PID();
  /**
   * [-255, 255] backward -> forward
   */
  double calc(double setVel, double curVel);
private:
  double kp, ki, kd;
  double ki_sum;

  double lasterror;
};

#endif /* POLOLUMOTOR_H_ */
