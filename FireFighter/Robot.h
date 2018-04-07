/*
 * Robot.h
 *  An interface intended for controlling the Pololu motors with a
 *    pololu motor controller with forward and reverse current pin
 *
 *  Created on: Feb 24, 2018
 *      Author: Lingrui
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include "PololuMotor.h"

class Robot {
public:
  Robot();
  virtual ~Robot();
  
  void drive(int leftspeed, int rightspeed); 
  bool turn(int degree, bool CCW, int maxspeed, double currentHeading);
private:
  PololuMotor motorright;
  PololuMotor motorleft;
};

#endif /* POLOLUMOTOR_H_ */
