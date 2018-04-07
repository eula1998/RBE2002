/*
 * Robot.cpp
 *  The implementation on controlling different parts of the robot;
 *  contain reference to all electronic parts except for the gripper, the rack, and the gyro
 *
 *  Created on: Feb 24, 2018
 *      Author: Lingrui
 */

#include "Robot.h"
#include "Arduino.h"

static int rmotorpinF = 5; //forward pin
static int rmotorpinB = 6; //backward pin

static int lmotorpinF = 7; //forward pin
static int lmotorpinB = 8; //backward pin

Robot::Robot() :
    motorright(rmotorpinF, rmotorpinB), motorleft(lmotorpinF, lmotorpinB){
}

Robot::~Robot() {
  // TODO Auto-generated destructor stub
}

/**
 * values [-255, 255]
 * positive means forward, and negative means backward
 */
void Robot::drive(int leftspeed, int rightspeed) {
  this->motorleft.drive(-leftspeed);
  this->motorright.drive(rightspeed);
}

/**
 * This function leaves one light sensor on the black line
 * assuming that the target degree is a multiple of 90, or along the black lines
 * theoretically the fix with a straight movement function call
 */

bool Robot::turn(int degree, bool CCW, int maxspeed, double currentHeading) {
  int buffer = 10;
  if (CCW) {
    if (abs(currentHeading) < (degree)) {
      drive(-maxspeed, maxspeed);
      Serial.print("rotating: ");
      Serial.println(currentHeading);
    } else {
        drive(0, 0);
        return true;
    }
  } else { //CW
    if (currentHeading < degree) {
      drive(maxspeed, -maxspeed); //pid control?
      Serial.print("rotating: ");
    } else {
        drive(0, 0);
        return true;
    }
  }
  return false;
}
