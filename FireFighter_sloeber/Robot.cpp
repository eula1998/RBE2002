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

static int rencoder1 = 2;
static int rencoder2 = 3;

static int lencoder1 = 18;
static int lencoder2 = 19;

static int usf_in = 27, usf_out = 26;
static int usr_in = 29, usr_out = 28;



Robot::Robot() :
		motorright(rmotorpinF, rmotorpinB), motorleft(lmotorpinF, lmotorpinB), imuPID(
				17, 0.05, 15), rightEnc(rencoder1, rencoder2), leftEnc(
				lencoder1, lencoder2), usFront(usf_in, usf_out), usRight(usr_in,
				usr_out), y(0), x(0), stepper(25, 23), ideal_heading(0) {
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


bool Robot::turn(int degree, bool CCW, int maxspeed, double currentHeading) {
	int target_heading = ideal_heading + (degree * (CCW ? 1 : -1));
	int speed = imuPID.calc(target_heading, currentHeading, maxspeed);
//	Serial.print(", target, ");
//	Serial.print(target_heading);
//	Serial.print(", ideal, ");
//	Serial.print(ideal_heading);
//	Serial.print(", speed, ");
//	Serial.println(speed);

	if (abs(speed) > 20) {
		drive(-speed, speed);
	} else {
		drive(0, 0);
		ideal_heading = target_heading;
		return true;
	}

	return false;
}

bool Robot::driveDist(int speed, int distance) {
	int s = (distance - readLeft()) / distance * speed + 30; //30 is the base speed
	if (distance > readLeft()) {
		drive(s, s);
		return false;
	}

	drive(0, 0);
	return true;
}

RightAlign Robot::rightAlign(int maxspeed) {
	if (isFront()){
		drive(0, 0);
		return TURNLEFT;
	}else if(usRight.distanceRead() > 70){
		return TURNRIGHT;
	}else{
//		int s = (usFront.distanceRead() - 15) / 100 * maxspeed + 30;
		int s = maxspeed;
		//if deviated away from the wall, tilt right until within range again

		drive(s, s);
		return ALRIGHT;
	}
//	else if (usRight.distanceRead() < 5){
//
//	}


//	else if (isRight()){//need to change
////		int s = (usFront.distanceRead() - 15) / 100 * maxspeed + 30;
//		int s = maxspeed;
//		//if deviated away from the wall, tilt right until within range again
//
//		drive(s, s);
//		return ALRIGHT;
//	}else{
//		drive(0, 0);
//		return TURNRIGHT;
//	}
}

bool Robot::isRight(){
	return usRight.distanceRead() < 15; //also need line follower readings
}

bool Robot::isFront(){
	return usFront.distanceRead() < 15;//also need line follower readings
}

void Robot::setStepperAngle(int deg){
	stepper.turnTo((double)deg);
}

//==================================================
//===                ENCODER                     ===
//==================================================

/**
 * resets the encoder count
 */
void Robot::resetEnc() {
	rightEnc.write(0);
	leftEnc.write(0);
	delay(100);
}

double Robot::readLeft() {
	return (double) leftEnc.read() * encFactor;
//	return usFront.distanceRead();
}

double Robot::readRight() {
	return (double) rightEnc.read() * encFactor;
//	return usRight.distanceRead();
}
